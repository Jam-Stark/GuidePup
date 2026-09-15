import { EventEmitter } from "node:events";
import { randomUUID } from "node:crypto";
import { mkdirSync, writeFileSync, readFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import type { Tool } from "@earendil-works/pi-ai";
import { PiDecision, type DecisionInput, type DecisionResult } from "./decision.js";
import { MonitorStore } from "./monitor-store.js";
import { SimClient, type SimStatus, type ObservationImage, type SceneFrame, type MotionSegment } from "./sim-client.js";
import { makePortal, portalClearance, isStopped, motionDelta, controlEventFact, navigationContract, bodyRadiusM,
  usableVerticalSurface, targetGeometry, waitingZoneConditions, boundaryRelation, mergeBoundaryEvidence, type TargetAnchor,
  type Observation, type Portal, type Region, type Layer, type BoundaryEvidence } from "./monitor-evidence.js";
import { type DecisionMemory, memoryContext } from "./decision-memory.js";
import * as tools from "./monitor-tools.js";

export interface MonitorOptions {
  logPath: string; dbPath: string; timeoutS: number; maxRecoveries: number; decisionTimeoutS: number;
}
export function defaultMonitorOptions(): MonitorOptions {
  return { logPath: resolve(".local/monitor/events.jsonl"), dbPath: resolve(".local/monitor/tasks.sqlite"),
    timeoutS: 360, maxRecoveries: 3, decisionTimeoutS: 30 };
}
export interface MonitorOutcome { status: "COMPLETED" | "NEEDS_USER" | "LIMIT_REACHED" | "ERROR" | "CANCELLED"; taskId: string; reason: string }
interface Attempt {
  maneuver: string; instruction: string; expectedEffect: string; controlTaskId: string | null;
  turnDegrees?: number;
  advanceMeters?: number;
  startPose: number[]; endPose?: number[]; result?: string; distanceM?: number; yawRad?: number;
}
interface Stage {
  kind: "pass_doorway" | "approach_target"; clause: string; target: string; instruction: string; distanceM?: number;
  targetId: string;
  status: "PENDING" | "ACTIVE" | "COMPLETED"; portal?: Portal; anchor?: TargetAnchor;
  boundaries?: BoundaryEvidence[];
  attempts: Attempt[]; unmet: string[]; evidenceIds: string[]; completionReason?: string;
}
interface Task {
  id: string; revision: number; originalGoal: string; status: string; stages: Stage[]; currentStage: number;
  instanceId: string; startedAt: number; pauseReason?: string;
  facts: unknown[]; claims: unknown[]; unknowns: string[];
  memory?: DecisionMemory;
}
type Kind = "plan" | "stage" | "verify" | "recover" | "dialogue";
interface Work {
  kind: Kind; serial: number; reason: string; text?: string;
  resolve?: (value: string) => void; reject?: (error: Error) => void;
}
export interface ProposalDriver { decide(input: DecisionInput): Promise<DecisionResult> }

/** Resident event processing and action authority; the model only returns proposals. */
export class TaskMonitor extends EventEmitter {
  private readonly id = randomUUID();
  private readonly store: MonitorStore;
  private readonly driver: ProposalDriver;
  private readonly artifactDir: string;
  private task?: Task;
  private control!: SimStatus;
  private stream = new AbortController();
  private listening?: Promise<void>;
  private heartbeatTimer?: ReturnType<typeof setInterval>;
  private taskTimer?: ReturnType<typeof setTimeout>;
  private heartbeatBusy = false;
  private heartbeatFailed = false;
  private queue: Work[] = [];
  private draining?: Promise<void>;
  private active?: AbortController;
  private serial = 0;
  private closed = false;
  private observationNumber = 0;
  private seenEvent = -1;
  private activeAttempt?: Attempt;
  private checkpointPending = false;
  private observation?: Observation;
  private interruptedWork?: { kind: Kind; reason: string };
  private currentWork?: Work;
  private outcome?: (outcome: MonitorOutcome) => void;

  constructor(readonly client: SimClient, readonly options: MonitorOptions, driver?: ProposalDriver) {
    super();
    this.store = new MonitorStore(options.dbPath, options.logPath);
    this.artifactDir = `${options.logPath.replace(/\.jsonl$/, "")}_observations`;
    mkdirSync(this.artifactDir, { recursive: true });
    this.driver = driver ?? new PiDecision((kind, data) => this.record(kind, data));
  }
  private record(kind: string, data: unknown): void {
    this.store.record(kind, data, this.task?.id, this.task?.revision);
  }
  private save(kind: string, data: unknown): void {
    if (!this.task) { this.record(kind, data); return; }
    this.store.saveTask(this.task, kind, data);
  }
  private say(text: string): void { this.record("user_message", { text }); this.emit("message", text); }
  snapshot(): unknown { return { task: this.task, control: this.control, decisionActive: Boolean(this.active), queued: this.queue.length, monitorId: this.id }; }
  private stage(): Stage | undefined { return this.task?.stages[this.task.currentStage]; }
  private live(): boolean { return Boolean(this.task && !["COMPLETED", "CANCELLED"].includes(this.task.status)); }

  async connect(): Promise<void> {
    this.control = await this.client.claim(this.id);
    this.record("monitor_connected", { control: this.control });
    const saved = this.store.loadLatest() as Task | undefined;
    if (saved) {
      this.task = saved;
      if (this.live()) {
        this.task.status = "WAITING_HUMAN";
        this.task.pauseReason = saved.instanceId === this.control.instance_id ? "process_restart" : "simulator_restarted";
        if (saved.instanceId !== this.control.instance_id) {
          this.task.unknowns.push("Simulator instance changed: old world-frame evidence is invalid; start a new task.");
          for (const stage of saved.stages) { delete stage.portal; delete stage.boundaries; }
        }
        this.save("task_restored", { control: this.control, reason: this.task.pauseReason });
      }
    }
    this.heartbeatTimer = setInterval(() => { void this.heartbeat(); }, 1000);
    this.listening = this.listen();
  }
  private async heartbeat(): Promise<void> {
    if (this.closed || this.heartbeatBusy || this.heartbeatFailed) return;
    this.heartbeatBusy = true;
    try { await this.client.heartbeat(this.id); }
    catch (error) {
      if (this.closed) return;
      this.heartbeatFailed = true;
      this.cancel("monitor lease failed");
      this.finish("ERROR", `控制服务心跳失败：${String(error)}`);
    } finally { this.heartbeatBusy = false; }
  }
  private async listen(): Promise<void> {
    try { for await (const status of this.client.events(this.stream.signal)) { if (this.closed) break; this.onControl(status); } }
    catch (error) {
      if (!this.closed) {
        this.cancel("SSE failed"); const serial = this.serial;
        await this.stopMotion("event_stream_failed");
        if (serial === this.serial) this.finish("ERROR", String(error));
      }
    }
  }
  private onControl(status: SimStatus): void {
    if (status.generation < this.control.generation && status.instance_id === this.control.instance_id) return;
    const previous = this.control;
    this.control = status;
    this.record("control_status", status);
    this.emit("control", status);
    if (!this.live()) return;
    if (status.instance_id !== this.task!.instanceId) {
      this.cancel("simulator instance changed"); this.finish("ERROR", "仿真实例已改变，需要重新开始任务。"); return;
    }
    const transition = status.event_id > this.seenEvent;
    this.seenEvent = Math.max(this.seenEvent, status.event_id);
    if (status.state === "HUMAN_PAUSED") {
      if (previous.state !== "HUMAN_PAUSED" || this.task!.status !== "WAITING_HUMAN") {
        this.endAttempt(status);
        this.cancel("human pause"); this.task!.status = "WAITING_HUMAN";
        this.task!.pauseReason = status.reason;
        this.save("human_paused", { control: status, currentStage: this.task!.currentStage });
        this.say(`人工暂停已生效，保留第 ${this.task!.currentStage + 1} 阶段。原因：${status.reason}。`);
      }
      return;
    }
    if (transition && status.state === "IDLE" && status.reason === "human_resume") {
      if (this.task!.pauseReason === "simulator_restarted") return;
      this.cancel("human resume"); this.task!.status = "RUNNING"; delete this.task!.pauseReason;
      this.save("human_resumed", { control: status, currentStage: this.task!.currentStage });
      this.schedule(this.task!.stages.length ? "stage" : "plan", "human_resume"); return;
    }
    if (this.task!.status !== "RUNNING") return;
    if (transition && ["ARRIVAL_CANDIDATE", "ERROR"].includes(status.state)
      || transition && status.state === "PAUSED" && ["mpc_blocked", "no_progress", "turn_checkpoint", "advance_checkpoint", "target_checkpoint"].includes(status.reason)) {
      this.endAttempt(status); this.cancel(`controller ${status.state}/${status.reason}`);
      this.schedule(status.state === "ARRIVAL_CANDIDATE" || ["advance_checkpoint", "target_checkpoint"].includes(status.reason) ? "verify"
        : status.reason === "turn_checkpoint" ? "stage" : "recover", status.reason || status.state);
      return;
    }
    const portal = this.stage()?.portal;
    const crossed = portal && portalClearance(portal, status.pose_world) >= navigationContract.doorway_exit_margin_m
      && portalClearance(portal, previous.pose_world) < navigationContract.doorway_exit_margin_m;
    const stage = this.stage();
    const geometry = stage?.anchor?.surface && targetGeometry(stage.anchor, status.pose_world);
    const reachedWaitingZone = stage?.distanceM === undefined && geometry && this.activeAttempt?.maneuver === "advance"
      && geometry.frontClearanceM <= navigationContract.waiting_zone.front_clearance_m[1];
    const latest = status.lightnav_latest as { visible: boolean | null; pointing: { opos_state?: string } | null } | null;
    const prior = previous.lightnav_latest as { visible: boolean | null } | null;
    const targetFound = stage?.kind === "approach_target" && !stage.anchor && this.activeAttempt?.maneuver === "advance"
      && this.activeAttempt.advanceMeters === undefined
      && latest?.visible === true && latest.pointing?.opos_state === "point";
    const targetLost = stage?.kind === "approach_target" && stage.anchor && this.activeAttempt?.maneuver === "advance"
      && this.activeAttempt.advanceMeters === undefined
      && latest?.visible === false && prior?.visible === true;
    if (status.state === "NAVIGATING" && !this.checkpointPending && (crossed || reachedWaitingZone || targetFound || targetLost)) {
      this.checkpointPending = true;
      const reason = crossed ? "doorway_checkpoint" : reachedWaitingZone ? "waiting_zone_checkpoint"
        : targetFound ? "target_visible" : "target_lost";
      this.cancel(reason);
      const serial = this.serial;
      void this.client.action(this.id, status.generation, "pause", undefined, reason).then(control => {
        if (serial !== this.serial || this.closed) return;
        this.control = control; this.endAttempt(control); this.schedule(crossed || reachedWaitingZone ? "verify" : "stage", reason);
      }).catch(error => { this.record("action_rejected", { error: String(error) }); }).finally(() => { this.checkpointPending = false; });
    }
  }
  private cancel(reason: string): void {
    this.serial++;
    this.active?.abort(new Error(reason));
    for (const work of this.queue) work.reject?.(new Error(`Cancelled: ${reason}`));
    this.queue = [];
    this.record("decisions_invalidated", { reason, serial: this.serial });
  }
  private schedule(kind: Kind, reason: string, extra: Partial<Work> = {}): void {
    this.queue.push({ kind, reason, serial: this.serial, ...extra });
    if (!this.draining) {
      this.draining = this.drain().finally(() => {
        this.draining = undefined;
        if (this.queue.length && !this.closed) {
          const next = this.queue.shift()!; this.schedule(next.kind, next.reason, next);
        }
      });
    }
  }
  private async drain(): Promise<void> {
    while (this.queue.length && !this.closed) {
      const work = this.queue.shift()!;
      if (work.serial !== this.serial) continue;
      this.active = new AbortController();
      this.currentWork = work;
      try { await this.handle(work, this.active.signal); }
      catch (error) {
        work.reject?.(error instanceof Error ? error : new Error(String(error)));
        if (work.serial === this.serial && !this.closed) {
          this.record("decision_failed", { purpose: work.kind, error: String(error) });
          await this.stopMotion("decision_failed");
          if (work.serial === this.serial) this.finish("NEEDS_USER", `决策未能执行，任务已暂停：${String(error)}`);
        } else this.record("stale_decision_discarded", { purpose: work.kind, error: String(error) });
      } finally { this.active = undefined; this.currentWork = undefined; }
    }
  }
  private check(work: Work, signal: AbortSignal): void {
    signal.throwIfAborted();
    if (work.serial !== this.serial || this.closed) throw new Error("Stale task decision");
  }
  async startGoal(goal: string): Promise<string> {
    if (!goal.trim()) throw new Error("Goal must not be empty");
    this.cancel("goal changed");
    this.interruptedWork = undefined;
    const serial = this.serial;
    if (this.outcome && this.task) this.outcome({ status: "CANCELLED", taskId: this.task.id, reason: "Goal replaced" });
    this.outcome = undefined;
    if (this.task && this.live()) { this.task.status = "CANCELLED"; this.save("task_replaced", { nextGoal: goal }); }
    if (this.task?.memory) this.record("decision_segment_archived", { ...this.task.memory, endReason: "goal_changed" });
    this.activeAttempt = undefined; this.observation = undefined;
    this.task = { id: randomUUID(), revision: 1, originalGoal: goal, status: "PLANNING", stages: [], currentStage: 0,
      instanceId: this.control.instance_id, startedAt: Date.now(), facts: [], claims: [], unknowns: [] };
    const task = this.task;
    const status = await this.client.status();
    if (serial !== this.serial) throw new Error("Goal request superseded before control invalidation");
    this.control = await this.client.action(this.id, status.generation, "invalidate", undefined, "goal_changed");
    if (serial !== this.serial) throw new Error("Goal request superseded after control invalidation");
    this.save("goal_changed", { originalGoal: goal, control: this.control });
    this.armDeadline();
    if (this.control.state === "HUMAN_PAUSED") {
      task.status = "WAITING_HUMAN"; task.pauseReason = this.control.reason;
      this.save("goal_waiting_human", { control: this.control });
    } else this.schedule("plan", "new_goal");
    return task.id;
  }
  async run(goal: string): Promise<MonitorOutcome> {
    await this.startGoal(goal);
    return new Promise(resolve => { this.outcome = resolve; });
  }
  private armDeadline(): void {
    clearTimeout(this.taskTimer);
    this.taskTimer = setTimeout(() => {
      this.cancel("task deadline");
      const serial = this.serial;
      void this.stopMotion("task_timeout").then(() => {
        if (serial === this.serial) this.finish("LIMIT_REACHED", "任务时间上限到达，进度保留，等待人工处理。");
      });
    }, this.options.timeoutS * 1000);
  }
  question(text: string): Promise<string> {
    if (!text.trim()) return Promise.reject(new Error("Question must not be empty"));
    // New input invalidates a proposal that has not been committed, but leaves physical motion alone.
    const work = this.queue.find(w => w.kind !== "dialogue") ?? this.currentWork;
    if (work && work.kind !== "dialogue") this.interruptedWork = { kind: work.kind, reason: work.reason };
    else if (this.task && ["RUNNING", "PLANNING"].includes(this.task.status) && this.control.state !== "NAVIGATING") {
      this.interruptedWork = { kind: this.task.stages.length ? "stage" : "plan", reason: "unfinished_stage" };
    }
    this.cancel("user question");
    return new Promise((resolve, reject) => this.schedule("dialogue", "user_question", { text, resolve, reject }));
  }
  async humanPause(): Promise<SimStatus> {
    this.cancel("human pause request");
    await this.client.pause("human");
    // /pause returns acceptance; read the authoritative status for the event handler.
    const current = await this.client.status(); this.onControl(current); return current;
  }
  async humanResume(): Promise<SimStatus> {
    if (this.task?.pauseReason === "simulator_restarted") throw new Error("Simulator changed; submit a new task");
    const status = await this.client.resume();
    const current = "generation" in status ? status : await this.client.status();
    this.onControl(current); this.armDeadline(); return current;
  }
  private async stopMotion(reason: string, generation = this.control.generation): Promise<void> {
    const serial = this.serial;
    try {
      const control = await this.client.action(this.id, generation, "pause", undefined, reason);
      if (serial === this.serial) this.control = control;
    } catch (error) { this.record("stop_failed", { reason, error: String(error) }); }
  }
  async stopCurrent(): Promise<void> {
    this.cancel("operator cancelled task"); const serial = this.serial;
    await this.stopMotion("operator_cancelled");
    if (serial === this.serial) this.finish("CANCELLED", "任务已取消。");
  }
  private finish(status: MonitorOutcome["status"], reason: string): void {
    clearTimeout(this.taskTimer);
    if (!this.task) { this.say(reason); return; }
    this.task.status = status; this.task.pauseReason = reason;
    if (["COMPLETED", "CANCELLED"].includes(status) && this.task.memory) {
      this.record("decision_segment_archived", { ...this.task.memory, endReason: status });
    }
    this.save(status === "COMPLETED" ? "task_completed" : "task_stopped", { status, reason, control: this.control });
    this.say(reason);
    this.outcome?.({ status, taskId: this.task.id, reason }); this.outcome = undefined;
  }
  async close(): Promise<void> {
    if (this.closed) return;
    this.closed = true; this.cancel("monitor closing");
    clearInterval(this.heartbeatTimer); clearTimeout(this.taskTimer); this.stream.abort();
    if (this.live()) await this.stopMotion("monitor_closed");
    await this.draining;
    await this.listening;
    this.store.close();
  }
  private archiveImages(prefix: string, images: ObservationImage[]): string[] {
    return images.map(({ image }, index) => {
      const path = join(this.artifactDir, `${prefix}_${index}.${image.mimeType === "image/png" ? "png" : "jpg"}`);
      writeFileSync(path, Buffer.from(image.data, "base64")); return path;
    });
  }
  private async observe(work: Work, signal: AbortSignal): Promise<Observation> {
    let before = await this.client.status(signal);
    if (work.kind !== "dialogue" && ["PAUSED", "ARRIVAL_CANDIDATE", "LOCAL_DONE", "IDLE"].includes(before.state) && !isStopped(before)) {
      // Wait on physical telemetry, not repeated model/status tool calls.
      before = await new Promise<SimStatus>((resolve, reject) => {
        const done = (error?: unknown, status?: SimStatus) => {
          clearTimeout(timer); this.off("control", listener); signal.removeEventListener("abort", aborted);
          if (error) reject(error); else resolve(status!);
        };
        const listener = (status: SimStatus) => { if (isStopped(status)) done(undefined, status); };
        const aborted = () => done(signal.reason);
        const timer = setTimeout(() => done(new Error("Robot did not reach a measured low-speed stop in 5 seconds")), 5000);
        this.on("control", listener); signal.addEventListener("abort", aborted, { once: true });
      });
    }
    const frame = await this.client.captureScene(signal);
    const control = await this.client.status(signal);
    this.check(work, signal);
    if (before.generation !== control.generation) throw new Error("Controller changed while taking observation");
    this.control = control;
    const motion = await this.client.motion(signal);
    this.check(work, signal);
    if (this.task?.memory && motion?.control_task_id === this.task.memory.decision.controlTaskId) {
      this.task.memory.motion = motion;
      this.save("decision_buffer_updated", { bufferId: this.task.memory.id, motion });
    }
    const id = `${this.task?.id ?? "dialogue"}_${this.id}_${++this.observationNumber}`;
    const imagePaths = this.archiveImages(id, frame.images);
    const observation = { id, frame, control, imagePaths, regions: [], reviewed: false };
    const { images, ...metadata } = frame;
    this.record("observation", { id, ...metadata, control, imagePaths });
    if (work.kind !== "dialogue") {
      this.observation = observation;
    }
    return observation;
  }
  private conditions(stage: Stage, observation: Observation): string[] {
    const unmet: string[] = [];
    if (stage.kind === "pass_doorway") {
      if (!stage.portal) unmet.push("Need two reviewed, spatially separated boundary measurements and confirmation of the passage between them");
      else if (portalClearance(stage.portal, observation.control.pose_world) < navigationContract.doorway_exit_margin_m) {
        unmet.push("The whole A2 footprint has not cleared the measured doorway entrance plane plus the explicit exit margin");
      }
    } else {
      if (!stage.attempts.some(a => a.controlTaskId === observation.control.task_id && a.result)) {
        unmet.push("Need an executed and ended local navigation for THIS approach stage; a previous stage's LOCAL_DONE does not satisfy it");
      }
      const target = observation.regions.find(r => r.role === "target" && r.selectedLayer);
      if (!observation.reviewed || !target) unmet.push("Need successfully reviewed CURRENT target surface ROI; grid is not ROI evidence");
      if (target && stage.distanceM !== undefined && Math.abs(target.selectedLayer!.median_m - stage.distanceM) > 0.15) {
        unmet.push(`Explicit user distance ${stage.distanceM} m is unsatisfied (target ROI median ${target.selectedLayer!.median_m} m; tolerance ±0.15 m)`);
      }
      if (!stage.anchor?.surface) unmet.push("Need an observed, semantically associated vertical target surface");
      else if (stage.distanceM === undefined) unmet.push(...waitingZoneConditions(stage.anchor, observation.control.pose_world));
    }
    if (!isStopped(observation.control)) unmet.push("Need zero motion commands and measured planar speed below 0.05 m/s");
    return unmet;
  }
  private context(work: Work, observation: Observation, extra: unknown, available: Tool[], purpose: string): unknown {
    const stage = this.stage();
    if (stage) stage.unmet = this.conditions(stage, observation);
    const status = observation.control;
    const scope = this.task && `${this.task.id}:${this.task.revision}:${this.task.currentStage}`;
    const hint = observation.frame.lightnav_hint;
    const isReview = purpose === "review_measurements" || purpose === "measurement_failure";
    const doorwayReady = stage?.kind === "pass_doorway" && Boolean(stage.portal);
    const targetReady = stage?.kind === "approach_target" && Boolean(stage.anchor);
    const currentStep = {
      availableTools: available.map(tool => tool.name),
      objective: work.kind === "dialogue" ? "根据当前图像、任务状态和控制事实回答用户问题。"
        : isReview ? "核对当前裁图和深度层的像素身份；看不清时继续观察。"
        : stage?.kind === "pass_doorway"
        ? doorwayReady ? "沿已确认门洞连续走到退出点；Monitor检查整个机身出门后自动推进。"
          : stage.boundaries?.length ? "当前门洞尚未确认。保留已有边界证据，寻找另一侧；直接转向观察不需要先确认另一侧。"
            : "寻找当前任务所指的通行开口。目标不可见时直接转向观察；看到候选边界后再测量。"
        : targetReady ? "沿已确认目标连续接近；到达后确认当前目标表面，Monitor用最新实测站位自动完成。"
          : "寻找并确认指定目标。不可见时直接转向观察，看到候选后再测量。",
      completionAuthority: "Monitor自动核验并推进阶段。下面的完成条件不是搜索/观察的前提。",
    };
    let passageTarget: unknown;
    if (stage?.portal) {
      const portal = stage.portal;
      const exitPoint = portal.origin.map((value, i) => value+portal.normal[i]*(bodyRadiusM+navigationContract.doorway_exit_margin_m+2*navigationContract.local_goal.position_tolerance_m));
      const dx = exitPoint[0]-status.pose_world[0], dy = exitPoint[1]-status.pose_world[1];
      const bearing = Math.atan2(dy, dx)-status.pose_world[2];
      passageTarget = { exitPointWorldM: exitPoint, distanceM: Math.hypot(dx, dy),
        bearingDegPositiveLeft: Math.atan2(Math.sin(bearing), Math.cos(bearing))*180/Math.PI,
        remainingNormalClearanceM: navigationContract.doorway_exit_margin_m-portalClearance(portal, status.pose_world) };
    }
    const depth = observation.frame.depth as { cells: { cell_id: number; valid_ratio: number; layers: { median_m: number }[] }[] };
    const compactLayer = (layer: Layer) => ({ layerId: layer.layer_id, medianM: layer.median_m,
      p10M: layer.p10_m, p90M: layer.p90_m, sampleWorldPointM: layer.sample_world_point_m,
        supportWorldBoundsM: layer.support_world_bounds_m,
        surface: layer.surface && { normalWorld: layer.surface.normal_world, rmsM: layer.surface.rms_m,
        geometry: usableVerticalSurface(layer) ? "resolved_vertical_surface"
          : Math.abs(layer.surface.normal_world[2]) > 0.9 && layer.surface.rms_m < 0.04 ? "horizontal_surface" : "mixed_or_unresolved_surface" } });
    return {
      originalGoal: this.task?.originalGoal, taskStatus: this.task?.status, currentStep,
      plan: this.task?.stages.map(({ clause, status }) => ({ clause, status })),
      currentStage: this.task?.currentStage,
      stage: stage && { kind: stage.kind, clause: stage.clause, targetId: stage.targetId,
        target: stage.target, instruction: stage.instruction, explicitUserDistanceM: stage.distanceM,
        completion: { checkedBy: "monitor", unmetConditions: stage.unmet },
        targetAnchor: stage.anchor, passageTarget,
        boundaryEvidence: stage.boundaries?.map(({ id, region: r, measurementCount, currentFrameMaskIds }) => ({
          id, measurementCount, currentFrameMaskIds, observationId: r.observationId, poseWorld: r.poseWorld,
          description: r.description, semanticClaim: r.semanticClaim, worldPointM: r.selectedLayer!.sample_world_point_m,
          supportWorldBoundsM: r.selectedLayer!.support_world_bounds_m })),
        boundaryEvidenceMeaning: stage.kind === "pass_doorway"
          ? "B编号是当前通行开口的空间边界证据组，可包含同一位置的多个mask/视角；不是已证明的永久物体ID。过近的测量不能构成可用开口两侧。几何分离后仍需RGB确认其任务语义关系。" : undefined,
        targetGeometry: stage.anchor?.surface && targetGeometry(stage.anchor, status.pose_world, stage.distanceM),
        completionConvention: stage.kind === "approach_target" && stage.distanceM === undefined
          ? { source: navigationContract.source, ...navigationContract.waiting_zone } : undefined,
        portal: stage.portal && { observationId: stage.portal.observationId, origin: stage.portal.origin,
          normal: stage.portal.normal, bodyClearanceM: portalClearance(stage.portal, status.pose_world),
          requiredExitClearanceM: navigationContract.doorway_exit_margin_m,
          bearingDegPositiveLeft: Math.atan2(
            Math.sin(Math.atan2(stage.portal.origin[1]-status.pose_world[1], stage.portal.origin[0]-status.pose_world[0])-status.pose_world[2]),
            Math.cos(Math.atan2(stage.portal.origin[1]-status.pose_world[1], stage.portal.origin[0]-status.pose_world[0])-status.pose_world[2]))*180/Math.PI },
      },
      reason: work.reason, userQuestion: work.text,
      control: { state: status.state, reason: status.reason, poseWorld: status.pose_world,
        actualVelocityBody: status.actual_velocity_body, effectiveCommand: status.effective_cmd,
        forceBrakeLatched: status.force_brake_latched, lightnav: status.lightnav_latest },
      controlEventFact: controlEventFact(status),
      currentObservation: { id: observation.id, simTimeS: observation.frame.sim_time_s,
        poseWorld: observation.frame.pose_world, camera: observation.frame.camera,
        lightnavTargetHint: hint?.navigation_scope === scope ? hint : null,
        semanticDepth: observation.frame.semantic_depth,
        depthOverview: { grid: "6 columns, 3 rows, top-left row-major; visible surface horizontal range from robot base in metres",
          cells: depth.cells.map(cell => ({ cell: cell.cell_id, valid: cell.valid_ratio,
            rangesM: cell.layers.map(layer => layer.median_m) })) } },
      measurements: observation.regions.map((region, index) => ({ index, role: region.role, bboxPx: region.bbox_px,
        description: region.description, error: region.error,
        semanticInstance: region.semanticInstanceId ? { id: region.semanticInstanceId,
          label: region.result?.label, confidence: region.result?.confidence } : undefined,
        layers: region.selectedLayer && observation.reviewed ? [compactLayer(region.selectedLayer)]
          : (region.result?.layers as Layer[] | undefined)?.map(compactLayer),
        semanticClaim: region.semanticClaim })),
      measurementsReviewed: observation.reviewed,
      boundaryMeasurementRelations: stage?.kind === "pass_doorway" ? this.boundaryMeasurementRelations(stage, observation) : undefined,
      buffer: memoryContext(this.task?.memory),
      capabilities: { observationSkill: "相机固定，search_target/change_view通过机身转向取得新RGB-D。每次实测转向后重新规划。", reverse: false,
        followTarget: available.some(tool => tool.name === "follow_target")
          ? "Executes the observed local goal continuously through alignment, approach and final heading. Internal phases do not require new decisions." : undefined,
        turnCheckpoint: "turnDegrees sets a control-tick measured yaw checkpoint (default 45°); MPC commands zero forward speed while turning. Actual compliant motion can drift.",
        boundedAdvance: available.some(tool => tool.name === "navigate")
          ? "advanceMeters executes a measured straight segment (0.1–2m) with an explicit MPC reference. LightNav keeps observing the semantic goal but does not override this primitive." : undefined,
        observations: "Current RGB-D and actual simulator odometry; depth bands are not object segmentation.",
        distance: "ROI median is base-to-visible-point range. Waiting clearance is footprint-to-observed-plane distance; neither is route remaining distance." },
      rules: work.kind === "dialogue" ? [
        "回答当前问题，并区分实测事实、模型判断与未知信息。任务状态和控制事件以当前记录为准。",
      ] : isReview ? [
        "依据当前任务和实际RGB判断所选像素的身份；几何只说明这些测量支持什么空间关系。",
        "当前观测不足时可以重新选区或转向观察。先前识别确实错误时明确更换保留证据。",
      ] : stage?.kind === "pass_doorway" ? doorwayReady ? [
        "通行开口及退出点已确认。用follow_target连续完成转向和穿越，直到整个机身清空开口。",
        "Monitor用已测边界与最新实际位姿核验完整穿越。已通过的边界不必仍出现在当前图中。",
      ] : [
        "当前还在寻找/确认任务所指开口。可以直接search_target观察，不要求先有测量，也不要求同时看见两侧。",
        "实体身份和关联依据用户需求及实际RGB判断。mask_relations和boundaryMeasurementRelations只是测量关系；多个ID不能直接作为多个实体或两侧的证明。",
        "空间分开的两份边界证据与同一开口的视觉确认共同建立穿越目标。若先前识别错误，用replacePreviousIdentification=true明确纠正。",
      ] : [
        "当前画面不能辨认指定目标时，直接转向寻找。看见候选才选择测量；只有身份确认后才接受测量锚定目标。",
        "保持同一个目标。发现以前认错时用replacePreviousIdentification=true明确纠正；同一目标的新视角不需要填写内部association状态。",
        "lightnavTargetHint是另一导航模型在同帧深度上测得、重投影到当前图的目标指向假设。可用它定位待核验小区域，但仍需当前RGB确认，不能把提示本身当实体真值。",
        ...(targetReady ? ["follow_target连续前往standingPointWorldM并最终面向目标，不需要自行拆成步长和转角。到达后确认当前ROI身份，Monitor会自动用最新实测站位核验完成。"] : []),
      ], extra,
    };
  }
  private boundaryMeasurementRelations(stage: Stage, observation: Observation): unknown[] {
    const options = observation.regions.flatMap((region, index) =>
      ((region.result?.layers as Layer[] | undefined) ?? []).filter(usableVerticalSurface)
        .map(layer => ({ reference: `measurement[${index}].${layer.layer_id}`, regionIndex: index, layer })));
    const relations: unknown[] = [];
    for (const option of options) {
      for (const jamb of stage.boundaries ?? []) relations.push({ a: option.reference, b: jamb.id,
        ...boundaryRelation(option.layer, jamb.region.selectedLayer!) });
      for (const other of options.filter(item => item.regionIndex > option.regionIndex)) {
        relations.push({ a: option.reference, b: other.reference, ...boundaryRelation(option.layer, other.layer) });
      }
    }
    return relations;
  }
  private async propose(work: Work, observation: Observation, purpose: string, available: Tool[], signal: AbortSignal,
    extra: unknown = {}, regionImages: ObservationImage[] = []): Promise<DecisionResult> {
    this.check(work, signal);
    const planning = purpose === "plan_task";
    const context = planning ? { originalGoal: this.task!.originalGoal, extra, rules: [
      "只理解原始需求，不根据当前看见什么来删掉后续目标。每个动作一个阶段。",
      "先后依赖分别列出，例如先穿过门、再接近远处目标，要两个阶段。",
      "clause 必须是用户原文的连续片段。按顺序拼接所有clause，要覆盖完整原文。",
      "pass_doorway表示整个机器人通过门，approach_target表示接近终点。不要互换。",
      "instruction必须是简短英文导航指令。用户未写数值距离时彻底省略distanceM，不得猜测。",
    ] } : this.context(work, observation, extra, available, purpose);
    const images = planning ? [] : observation.frame.images.filter(image => image.kind === "rgb" || image.kind === "semantic_rgb");
    const previous = this.task?.memory?.observation;
    if (!planning && previous) images.push({ kind: "previous_decision",
      label: `Previous accepted movement decision, sim ${previous.simTimeS}s; compare with CURRENT RGB. Historical, not current.`,
      image: { type: "image", mimeType: "image/jpeg", data: readFileSync(previous.imagePath).toString("base64") } });
    images.push(...regionImages.filter(i => i.kind !== "selection"));
    this.record("decision_context", { purpose, context, imagePaths: planning ? [] : observation.imagePaths });
    const timeout = AbortSignal.timeout(this.options.decisionTimeoutS * 1000);
    const combined = AbortSignal.any([signal, timeout]);
    // Racing the deadline also handles a driver that ignores AbortSignal. Its late result has no execution path.
    const result = await Promise.race([
      this.driver.decide({ purpose, context, images, tools: available, signal: combined }),
      new Promise<never>((_, reject) => combined.addEventListener("abort", () => reject(combined.reason), { once: true })),
    ]);
    this.check(work, signal);
    this.record("proposal", { purpose, observationId: observation.id, controlGeneration: observation.control.generation, result });
    return result;
  }
  private async measure(work: Work, observation: Observation, regions: Region[], signal: AbortSignal): Promise<ObservationImage[]> {
    observation.reviewed = false;
    observation.regions = regions.map(({ role, bbox_px, description, semanticInstanceId }) =>
      ({ role, bbox_px, description, semanticInstanceId, observationId: observation.id, poseWorld: observation.frame.pose_world }));
    const images: ObservationImage[] = [];
    for (let i = 0; i < observation.regions.length; i++) {
      const region = observation.regions[i];
      this.check(work, signal);
      this.record("measurement_requested", { observationId: observation.id, frameId: observation.frame.frame_id, regionIndex: i, region });
      try {
        const { images: resultImages, ...result } = region.semanticInstanceId
          ? await this.client.measureSemanticInstance(observation.frame.frame_id, region.semanticInstanceId, signal)
          : await this.client.measureRegion(observation.frame.frame_id, region.bbox_px, signal);
        this.check(work, signal);
        if (String(result.frame_id) !== String(observation.frame.frame_id)) throw new Error("Measurement returned a different observation frame");
        if (result.status !== "measured") throw new Error("Selected ROI has no valid depth: unknown, not free space");
        region.result = result;
        const paths = this.archiveImages(`${observation.id}_roi${i}_${Date.now()}`, resultImages);
        region.cropImagePath = paths[resultImages.findIndex(img => img.kind === "crop")];
        images.push(...resultImages.map(img => ({ ...img, label: `Region ${i} (${region.role}): ${img.label}` })));
        this.record("measurement_result", { observationId: observation.id, regionIndex: i, success: true, result, imagePaths: paths });
      } catch (error) {
        this.check(work, signal);
        region.error = String(error);
        this.record("measurement_result", { observationId: observation.id, regionIndex: i, success: false, error: region.error });
      }
    }
    return images;
  }
  private async reviewRegions(work: Work, observation: Observation, first: Region[], signal: AbortSignal): Promise<boolean> {
    let regions = first;
    let geometryErrors: string[] = [];
    let images: ObservationImage[] = [];
    let needsMeasurement = true;
    for (let trial = 0; trial < 3; trial++) {
      if (needsMeasurement) {
        const stage = this.stage()!;
        const roles = regions.map(r => r.role);
        const validRoles = stage.kind === "pass_doorway"
          ? roles.length >= 1 && roles.length <= 2 && roles.every(role => role === "boundary")
          : roles.length === 1 && roles[0] === "target";
        if (!validRoles) throw new Error("Selected region roles do not belong to this stage");
        images = await this.measure(work, observation, regions, signal);
        const retained = (stage.boundaries ?? []).filter(jamb => jamb.region.observationId !== observation.id);
        for (const { id, region } of retained) {
          if (!region.cropImagePath) throw new Error("Retained boundary is missing its source RGB crop");
          images.push({ kind: "retained_boundary", label: `Boundary evidence ${id}, observation ${region.observationId}, claim: ${region.semanticClaim}. Compare world support and RGB to decide whether CURRENT measurements add another boundary of this opening.`,
            image: { type: "image", mimeType: "image/jpeg", data: readFileSync(region.cropImagePath).toString("base64") } });
        }
      }
      needsMeasurement = false;
      const success = observation.regions.every(r => r.result && !r.error);
      const stageKind = this.stage()!.kind;
      const selections = [tools.selectFor(stageKind)];
      if (observation.frame.semantic_depth.instances.length) selections.push(tools.selectSemanticFor(stageKind,
        observation.frame.semantic_depth.instances.map(instance => instance.id)));
      const observationTools = [tools.searchTarget, tools.changeView, tools.help];
      const response = await this.propose(work, observation, success ? "review_measurements" : "measurement_failure",
        success ? [tools.reviewFor(stageKind, observation.regions.length), ...selections, ...observationTools] : [...selections, ...observationTools], signal,
        { requirement: stageKind === "pass_doorway"
          ? "依据当前任务核对所选边界区域及层成员；参考掩码重叠和世界支持关系。同位置的多份测量可保留为一份边界证据，缺另一侧时继续观察。"
          : "核对指定目标的一块可见表面即可，不要求整扇门。RGB身份与垂直表面层成立就接受；否则重新选区或转向观察。",
          trial, geometryErrors }, images);
      if (response.name === "request_help") { await this.help(response.args, work, observation); return false; }
      if (response.name === "change_view" || response.name === "search_target") {
        const args = response.args as { direction: "left" | "right"; degrees: number; reason: string; soughtEvidence?: string };
        this.record("observation_skill_proposed", { skill: response.name, observationId: observation.id, ...args });
        await this.commitMovement({ maneuver: `turn_${args.direction}`, instruction: `Turn ${args.direction}.`,
          turnDegrees: args.degrees, expectedEffect: args.reason }, work, observation, signal);
        return false;
      }
      if (response.name === "select_regions") {
        regions = tools.originalPixels((response.args as { regions: Parameters<typeof tools.originalPixels>[0] }).regions,
          observation.frame.camera.width, observation.frame.camera.height, stageKind);
        needsMeasurement = true; geometryErrors = []; continue;
      }
      if (response.name === "select_semantic_instances") {
        regions = this.semanticRegions(observation, response.args);
        needsMeasurement = true; geometryErrors = []; continue;
      }
      if (response.name !== "accept_measurements" || !success) throw new Error("Measurement review phase violation");
      const accepted = response.args as { selections: { layerId: string; semanticClaim: string;
        visualIdentity: "confirmed_target" | "uncertain" | "not_target" }[];
        sameOpening?: boolean; replacePreviousIdentification?: boolean };
      const reviewedSelections = accepted.selections.map((selection, regionIndex) => ({ ...selection, regionIndex }));
      const { sameOpening, replacePreviousIdentification } = accepted;
      if (replacePreviousIdentification) {
        const stage = this.stage()!;
        delete stage.anchor; delete stage.portal; delete stage.boundaries;
        this.save("target_identity_corrected", { observationId: observation.id, selections: reviewedSelections });
      }
      if (reviewedSelections.some(selection => selection.visualIdentity !== "confirmed_target")) {
        geometryErrors = ["Identity is unconfirmed or rejected. Select the actual target or change_view; uncertain/non-target pixels cannot anchor motion or completion."];
        this.record("measurement_identity_rejected", { observationId: observation.id, selections: reviewedSelections });
        continue;
      }
      if (reviewedSelections.length !== observation.regions.length) throw new Error("Must review each measured region exactly once");
      for (const selection of reviewedSelections) {
        const region = observation.regions[selection.regionIndex];
        const layer = (region?.result?.layers as Layer[])?.find(layer => layer.layer_id === selection.layerId);
        if (!layer) throw new Error("Semantic review references an unmeasured layer");
        region.selectedLayer = layer; region.semanticClaim = selection.semanticClaim;
      }
      const stage = this.stage()!;
      geometryErrors = [];
      for (const region of observation.regions) {
        if (!usableVerticalSurface(region.selectedLayer!)) geometryErrors.push(`${region.role}: selected pixels do not support a resolved vertical plane; reselect the actual upright target surface.`);
      }
      if (stage.kind === "pass_doorway" && !geometryErrors.length) {
        const combined = mergeBoundaryEvidence(stage.boundaries ?? [], observation.regions);
        if (combined.length > 2) {
          geometryErrors.push("New measurements introduce a third spatial boundary to the current opening hypothesis. Reinspect which opening is intended; use replacePreviousIdentification=true only when the retained identification was wrong.");
        } else {
          stage.boundaries = combined;
          delete stage.portal;
          this.save("boundary_evidence_merged", { observationId: observation.id, boundaries: combined,
            meaning: "Spatial boundary evidence groups, not mask-count-based object identities." });
          if (combined.length === 2) {
            if (sameOpening === true) stage.portal = makePortal(observation, combined);
            else geometryErrors.push("Two spatially separated boundary groups are available. Confirm the visible opening between them with sameOpening=true; if the retained opening was wrong, explicitly replace that identification. Width alone does not identify an opening.");
          }
        }
      }
      if (stage.kind === "approach_target" && !geometryErrors.length) {
        const region = observation.regions[0], layer = region.selectedLayer!;
        const next: TargetAnchor = { id: stage.targetId, description: stage.target, relation: "wait_outside",
          observationId: observation.id, bboxPx: region.bbox_px, pointWorldM: layer.sample_world_point_m,
          surface: layer.surface!, semanticClaim: region.semanticClaim!,
          referencePointWorldM: stage.anchor
            ? stage.anchor.referencePointWorldM : layer.sample_world_point_m };
        if (stage.anchor?.surface) {
          const old = stage.anchor.surface, current = next.surface!;
          const agreement = Math.abs(old.normal_world.reduce((sum, n, i) => sum+n*current.normal_world[i], 0));
          const offset = Math.abs(old.normal_world.reduce((sum, n, i) => sum+n*(current.centroid_world_m[i]-old.centroid_world_m[i]), 0));
          if (agreement < Math.cos(Math.PI/6) || offset > 0.25) {
            geometryErrors.push("This surface conflicts with the retained target plane. Reinspect it; set replacePreviousIdentification=true only if the earlier identification was wrong.");
          }
        }
        if (!geometryErrors.length) {
          this.record("target_associated", { previous: stage.anchor, current: next,
            association: replacePreviousIdentification ? "correct_previous_identification" : stage.anchor ? "same_target" : "first_observation" });
          stage.anchor = next;
        }
      }
      if (geometryErrors.length) {
        this.record("measurement_geometry_rejected", { observationId: observation.id, geometryErrors, regions: observation.regions });
        continue;
      }
      observation.reviewed = true;
      stage.evidenceIds.push(observation.id);
      this.task!.facts.push({ type: "measured_regions", observationId: observation.id, regions: observation.regions.map(r => ({ role: r.role, layer: r.selectedLayer })) });
      this.task!.claims.push({ type: "pixel_semantics", observationId: observation.id, claims: observation.regions.map(r => r.semanticClaim) });
      this.task!.facts = this.task!.facts.slice(-12); this.task!.claims = this.task!.claims.slice(-12);
      this.save("evidence_reviewed", { observationId: observation.id, regions: observation.regions, portal: stage.portal, target: stage.anchor });
      return true;
    }
    await this.help({ reason: "同一观察的目标选区三次未取得可核验证据。", needed: "帮助确认目标位置或提供更清晰视角。" }, work, observation);
    return false;
  }
  private semanticRegions(observation: Observation, args: unknown): Region[] {
    const { regions } = args as { regions: { instanceId: string; description: string }[] };
    return regions.map(selection => {
      const instance = observation.frame.semantic_depth.instances.find(candidate => candidate.id === selection.instanceId);
      if (!instance) throw new Error("Semantic selection does not reference a current-frame instance");
      return { role: this.stage()!.kind === "pass_doorway" ? "boundary" : "target", description: selection.description,
        bbox_px: instance.bbox_px, semanticInstanceId: instance.id };
    });
  }
  private recall(args: unknown): { context: unknown; images: ObservationImage[] } {
    const { stageIndex, reason, failedAttemptsOnly } = args as { stageIndex: number; reason: string; failedAttemptsOnly?: boolean };
    if (!this.task || stageIndex >= this.task.stages.length) throw new Error("Recall stage is outside the current task");
    const segments = this.store.recallSegments(this.task.id, stageIndex, failedAttemptsOnly === true) as DecisionMemory[];
    const images: ObservationImage[] = segments.map(segment => ({ kind: "recalled_memory",
      label: `Archived decision ${segment.id}, stage ${segment.stageIndex}, sim ${segment.observation.simTimeS}s. Historical evidence requested for ${reason}.`,
      image: { type: "image", mimeType: "image/jpeg", data: readFileSync(segment.observation.imagePath).toString("base64") } }));
    this.record("memory_recalled", { reason, stageIndex, segmentIds: segments.map(segment => segment.id) });
    return { context: { reason, segments: segments.map(memoryContext) }, images };
  }
  private endAttempt(control: SimStatus): void {
    const attempt = this.activeAttempt;
    if (!attempt || attempt.result) return;
    Object.assign(attempt, { endPose: control.pose_world, result: control.reason || control.state,
      ...motionDelta(attempt.startPose, control.pose_world) });
    this.save("action_result", { attempt, control });
    this.activeAttempt = undefined;
  }
  private async help(args: unknown, work: Work, observation: Observation): Promise<void> {
    const { reason, needed } = args as { reason: string; needed: string };
    if (work.serial !== this.serial) return;
    await this.stopMotion("needs_user", observation.control.generation);
    if (work.serial === this.serial) this.finish("NEEDS_USER", `${reason} 需要帮助：${needed}`);
  }
  private async commitMovement(args: { maneuver: string; instruction: string; expectedEffect: string; turnDegrees?: number; advanceMeters?: number;
    goalPointWorld?: number[]; goalHeadingRad?: number },
    work: Work, observation: Observation, signal: AbortSignal): Promise<void> {
    const stage = this.stage()!;
    if (!this.task) throw new Error("Movement requires an active task");
    const instruction = args.maneuver === "follow_target" ? "Follow the observed navigation goal."
      : args.maneuver === "advance"
      ? (args.advanceMeters !== undefined ? "Move straight forward."
        : stage.kind === "pass_doorway" ? "Move straight forward to pass through the doorway." : args.instruction)
      : args.maneuver === "turn_left" ? "Turn left." : "Turn right.";
    const movement = { maneuver: args.maneuver, advanceMeters: args.advanceMeters, semanticInstruction: stage.instruction,
      goalPointWorld: args.goalPointWorld, goalHeadingRad: args.goalHeadingRad,
      positionToleranceM: navigationContract.local_goal.position_tolerance_m,
      headingToleranceDeg: navigationContract.local_goal.heading_tolerance_deg,
      turnDegrees: args.maneuver.startsWith("turn_") ? args.turnDegrees ?? 45 : undefined };
    const control = await this.client.action(this.id, observation.control.generation, "navigate", instruction, undefined, signal,
      `${this.task.id}:${this.task.revision}:${this.task.currentStage}`, movement);
    this.check(work, signal);
    this.control = control;
    if (this.task.memory) {
      const previousMotion = control.previous_motion as MotionSegment | null;
      if (previousMotion?.control_task_id === this.task.memory.decision.controlTaskId) this.task.memory.motion = previousMotion;
      this.record("decision_segment_archived", { ...this.task.memory, endReason: "next_decision_accepted" });
    }
    this.task.memory = { id: randomUUID(), stageIndex: this.task.currentStage, targetId: stage.targetId,
      observation: { id: observation.id, imagePath: observation.imagePaths[0], simTimeS: observation.frame.sim_time_s,
        poseWorld: observation.frame.pose_world },
      decision: { maneuver: args.maneuver, instruction, expectedEffect: args.expectedEffect,
        turnDegrees: movement.turnDegrees, advanceMeters: movement.advanceMeters,
        semanticInstruction: stage.instruction, referenceSource: control.reference_source as string,
        goalPointWorld: args.goalPointWorld, goalHeadingRad: args.goalHeadingRad,
        controlTaskId: control.task_id!, acceptedSimTimeS: control.sim_time_s as number }, motion: null };
    this.activeAttempt = { ...args, ...movement, instruction, controlTaskId: control.task_id, startPose: control.pose_world };
    stage.attempts.push(this.activeAttempt); stage.attempts = stage.attempts.slice(-12);
    this.save("action_executed", { stageIndex: this.task.currentStage, attempt: this.activeAttempt,
      buffer: this.task.memory, proposedInstruction: args.instruction, control });
    this.say(`第 ${this.task.currentStage + 1} 阶段执行：${instruction}`);
  }
  private targetMovement(stage: Stage, pose: number[], reason: string): {
    maneuver: string; instruction: string; expectedEffect: string; goalPointWorld: number[]; goalHeadingRad: number;
  } | null {
    let point: number[], heading: number;
    if (stage.kind === "pass_doorway") {
      if (!stage.portal) throw new Error("A measured doorway is required for follow_target");
      if (portalClearance(stage.portal, pose) >= navigationContract.doorway_exit_margin_m) return null;
      const offset = bodyRadiusM+navigationContract.doorway_exit_margin_m+2*navigationContract.local_goal.position_tolerance_m;
      point = stage.portal.origin.map((value, i) => value+stage.portal!.normal[i]*offset);
      heading = Math.atan2(stage.portal.normal[1], stage.portal.normal[0]);
    } else {
      if (!stage.anchor?.surface) throw new Error("A measured target is required for follow_target");
      if (stage.distanceM === undefined && !waitingZoneConditions(stage.anchor, pose).length) return null;
      point = targetGeometry(stage.anchor, pose, stage.distanceM).standingPointWorldM;
      heading = Math.atan2(-stage.anchor.surface.normal_world[1], -stage.anchor.surface.normal_world[0]);
    }
    return { maneuver: "follow_target", instruction: "Follow the observed navigation goal.",
      expectedEffect: reason, goalPointWorld: point, goalHeadingRad: heading };
  }
  private async completeIfReady(work: Work, observation: Observation, signal: AbortSignal): Promise<boolean> {
    const stage = this.stage()!;
    if (this.conditions(stage, observation).length) return false;
    const current = await this.client.status(signal);
    this.check(work, signal);
    if (current.generation !== observation.control.generation) throw new Error("Control changed before completion");
    observation.control = current;
    const moved = motionDelta(observation.frame.pose_world, current.pose_world);
    const unmet = this.conditions(stage, observation);
    // Numeric ROI range is frame-bound; ordinary waiting geometry is measured in world coordinates.
    if (stage.distanceM !== undefined && moved.distanceM > 0.1) unmet.push("Re-measure the explicit ROI range after translation");
    if (unmet.length) {
      this.record("stage_completion_deferred", { stageIndex: this.task!.currentStage, unmet,
        observationId: observation.id, motionSinceObservation: moved, control: current });
      this.schedule("verify", "completion_conditions_changed");
      return true;
    }
    const reason = stage.kind === "pass_doorway"
      ? "Monitor已核验整个机身越过已确认门洞并停稳。"
      : "Monitor已核验当前目标身份、最新实测停点条件和停止状态。";
    this.record("stage_completion_checked", { stageIndex: this.task!.currentStage,
      authority: "monitor", observationId: observation.id, motionSinceObservation: moved, control: current });
    this.control = await this.client.action(this.id, current.generation, "complete", undefined, reason, signal);
    this.check(work, signal);
    stage.status = "COMPLETED"; stage.unmet = []; stage.completionReason = reason;
    const completedIndex = this.task!.currentStage;
    this.task!.currentStage++; this.task!.revision++;
    if (this.task!.currentStage === this.task!.stages.length) this.task!.status = "COMPLETED";
    this.save("stage_completed", { stageIndex: completedIndex, stage, control: this.control,
      completionAuthority: "monitor", reason,
      evidence: { observationId: observation.id, portal: stage.portal, target: stage.anchor,
        navigationContract, regions: observation.regions } });
    if (this.task!.currentStage === this.task!.stages.length) {
      this.finish("COMPLETED", "各阶段已由Monitor依据语义确认与最新实测状态完成。");
    } else {
      this.save("stage_advanced", { currentStage: this.task!.currentStage });
      this.observation = undefined; this.schedule("stage", "previous_stage_completed");
    }
    return true;
  }
  private async handle(work: Work, signal: AbortSignal): Promise<void> {
    if (work.kind === "stage" && work.reason === "previous_stage_completed") {
      this.control = await this.client.action(this.id, this.control.generation, "invalidate", undefined, "stage_begin", signal);
      this.check(work, signal);
    }
    const observation = await this.observe(work, signal);
    if (work.kind === "dialogue") {
      let result = await this.propose(work, observation, "user_dialogue", [tools.answer, tools.recall], signal);
      if (result.name === "recall_memory") {
        const recalled = this.recall(result.args);
        result = await this.propose(work, observation, "user_dialogue", [tools.answer], signal,
          { recalledEvidence: recalled.context }, recalled.images);
      }
      const text = (result.args as { text: string }).text;
      this.say(text); work.resolve?.(text);
      if (this.interruptedWork && this.live() && ["RUNNING", "PLANNING"].includes(this.task!.status)
        && !["HUMAN_PAUSED", "NAVIGATING", "MANUAL"].includes(this.control.state)) {
        this.schedule(this.interruptedWork.kind, `after_dialogue:${this.interruptedWork.reason}`);
      }
      this.interruptedWork = undefined; return;
    }
    if (!this.task || observation.control.state === "HUMAN_PAUSED") return;
    if (work.kind === "plan") {
      let stages: Stage[] = [];
      let errors: string[] = [];
      for (let trial = 0; trial < 2; trial++) {
        const result = await this.propose(work, observation, "plan_task", [tools.plan, tools.help], signal, { rejectedPlanReasons: errors });
        if (result.name === "request_help") { await this.help(result.args, work, observation); return; }
        stages = (result.args as { stages: Stage[] }).stages;
        const normalized = (s: string) => s.replace(/[\s，。！？、,.!?；;：:]/g, "");
        errors = [];
        if (normalized(stages.map(s => s.clause).join("")) !== normalized(this.task.originalGoal)) errors.push("各clause必须逐字拼成完整用户原文，不能省略后续目标或加入观察说明。");
        for (const stage of stages) {
          if (!/^[\x20-\x7e]+$/.test(stage.instruction)) errors.push("instruction必须使用英文。");
          if (stage.distanceM !== undefined && !new RegExp(`(?:^|[^0-9.])${String(stage.distanceM).replace('.', '\\.')}\\s*(?:米|m(?:eters?|etres?)?)(?:$|[^a-z])`, "i").test(this.task.originalGoal)) {
            errors.push("distanceM必须来自用户明确写出的同一数值米距离；本请求没有该数值就省略，不能估计或默认。");
          }
        }
        if (!errors.length) break;
        this.record("plan_rejected", { stages, errors });
      }
      if (errors.length) { await this.help({ reason: errors.join(" "), needed: "明确未能保留的需求，重新规划。" }, work, observation); return; }
      this.task.stages = stages.map((stage, index) => ({ ...stage, targetId: `${this.task!.id}:target:${index}`,
        status: "PENDING", attempts: [], unmet: [], evidenceIds: [] }));
      this.task.currentStage = 0; this.task.status = "RUNNING";
      this.save("task_planned", { stages: this.task.stages, originalGoal: this.task.originalGoal,
        navigationContract, control: observation.control });
      this.say(`任务已分为 ${stages.length} 个阶段：${stages.map(s => s.clause).join(" → ")}`);
      this.schedule("stage", "plan_ready"); return;
    }
    this.task.status = "RUNNING";
    const stage = this.stage()!;
    stage.status = "ACTIVE";
    let feedback: unknown = {};
    let recalled: { context: unknown; images: ObservationImage[] } | undefined;
    if (work.kind === "recover") {
      const failed = stage.attempts.filter(a => a.result && ["mpc_blocked", "no_progress"].includes(a.result));
      this.save("recovery_started", { stageIndex: this.task.currentStage, reason: work.reason, attempts: stage.attempts, control: observation.control });
      if (failed.length > this.options.maxRecoveries) {
        await this.help({ reason: "本阶段已达到恢复次数上限，控制器没有完成预期动作。", needed: "人工调整位置或更改目标，再明确恢复。" }, work, observation); return;
      }
    }
    for (let round = 0; round < 5; round++) {
      if (await this.completeIfReady(work, observation, signal)) return;
      const hasTarget = stage.kind === "pass_doorway" ? Boolean(stage.portal) : Boolean(stage.anchor);
      const available = [tools.help];
      if (hasTarget) available.unshift(tools.followTarget);
      else if (stage.kind === "approach_target") available.unshift(tools.navigate);
      if (!hasTarget) available.unshift(tools.searchTarget);
      if (!observation.reviewed && !(stage.kind === "pass_doorway" && stage.portal)
        && observation.frame.semantic_depth.instances.length) available.unshift(tools.selectSemanticFor(stage.kind,
          observation.frame.semantic_depth.instances.map(instance => instance.id)));
      const hint = observation.frame.lightnav_hint;
      const scope = `${this.task.id}:${this.task.revision}:${this.task.currentStage}`;
      if (stage.kind === "approach_target" && !observation.reviewed && hint?.navigation_scope === scope && hint.in_view && hint.point_1000) {
        available.unshift(tools.inspectHint);
      }
      if (hasTarget && work.kind === "recover") available.push(tools.navigate, tools.changeView);
      if (!recalled && this.task.memory) available.push(tools.recall);
      if (!observation.reviewed && !(stage.kind === "pass_doorway" && stage.portal)) available.unshift(tools.selectFor(stage.kind));
      const result = await this.propose(work, observation,
        work.kind === "recover" ? "error_recovery" : work.kind === "verify" ? "verify_arrival" : "stage_decision",
        available, signal, { feedback, recalledEvidence: recalled?.context }, recalled?.images);
      if (result.name === "recall_memory") { recalled = this.recall(result.args); continue; }
      if (result.name === "search_target") {
        const args = result.args as { direction: "left" | "right"; degrees: number; soughtEvidence: string; reason: string };
        this.record("observation_skill_proposed", { skill: "search_target", observationId: observation.id, ...args });
        await this.commitMovement({ maneuver: `turn_${args.direction}`, instruction: `Turn ${args.direction}.`, turnDegrees: args.degrees,
          expectedEffect: `${args.soughtEvidence}: ${args.reason}` }, work, observation, signal); return;
      }
      if (result.name === "inspect_target_hint") {
        const [u, v] = hint!.point_1000!;
        const { width, height } = observation.frame.camera;
        const x = u*width/1000, y = v*height/1000;
        const region: Region = { role: "target", description: "Candidate patch around the reprojected LightNav semantic target; identity still requires current RGB review.",
          bbox_px: [Math.max(0, Math.floor(x-width*.025)), Math.max(0, Math.floor(y-height*.04)),
            Math.min(width, Math.ceil(x+width*.025)), Math.min(height, Math.ceil(y+height*.04))] };
        this.record("target_hint_inspected", { observationId: observation.id, hint, region });
        if (!await this.reviewRegions(work, observation, [region], signal)) return;
        feedback = { observationMeasurementCompleted: true, next: "候选测量已审核。根据currentStep继续；Monitor自动检查完成条件。" };
        continue;
      }
      if (result.name === "follow_target") {
        const movement = this.targetMovement(stage, observation.control.pose_world, (result.args as { reason: string }).reason);
        if (!movement) { feedback = { next: "站位几何已满足。若当前目标表面尚未审核，选择并确认这一小块；Monitor会自动检查最新停止状态和阶段完成。" }; continue; }
        this.record("target_navigation_resolved", { targetId: stage.targetId, model: result.args, movement, poseWorld: observation.control.pose_world });
        await this.commitMovement(movement, work, observation, signal); return;
      }
      if (result.name === "change_view") {
        const args = result.args as { direction: "left" | "right"; degrees: number; reason: string };
        await this.commitMovement({ maneuver: `turn_${args.direction}`, instruction: `Turn ${args.direction}.`,
          turnDegrees: args.degrees, expectedEffect: args.reason }, work, observation, signal); return;
      }
      if (result.name === "request_help") { await this.help(result.args, work, observation); return; }
      if (result.name === "select_semantic_instances") {
        const regions = this.semanticRegions(observation, result.args);
        if (!await this.reviewRegions(work, observation, regions, signal)) return;
        feedback = { observationMeasurementCompleted: true, next: "Use only visually confirmed target identity for navigation or completion; detector labels are candidate evidence." };
        continue;
      }
      if (result.name === "select_regions") {
        const regions = tools.originalPixels((result.args as { regions: Parameters<typeof tools.originalPixels>[0] }).regions,
          observation.frame.camera.width, observation.frame.camera.height, stage.kind);
        if (!await this.reviewRegions(work, observation, regions, signal)) return;
        feedback = { observationMeasurementCompleted: true, next: "测量已审核。参考当前空间边界证据和currentStep继续；Monitor自动检查阶段完成。" };
        continue;
      }
      if (result.name !== "navigate") throw new Error(`Unavailable proposal ${result.name}`);
      const args = result.args as { maneuver: string; instruction: string; expectedEffect: string; turnDegrees?: number; advanceMeters?: number };
      if (args.maneuver === "advance" && args.turnDegrees !== undefined
        || args.maneuver.startsWith("turn_") && args.advanceMeters !== undefined) {
        feedback = { rejected: "turnDegrees只用于turn，advanceMeters只用于advance。一次只提出一种动作。" };
        this.record("action_rejected", feedback); continue;
      }
      if (!/^[\x20-\x7e]+$/.test(args.instruction)) {
        feedback = { rejected: "LightNav instruction 必须使用简短英文。" };
        this.record("action_rejected", feedback); continue;
      }
      if (args.maneuver === "advance" && args.advanceMeters === undefined) {
        feedback = { rejected: "Use follow_target for a confirmed goal. Exploratory advance requires a bounded advanceMeters and a visible reason; use search_target for a measured body-turn observation checkpoint." };
        this.record("action_rejected", feedback); continue;
      }
      const duplicate = stage.attempts.find(a => a.maneuver === args.maneuver && a.endPose
        && ["mpc_blocked", "no_progress"].includes(a.result ?? "")
        && (a.distanceM ?? 0) < 0.25 && (a.yawRad ?? 0) < 0.2
        && motionDelta(a.endPose, observation.control.pose_world).distanceM < 0.3
        && motionDelta(a.endPose, observation.control.pose_world).yawRad < 0.25);
      if (duplicate) {
        feedback = { rejected: "Same ineffective maneuver at same pose. Different wording is not a strategy. Choose a supported different maneuver or request help.", failedAttempt: duplicate };
        this.record("repeated_strategy_rejected", { ...feedback as object, proposal: args }); continue;
      }
      await this.commitMovement(args, work, observation, signal);
      return;
    }
    await this.help({ reason: "本次决策多次未形成满足证据或恢复条件的可执行提议。", needed: "确认目标/选区，或人工改变受阻位置。" }, work, observation);
  }
}
