import { EventEmitter } from "node:events";
import { randomUUID } from "node:crypto";
import { mkdirSync, writeFileSync, readFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import type { Tool } from "@earendil-works/pi-ai";
import { PiDecision, type DecisionInput, type DecisionResult } from "./decision.js";
import { MonitorStore } from "./monitor-store.js";
import { SimClient, type SimStatus, type ObservationImage, type SceneFrame, type MotionSegment, type SpatialEvidence } from "./sim-client.js";
import { makePortal, portalClearance, insidePortalSides, isStopped, motionDelta, controlEventFact, navigationContract, bodyRadiusM,
  PassageObservationNeeded, currentTargetGeometry, usableVerticalSurface, targetGeometry, waitingZoneConditions, boundaryRelation, mergeBoundaryEvidence, type TargetAnchor,
  type Observation, type Portal, type Region, type Layer, type BoundaryEvidence } from "./monitor-evidence.js";
import { type DecisionMemory, memoryContext } from "./decision-memory.js";
import * as tools from "./monitor-tools.js";

export interface MonitorOptions {
  logPath: string; dbPath: string; maxRecoveries: number; decisionTimeoutS: number;
}
export function defaultMonitorOptions(): MonitorOptions {
  return { logPath: resolve(".local/monitor/events.jsonl"), dbPath: resolve(".local/monitor/tasks.sqlite"),
    maxRecoveries: 3, decisionTimeoutS: 30 };
}
export interface MonitorOutcome { status: "COMPLETED" | "NEEDS_USER" | "LIMIT_REACHED" | "ERROR" | "CANCELLED"; taskId: string; reason: string; completionAuthority?: "qwen"; physicalOutcome?: "not_evaluated" }
interface Attempt {
  maneuver: string; instruction: string; expectedEffect: string; controlTaskId: string | null;
  turnDegrees?: number;
  advanceMeters?: number;
  startPose: number[]; endPose?: number[]; result?: string; distanceM?: number; yawRad?: number;
}
interface Stage {
  kind: "pass_doorway" | "approach_target"; clause: string; target: string; instruction: string; distanceM?: number;
  navigationInstruction: string; strategyRevision: number; recoveries: number;
  stopCandidate?: SimStatus;
  targetId: string;
  status: "PENDING" | "ACTIVE" | "DEFERRED" | "COMPLETED"; portal?: Portal; anchor?: TargetAnchor;
  boundaries?: BoundaryEvidence[];
  attempts: Attempt[]; unmet: string[]; evidenceIds: string[]; completionReason?: string;
}
interface Task {
  id: string; revision: number; originalGoal: string; status: string; stages: Stage[]; currentStage: number;
  instanceId: string; estimatorEpoch: string; startedAt: number; pauseReason?: string;
  facts: unknown[]; claims: unknown[]; unknowns: string[];
  memory?: DecisionMemory;
  completionDeclaration?: { authority: "qwen"; reason: string; evidence: string; observationId: string; physicalOutcome: "not_evaluated" };
}
type Kind = "plan" | "stage" | "verify" | "recover" | "dialogue";
interface Work {
  kind: Kind; serial: number; reason: string; text?: string;
  resolve?: (value: string) => void; reject?: (error: Error) => void;
}
export interface ProposalDriver { decide(input: DecisionInput): Promise<DecisionResult> }

/** Coordinates Qwen decisions and execution; evidence verification never selects a semantic action. */
export class TaskMonitor extends EventEmitter {
  private readonly id = randomUUID();
  private readonly store: MonitorStore;
  private readonly driver: ProposalDriver;
  private readonly artifactDir: string;
  private task?: Task;
  private control!: SimStatus;
  private stopSamples: SimStatus[] = [];
  private stream = new AbortController();
  private listening?: Promise<void>;
  private heartbeatTimer?: ReturnType<typeof setInterval>;
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
  private navigationScope(): string {
    return `${this.task!.id}:${this.task!.revision}:${this.task!.currentStage}:${this.stage()!.strategyRevision}`;
  }
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
        if (saved.instanceId !== this.control.instance_id || saved.estimatorEpoch !== this.control.robot_state.epoch) {
          this.task.unknowns.push("Simulator instance changed: old odometry-frame evidence is invalid; start a new task.");
          for (const stage of saved.stages) { delete stage.portal; delete stage.boundaries; delete stage.anchor; }
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
    this.addStopSample(status);
    this.record("control_status", status);
    this.emit("control", status);
    if (!this.live()) return;
    if (status.instance_id !== this.task!.instanceId) {
      this.cancel("simulator instance changed"); this.finish("ERROR", "仿真实例已改变，需要重新开始任务。"); return;
    }
    if (status.robot_state.epoch !== this.task!.estimatorEpoch || status.robot_state.tracking !== "tracking") {
      this.cancel("odometry tracking lost or epoch changed");
      void this.stopMotion("odometry_unavailable").then(() => this.finish("ERROR", status.robot_state.tracking === "initializing"
        ? "里程计尚未完成初始化，不能开始导航；取得连续RGB-D观测后重新提交任务。"
        : "里程计失效或坐标系已改变，需要重新开始任务。"));
      return;
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
    const portal = this.stage()?.portal;
    const crossed = portal && portal.epoch === status.robot_state.epoch && portalClearance(portal, status.robot_state.pose_odom) >= navigationContract.doorway_exit_margin_m
      && portalClearance(portal, previous.robot_state.pose_odom) < navigationContract.doorway_exit_margin_m;
    if (portal && this.activeAttempt && portal.epoch === status.robot_state.epoch) {
      const pose = status.robot_state.pose_odom;
      const beforeEntry = (previous.robot_state.pose_odom[0]-portal.entryOrigin[0])*portal.normal[0]
        +(previous.robot_state.pose_odom[1]-portal.entryOrigin[1])*portal.normal[1];
      const afterEntry = (pose[0]-portal.entryOrigin[0])*portal.normal[0]+(pose[1]-portal.entryOrigin[1])*portal.normal[1];
      const insideSides = insidePortalSides(portal, pose);
      if (beforeEntry < 0 && afterEntry >= 0 && insideSides) portal.entered = true;
      if (afterEntry >= 0 && !insideSides) portal.entered = false;
      if (portal.entered && insideSides && crossed) portal.crossed = true;
    }
    if (transition && status.state === "ERROR") {
      this.endAttempt(status); this.cancel("controller error"); this.finish("ERROR", status.reason); return;
    }
    if (transition && status.state === "ARRIVAL_CANDIDATE"
      || transition && status.state === "PAUSED" && ["local_controller_blocked", "no_progress", "search_repeated", "turn_checkpoint", "advance_checkpoint", "target_checkpoint", "recovery_checkpoint", "recovery_unavailable", "recovery_expired"].includes(status.reason)) {
      if (status.state === "ARRIVAL_CANDIDATE") this.stage()!.stopCandidate = status;
      this.endAttempt(status); this.cancel(`controller ${status.state}/${status.reason}`);
      this.schedule(status.state === "ARRIVAL_CANDIDATE" || ["advance_checkpoint", "target_checkpoint"].includes(status.reason) ? "verify"
        : ["turn_checkpoint", "recovery_checkpoint"].includes(status.reason) ? "stage" : "recover", status.reason || status.state);
      return;
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
      instanceId: this.control.instance_id, estimatorEpoch: this.control.robot_state.epoch, startedAt: Date.now(), facts: [], claims: [], unknowns: [] };
    const task = this.task;
    const status = await this.client.status();
    if (serial !== this.serial) throw new Error("Goal request superseded before control invalidation");
    this.control = await this.client.action(this.id, status.generation, "invalidate", undefined, "goal_changed");
    if (serial !== this.serial) throw new Error("Goal request superseded after control invalidation");
    this.save("goal_changed", { originalGoal: goal, control: this.control });
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
    this.onControl(current); return current;
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
    if (!this.task) { this.say(reason); return; }
    this.task.status = status; this.task.pauseReason = reason;
    if (["COMPLETED", "CANCELLED"].includes(status) && this.task.memory) {
      this.record("decision_segment_archived", { ...this.task.memory, endReason: status });
    }
    this.save(status === "COMPLETED" ? "task_completed" : "task_stopped", { status, reason, control: this.control, completionDeclaration: this.task.completionDeclaration });
    this.say(reason);
    this.outcome?.({ status, taskId: this.task.id, reason,
      ...(this.task.completionDeclaration ? { completionAuthority: "qwen" as const, physicalOutcome: "not_evaluated" as const } : {}) }); this.outcome = undefined;
  }
  async close(): Promise<void> {
    if (this.closed) return;
    this.closed = true; this.cancel("monitor closing");
    clearInterval(this.heartbeatTimer); this.stream.abort();
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
  private addStopSample(status: SimStatus): void {
    if (!isStopped(status)) { this.stopSamples = []; return; }
    const last = this.stopSamples.at(-1);
    if (last && last.robot_state.epoch !== status.robot_state.epoch) this.stopSamples = [];
    if (!last || status.robot_state.stamp_s > last.robot_state.stamp_s) this.stopSamples.push(status);
    this.stopSamples = this.stopSamples.filter(sample => status.robot_state.stamp_s-sample.robot_state.stamp_s <= 2);
  }
  private stopped(status: SimStatus): boolean {
    const first = this.stopSamples[0], last = this.stopSamples.at(-1);
    return isStopped(status) && Boolean(first && last && first.robot_state.epoch === status.robot_state.epoch
      && last.robot_state.stamp_s-first.robot_state.stamp_s >= navigationContract.stopped_duration_s
      && status.robot_state.stamp_s-last.robot_state.stamp_s < .3);
  }
  private async observe(work: Work, signal: AbortSignal): Promise<Observation> {
    const before = await this.client.status(signal);
    this.addStopSample(before);
    // A synchronized observation is valid while the body settles. Continuous
    // measured stopping is read-only evidence, not permission to see, plan or complete.
    const frame = await this.client.captureScene(signal);
    const control = await this.client.status(signal);
    this.check(work, signal);
    if (before.generation !== control.generation) throw new Error("Controller changed while taking observation");
    this.control = control;
    if (frame.robot_state.epoch !== control.robot_state.epoch) throw new Error("Observation crossed an odometry epoch");
    const motion = await this.client.motion(signal);
    this.check(work, signal);
    if (motion && motion.epoch !== frame.robot_state.epoch) throw new Error("Motion segment belongs to another odometry epoch");
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
      if (!stage.portal) unmet.push("Need sensor-observed opposing supports, a free whole-body corridor and observed far-side expansion");
      else if (!stage.portal.crossed) unmet.push("Need this stage's estimated motion to cross the certified corridor from its entrance");
      else if (portalClearance(stage.portal, observation.control.robot_state.pose_odom) < navigationContract.doorway_exit_margin_m) {
        unmet.push("The whole A2 footprint has not cleared the observed far support extent plus the explicit exit margin");
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
      else if (target) {
        const relative = currentTargetGeometry(target.selectedLayer!, observation.frame.robot_state);
        unmet.push(...relative.unmet.filter(condition => stage.distanceM === undefined || !condition.startsWith("Prototype waiting zone")));
      }
      if (stage.anchor && stage.anchor.epoch !== observation.control.robot_state.epoch) unmet.push("Target odometry epoch changed");
    }
    if (observation.frame.robot_state.epoch !== observation.control.robot_state.epoch) unmet.push("Observation odometry epoch changed");
    if (observation.control.robot_state.stamp_s-observation.frame.robot_state.stamp_s > navigationContract.evidence_max_age_s) unmet.push("Current visual evidence has expired; observe again");
    if (observation.spatialEvidence?.footprint_states.some(value => value !== "free")) {
      unmet.push("Stopping volume or far-side expansion remains occupied/unknown; change view or position to observe it");
    }
    if (!this.stopped(observation.control)) unmet.push("Need zero commands and multiple fresh low-speed odometry samples including yaw rate");
    return unmet;
  }
  private monitorFeedback(stage: Stage, observation: Observation): unknown {
    const unmet = this.conditions(stage, observation);
    const identity = observation.identityReview;
    const rejected = identity?.identities.includes("not_target") || identity?.sameOpening === false;
    const uncertain = identity?.identities.includes("uncertain");
    const portal = stage.portal?.epoch === observation.control.robot_state.epoch ? stage.portal : undefined;
    const clearance = portal ? portalClearance(portal, observation.control.robot_state.pose_odom) : null;
    return {
      source: "read_only_semantic_observer", observationId: observation.id, frameId: observation.frame.frame_id,
      status: rejected ? "contradicted" : unmet.length ? "unverified" : "observed_conditions_met",
      meaning: "Read-only sensor reference, never a progression or completion gate. Missing evidence is not proof that passage did not happen. Qwen decides completion; physical outcome is evaluated offline.",
      unmetConditions: unmet, measurementIssues: observation.measurementIssues ?? [],
      targetIdentity: rejected ? "rejected" : uncertain ? "uncertain" : identity ? "reviewed" : "not_reviewed",
      identityReview: identity ?? null,
      passage: stage.kind === "pass_doorway" ? { certified: Boolean(portal),
        estimatedCrossingRecorded: portal?.crossed ?? false, measuredExitClearanceM: clearance,
        requiredExitMarginM: navigationContract.doorway_exit_margin_m,
        measuredCondition: clearance === null ? "not_observed"
          : clearance < 0 ? "not_fully_through_measured_portal"
          : clearance < navigationContract.doorway_exit_margin_m ? "exit_margin_unmet" : "exit_margin_met" } : null,
      stableStopVerified: this.stopped(observation.control), spatialEvidence: observation.spatialEvidence ?? null,
      robotState: observation.control.robot_state,
      stopCandidate: stage.stopCandidate ? { state: stage.stopCandidate.state,
        controlStampS: stage.stopCandidate.robot_state.stamp_s,
        model: stage.stopCandidate.lightnav_latest, navigationSessionId: stage.stopCandidate.navigation_session_id } : null,
    };
  }
  private context(work: Work, observation: Observation, extra: unknown, available: Tool[], purpose: string): unknown {
    const stage = this.stage();
    if (stage) stage.unmet = stage.status === "COMPLETED" ? [] : this.conditions(stage, observation);
    const status = observation.control;
    const scope = stage && this.navigationScope();
    const hint = observation.frame.lightnav_hint;
    const isReview = purpose === "review_measurements" || purpose === "measurement_failure";
    const doorwayReady = stage?.kind === "pass_doorway" && Boolean(stage.portal);
    const targetReady = stage?.kind === "approach_target" && Boolean(stage.anchor);
    const currentStep = {
      availableTools: available.map(tool => tool.name),
      objective: work.kind === "dialogue" ? "根据当前图像、任务状态和控制事实回答用户问题。"
        : isReview ? "核对当前像素身份和测量；证据失败时重新判断任务进展，可换视角、换阶段或生成新的导航指令。"
        : "结合完整任务、当前与历史画面及估计运动判断进展，由你选择执行哪个阶段、继续指令或补充观察。",
      decisionAuthority: "Qwen选择阶段和策略、生成实际instruction；LightNav执行局部轨迹；几何/local controller约束物理运动。",
      completionAuthority: "Qwen通过complete_task提交完成理由和证据，自主决定结束；只读观测不要求阶段证书。物理结果由离线评估独立判定。",
    };
    let passageTarget: unknown;
    if (stage?.portal) {
      const portal = stage.portal;
      const exitPoint = portal.exitPointOdomM;
      const dx = exitPoint[0]-status.robot_state.pose_odom[0], dy = exitPoint[1]-status.robot_state.pose_odom[1];
      const bearing = Math.atan2(dy, dx)-status.robot_state.pose_odom[2];
      passageTarget = { exitPointOdomM: exitPoint, distanceM: Math.hypot(dx, dy),
        bearingDegPositiveLeft: Math.atan2(Math.sin(bearing), Math.cos(bearing))*180/Math.PI,
        remainingNormalClearanceM: navigationContract.doorway_exit_margin_m-portalClearance(portal, status.robot_state.pose_odom) };
    }
    const depth = observation.frame.depth as { cells: { cell_id: number; valid_ratio: number; layers: { median_m: number }[] }[] };
    const compactLayer = (layer: Layer) => ({ layerId: layer.layer_id, medianM: layer.median_m,
      p10M: layer.p10_m, p90M: layer.p90_m, sampleOdomPointM: layer.sample_odom_point_m,
        supportOdomBoundsM: layer.support_odom_bounds_m,
        surface: layer.surface && { normalOdom: layer.surface.normal_odom, rmsM: layer.surface.rms_m,
        geometry: usableVerticalSurface(layer) ? "resolved_vertical_surface"
          : Math.abs(layer.surface.normal_odom[2]) > 0.9 && layer.surface.rms_m < 0.04 ? "horizontal_surface" : "mixed_or_unresolved_surface" } });
    return {
      originalGoal: this.task?.originalGoal, taskStatus: this.task?.status, currentStep,
      plan: this.task?.stages.map(({ kind, clause, target, instruction, status, unmet }, stageIndex) => ({
        stageIndex, kind, clause, target, instruction, status,
        verification: status === "COMPLETED" ? "verified" : "unverified", unmetConditions: unmet })),
      currentStage: this.task?.currentStage,
      recoveryBudget: { used: this.task?.stages.reduce((sum, item) => sum + item.recoveries, 0), maximum: this.options.maxRecoveries },
      monitorFeedback: stage ? this.monitorFeedback(stage, observation) : null,
      stage: stage && { kind: stage.kind, clause: stage.clause, targetId: stage.targetId,
        target: stage.target, instruction: stage.instruction, navigationInstruction: stage.navigationInstruction,
        strategyRevision: stage.strategyRevision, recoveries: stage.recoveries, explicitUserDistanceM: stage.distanceM,
        completionReference: { source: "read_only_observer", unmetConditions: stage.unmet },
        failedAttempts: stage.attempts.filter(attempt => ["local_controller_blocked", "no_progress"].includes(attempt.result ?? "")),
        targetAnchor: stage.anchor, passageTarget,
        boundaryEvidence: stage.boundaries?.map(({ id, region: r, measurementCount, currentFrameMaskIds }) => ({
          id, measurementCount, currentFrameMaskIds, observationId: r.observationId, poseOdom: r.poseOdom,
          description: r.description, semanticClaim: r.semanticClaim, odomPointM: r.selectedLayer!.sample_odom_point_m,
          supportOdomBoundsM: r.selectedLayer!.support_odom_bounds_m })),
        boundaryEvidenceMeaning: stage.kind === "pass_doorway"
          ? "B编号是当前通行开口的空间边界证据组，可包含同一位置的多个mask/视角；不是已证明的永久物体ID。过近的测量不能构成可用开口两侧。几何分离后仍需RGB确认其任务语义关系。" : undefined,
        targetGeometry: stage.anchor?.surface && targetGeometry(stage.anchor, status.robot_state.pose_odom, stage.distanceM),
        completionConvention: stage.kind === "approach_target" && stage.distanceM === undefined
          ? { source: navigationContract.source, ...navigationContract.waiting_zone } : undefined,
        portal: stage.portal && { observationId: stage.portal.observationId, origin: stage.portal.origin,
          normal: stage.portal.normal, bodyClearanceM: portalClearance(stage.portal, status.robot_state.pose_odom),
          requiredExitClearanceM: navigationContract.doorway_exit_margin_m,
          bearingDegPositiveLeft: Math.atan2(
            Math.sin(Math.atan2(stage.portal.origin[1]-status.robot_state.pose_odom[1], stage.portal.origin[0]-status.robot_state.pose_odom[0])-status.robot_state.pose_odom[2]),
            Math.cos(Math.atan2(stage.portal.origin[1]-status.robot_state.pose_odom[1], stage.portal.origin[0]-status.robot_state.pose_odom[0])-status.robot_state.pose_odom[2]))*180/Math.PI },
      },
      reason: work.reason, userQuestion: work.text,
      control: { state: status.state, reason: status.reason, poseOdom: status.robot_state.pose_odom,
        odometry: { epoch: status.robot_state.epoch, stampS: status.robot_state.stamp_s, source: status.robot_state.source, quality: status.robot_state.quality },
        estimatedTwistBase: status.robot_state.twist_base, effectiveCommand: status.effective_cmd,
        forceBrakeLatched: status.force_brake_latched, lightnav: status.lightnav_latest,
        referenceSource: status.reference_source, localController: status.local_controller, searchProgress: status.search_progress },
      controlEventFact: controlEventFact(status),
      currentObservation: { id: observation.id, stampS: observation.frame.robot_state.stamp_s,
        poseOdom: observation.frame.robot_state.pose_odom, camera: observation.frame.camera,
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
      capabilities: { observationSkill: "正常搜索由LightNav持续执行。change_view只用于当前具体测量缺少视角时的机身观察。", reverse: false,
        localNavigation: "continue_navigation lets LightNav choose and continuously update local trajectories under local controller body constraints. A missing target does not require a Qwen turn command.",
        precisionControl: "Qwen may request follow_measured_target when measured geometry is available; the controller executes it under unchanged constraints. Observer evidence is read-only; Qwen may declare completion regardless of certificate availability.",
        turnCheckpoint: "turnDegrees sets a control-tick measured yaw checkpoint (default 45°); local controller commands zero forward speed while turning. Actual compliant motion can drift.",
        boundedAdvance: available.some(tool => tool.name === "navigate")
          ? "advanceMeters executes a measured straight segment (0.1–2m) with an explicit local controller reference. LightNav keeps observing the semantic goal but does not override this primitive." : undefined,
        observations: "Current RGB-D and RGB-D/inertial estimated odometry; depth bands are not object segmentation.",
        distance: "ROI median is base-to-visible-point range. Waiting clearance is footprint-to-observed-plane distance; neither is route remaining distance." },
      rules: work.kind === "dialogue" ? [
        "回答当前问题，并区分实测事实、模型判断与未知信息。任务状态和控制事件以当前记录为准。",
      ] : isReview ? [
        "依据当前任务和实际RGB判断所选像素的身份；几何只说明这些测量支持什么空间关系。",
        "当前观测不足时可以重新选区、为具体缺失表面换视角，或让LightNav继续局部寻找。先前识别确实错误时明确更换保留证据。",
      ] : stage?.kind === "pass_doorway" ? doorwayReady ? [
        "通行开口及退出点已确认。由你选择继续LightNav、使用已测几何精确通过，或根据任务进展选择另一个阶段。",
        "Monitor用已测边界与最新里程计估计位姿核验完整穿越。已通过的边界不必仍出现在当前图中。",
      ] : [
        "当前开口完成证据尚不充分；这不能证明机器人还在原房间。结合前后图像与估计运动判断，应继续当前目标还是select_stage切换目标。",
        "实体身份和关联依据用户需求及实际RGB判断。mask_relations和boundaryMeasurementRelations只是测量关系；多个ID不能直接作为多个实体或两侧的证明。",
        "空间分开的两份边界证据与同一开口的视觉确认共同建立穿越目标。若先前识别错误，用replacePreviousIdentification=true明确纠正。",
      ] : [
        "当前画面不能辨认指定目标时，让LightNav继续寻找，或基于已发生的问题改变语义策略；正常转向无需你指定。候选身份仍需当前RGB核验。",
        "保持同一个目标。发现以前认错时用replacePreviousIdentification=true明确纠正；同一目标的新视角不需要填写内部association状态。",
        "lightnavTargetHint是另一导航模型在同帧深度上测得、重投影到当前图的目标指向假设。可用它定位待核验小区域，但仍需当前RGB确认，不能把提示本身当实体真值。",
        ...(targetReady ? ["可由你选择follow_measured_target精确停靠。到达后确认当前ROI身份，只读观测报告最新实测站位；由你决定完成。"] : []),
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
    const context = planning ? tools.planningContext(this.task!.originalGoal, extra)
      : this.context(work, observation, extra, available, purpose);
    const images = planning ? [] : observation.frame.images.filter(image => image.kind === "rgb" || image.kind === "semantic_rgb");
    const previous = this.task?.memory?.observation;
    if (!planning && previous) images.push({ kind: "previous_decision",
      label: `Previous accepted movement decision, stamp ${previous.stampS}s; compare with CURRENT RGB. Historical, not current.`,
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
      ({ role, bbox_px, description, semanticInstanceId, observationId: observation.id, poseOdom: observation.frame.robot_state.pose_odom, robotState: observation.frame.robot_state }));
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
        if ((result.robot_state as import("./sim-client.js").RobotState).epoch !== observation.frame.robot_state.epoch) throw new Error("Measurement odometry epoch changed");
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
          images.push({ kind: "retained_boundary", label: `Boundary evidence ${id}, observation ${region.observationId}, claim: ${region.semanticClaim}. Compare odometry-frame support and RGB to decide whether CURRENT measurements add another boundary of this opening.`,
            image: { type: "image", mimeType: "image/jpeg", data: readFileSync(region.cropImagePath).toString("base64") } });
        }
      }
      needsMeasurement = false;
      observation.measurementIssues = geometryErrors;
      const success = observation.regions.every(r => r.result && !r.error);
      const stageKind = this.stage()!.kind;
      const selections = [tools.selectFor(stageKind)];
      if (observation.frame.semantic_depth.instances.length) selections.push(tools.selectSemanticFor(stageKind,
        observation.frame.semantic_depth.instances.map(instance => instance.id)));
      const observationTools = this.decisionTools(work);
      const response = await this.propose(work, observation, success ? "review_measurements" : "measurement_failure",
        success ? [tools.reviewFor(stageKind, observation.regions.length), ...selections, ...observationTools] : [...selections, ...observationTools], signal,
        { requirement: stageKind === "pass_doorway"
          ? "依据当前任务核对所选边界区域及层成员；参考掩码重叠和里程计支持关系。同位置的多份测量可保留为一份边界证据，缺另一侧时继续观察。"
          : "核对指定目标的一块可见表面即可，不要求整扇门。RGB身份与垂直表面层成立就接受；否则重新选区或转向观察。",
          trial, geometryErrors }, images);
      if (await this.executeDecision(response, work, observation, signal)) return false;
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
      observation.identityReview = { identities: reviewedSelections.map(selection => selection.visualIdentity), sameOpening };
      if (replacePreviousIdentification) {
        const stage = this.stage()!;
        delete stage.anchor; delete stage.portal; delete stage.boundaries;
        stage.strategyRevision++;
        this.save("target_identity_corrected", { observationId: observation.id, selections: reviewedSelections });
      }
      if (reviewedSelections.some(selection => selection.visualIdentity !== "confirmed_target")) {
        geometryErrors = ["Identity is unconfirmed or rejected; these pixels are not a reliable geometric reference. This does not restrict Qwen navigation or completion decisions."];
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
            if (sameOpening === true) {
              let portal: Portal | undefined;
              try { portal = makePortal(observation, combined); }
              catch (error) {
                if (!(error instanceof PassageObservationNeeded)) throw error;
                geometryErrors.push(error.message);
              }
              if (portal) {
                const spatial = await this.client.spatialEvidence(portal.epoch, portal.stampS,
                  [], [...portal.corridorPoses, ...portal.expansionPoses], signal);
                this.check(work, signal); this.checkSpatial(spatial, observation);
                this.record("passage_spatial_evidence", { observationId: observation.id,
                  epoch: portal.epoch, footprintPosesOdom: [...portal.corridorPoses, ...portal.expansionPoses], result: spatial });
                if (spatial.point_states.every(value => value === "free") && spatial.footprint_states.every(value => value === "free")) {
                  portal.spatialEvidence = spatial; stage.portal = portal;
                } else geometryErrors.push("Passage corridor or far-side expansion is occupied or unobserved. Obtain a view of the missing side/exit; do not infer unseen door thickness.");
              }
            }
            else geometryErrors.push("Two spatially separated boundary groups are available. Confirm the visible opening between them with sameOpening=true; if the retained opening was wrong, explicitly replace that identification. Width alone does not identify an opening.");
          }
        }
      }
      if (stage.kind === "approach_target" && !geometryErrors.length) {
        const region = observation.regions[0], layer = region.selectedLayer!;
        const next: TargetAnchor = { id: stage.targetId, description: stage.target, relation: "wait_outside",
          epoch: observation.frame.robot_state.epoch, stampS: observation.frame.robot_state.stamp_s,
          observationId: observation.id, bboxPx: region.bbox_px, pointOdomM: layer.sample_odom_point_m,
          surface: layer.surface!, semanticClaim: region.semanticClaim!,
          referencePointOdomM: stage.anchor
            ? stage.anchor.referencePointOdomM : layer.sample_odom_point_m };
        if (stage.anchor?.surface) {
          const old = stage.anchor.surface, current = next.surface!;
          const agreement = Math.abs(old.normal_odom.reduce((sum, n, i) => sum+n*current.normal_odom[i], 0));
          const offset = Math.abs(old.normal_odom.reduce((sum, n, i) => sum+n*(current.centroid_odom_m[i]-old.centroid_odom_m[i]), 0));
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
      observation.measurementIssues = [];
      observation.reviewed = true;
      stage.evidenceIds.push(observation.id);
      this.task!.facts.push({ type: "measured_regions", observationId: observation.id, regions: observation.regions.map(r => ({ role: r.role, layer: r.selectedLayer })) });
      this.task!.claims.push({ type: "pixel_semantics", observationId: observation.id, claims: observation.regions.map(r => r.semanticClaim) });
      this.task!.facts = this.task!.facts.slice(-12); this.task!.claims = this.task!.claims.slice(-12);
      this.save("evidence_reviewed", { observationId: observation.id, regions: observation.regions, portal: stage.portal, target: stage.anchor });
      return true;
    }
    observation.measurementIssues = geometryErrors;
    this.record("measurement_review_exhausted", { observationId: observation.id, geometryErrors,
      monitorFeedback: this.monitorFeedback(this.stage()!, observation), decisionAuthority: "qwen" });
    const reassessment = { ...work, reason: "measurement_review_exhausted" };
    const decision = await this.propose(reassessment, observation, "progress_reassessment",
      this.decisionTools(reassessment), signal, {
        measurementAttempts: 3, geometryErrors,
        instruction: "These repeated reviews did not establish completion. Reassess progress using current vs prior RGB and estimated motion. Choose completion, the next stage, a new instruction/view, or explicitly request help; do not repeat this same failed review.",
      }, images);
    if (!await this.executeDecision(decision, reassessment, observation, signal)) throw new Error("Progress reassessment must select an executable Qwen decision");
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
      label: `Archived decision ${segment.id}, stage ${segment.stageIndex}, stamp ${segment.observation.stampS}s. Historical evidence requested for ${reason}.`,
      image: { type: "image", mimeType: "image/jpeg", data: readFileSync(segment.observation.imagePath).toString("base64") } }));
    this.record("memory_recalled", { reason, stageIndex, segmentIds: segments.map(segment => segment.id) });
    return { context: { reason, segments: segments.map(memoryContext) }, images };
  }
  private endAttempt(control: SimStatus): void {
    const attempt = this.activeAttempt;
    if (!attempt || attempt.result) return;
    Object.assign(attempt, { endPose: control.robot_state.pose_odom, result: control.reason || control.state,
      ...motionDelta(attempt.startPose, control.robot_state.pose_odom) });

    this.activeAttempt = undefined;
  }
  private async help(args: unknown, work: Work, observation: Observation): Promise<void> {
    const { reason, needed } = args as { reason: string; needed: string };
    if (work.serial !== this.serial) return;
    await this.stopMotion("needs_user", observation.control.generation);
    if (work.serial === this.serial) this.finish("NEEDS_USER", `${reason} 需要帮助：${needed}`);
  }
  private async commitMovement(args: { maneuver: string; instruction: string; expectedEffect: string; turnDegrees?: number; advanceMeters?: number;
    goalPointOdom?: number[]; goalHeadingRad?: number },
    work: Work, observation: Observation, signal: AbortSignal): Promise<void> {
    const stage = this.stage()!;
    if (!this.task) throw new Error("Movement requires an active task");
    const instruction = args.instruction;
    if (!/^[\x20-\x7e]+$/.test(instruction)) throw new Error("Qwen navigation instruction must be English");
    const movement = { maneuver: args.maneuver, advanceMeters: args.advanceMeters, semanticInstruction: instruction,
      goalPointOdom: args.goalPointOdom, goalHeadingRad: args.goalHeadingRad,
      positionToleranceM: navigationContract.local_goal.position_tolerance_m,
      headingToleranceDeg: navigationContract.local_goal.heading_tolerance_deg,
      turnDegrees: args.maneuver.startsWith("turn_") ? args.turnDegrees ?? 45 : undefined };
    const control = await this.client.action(this.id, observation.control.generation, "navigate", instruction, undefined, signal,
      this.navigationScope(), movement);
    this.check(work, signal);
    this.control = control;
    if (this.task.memory) {
      const previousMotion = control.previous_motion as MotionSegment | null;
      if (previousMotion?.control_task_id === this.task.memory.decision.controlTaskId) this.task.memory.motion = previousMotion;
      this.record("decision_segment_archived", { ...this.task.memory, endReason: "next_decision_accepted" });
    }
    this.task.memory = { id: randomUUID(), stageIndex: this.task.currentStage, targetId: stage.targetId,
      observation: { id: observation.id, imagePath: observation.imagePaths[0], stampS: observation.frame.robot_state.stamp_s,
        poseOdom: observation.frame.robot_state.pose_odom, epoch: observation.frame.robot_state.epoch },
      decision: { maneuver: args.maneuver, instruction, expectedEffect: args.expectedEffect,
        turnDegrees: movement.turnDegrees, advanceMeters: movement.advanceMeters,
        semanticInstruction: instruction, referenceSource: control.reference_source as string,
        goalPointOdom: args.goalPointOdom, goalHeadingRad: args.goalHeadingRad,
        controlTaskId: control.task_id!, acceptedStampS: control.robot_state.stamp_s }, motion: null };
    this.activeAttempt = { ...args, ...movement, instruction, controlTaskId: control.task_id, startPose: control.robot_state.pose_odom };
    stage.attempts.push(this.activeAttempt); stage.attempts = stage.attempts.slice(-12);
    this.save("action_executed", { stageIndex: this.task.currentStage, attempt: this.activeAttempt,
      buffer: this.task.memory, proposedInstruction: args.instruction, control });
    this.say(`第 ${this.task.currentStage + 1} 阶段执行：${instruction}`);
  }
  private decisionTools(work: Work): Tool[] {
    const stage = this.stage()!;
    const available = [tools.continueNavigation, tools.selectStage(this.task!.stages.length), tools.changeView, tools.help];
    if (stage.portal || stage.anchor?.surface) available.push(tools.precisionNavigation);
    available.push(tools.completeTask);
    return available;
  }
  private async consumeRecovery(work: Work, observation: Observation): Promise<boolean> {
    const stage = this.stage()!;
    if (!stage.stopCandidate && work.kind !== "recover") return true;
    const taskRecoveries = this.task!.stages.reduce((sum, item) => sum + item.recoveries, 0);
    if (taskRecoveries >= this.options.maxRecoveries) {
      await this.stopMotion("recovery_limit", observation.control.generation);
      this.finish("LIMIT_REACHED", "整个任务已达到恢复次数上限；观测和进度已保留。");
      return false;
    }
    stage.recoveries++;
    this.save("recovery_started", { stageIndex: this.task!.currentStage, recovery: stage.recoveries,
      taskRecoveries: taskRecoveries + 1, maxTaskRecoveries: this.options.maxRecoveries,
      reason: work.reason, authority: "budget_for_qwen_selected_action", control: observation.control });
    return true;
  }
  private async executeDecision(result: DecisionResult, work: Work, observation: Observation, signal: AbortSignal): Promise<boolean> {
    if (result.name === "request_help") { await this.help(result.args, work, observation); return true; }
    if (result.name === "complete_task") {
      await this.declareCompletion(result.args as { reason: string; evidence: string }, work, observation, signal); return true;
    }
    if (!["select_stage", "continue_navigation", "change_view", "follow_measured_target"].includes(result.name)) return false;
    const args = result.args as { stageIndex?: number; instruction: string; reason: string; direction?: "left" | "right"; degrees?: number };
    if (!/^[\x20-\x7e]+$/.test(args.instruction)) throw new Error("Qwen navigation instruction must be English");
    const previous = this.stage()!;
    const feedback = this.monitorFeedback(previous, observation);
    if (!await this.consumeRecovery(work, observation)) return true;
    this.save("navigation_decision", { authority: "qwen", tool: result.name, args,
      stageIndex: this.task!.currentStage, monitorFeedback: feedback });
    if (result.name === "select_stage" && args.stageIndex !== this.task!.currentStage) {
      const next = this.task!.stages[args.stageIndex!];
      if (!next) throw new Error("Qwen selected a stage outside the plan");
      const previousIndex = this.task!.currentStage;
      if (previous.status !== "COMPLETED") previous.status = "DEFERRED";
      if (next.status !== "COMPLETED") next.status = "ACTIVE";
      this.task!.currentStage = args.stageIndex!;
      this.task!.revision++;
      this.save("stage_selected", { authority: "qwen", previousStageIndex: previousIndex,
        currentStage: args.stageIndex, reason: args.reason, instruction: args.instruction,
        previousStageStatus: previous.status, previousStageEvidence: feedback,
        completionGrantedBySelection: false, sessionResetExpected: true });
      // Region reviews belong to the old target; the RGB frame is still a valid observation.
      observation.regions = []; observation.reviewed = false;
      delete observation.identityReview; delete observation.measurementIssues; delete observation.spatialEvidence;
    }
    if (result.name === "select_stage" || result.name === "continue_navigation") {
      await this.continueLocal(args, work, observation, signal);
    } else if (result.name === "change_view") {
      this.record("observation_skill_proposed", { authority: "qwen", observationId: observation.id, ...args });
      await this.commitMovement({ maneuver: `turn_${args.direction}`, instruction: args.instruction,
        turnDegrees: args.degrees, expectedEffect: args.reason }, work, observation, signal);
    } else if (result.name === "follow_measured_target") {
      const movement = this.targetMovement(this.stage()!, observation.control.robot_state.pose_odom, args.reason, args.instruction);
      if (!movement) throw new Error("Measured goal is already reached; choose evidence collection or another stage");
      this.record("precision_navigation_started", { authority: "qwen", stageIndex: this.task!.currentStage, movement });
      await this.commitMovement(movement, work, observation, signal);
    }
    delete previous.stopCandidate;
    delete this.stage()!.stopCandidate;
    return true;
  }
  private async continueLocal(args: { instruction: string; reason: string }, work: Work, observation: Observation, signal: AbortSignal): Promise<void> {
    const stage = this.stage()!;
    const previousInstruction = stage.navigationInstruction;
    const feedback = this.monitorFeedback(stage, observation);
    stage.navigationInstruction = args.instruction;
    const scope = this.navigationScope();
    this.save(stage.stopCandidate ? "stop_continuation_feedback" : "navigation_instruction_selected", {
      authority: "qwen", stageIndex: this.task!.currentStage, recovery: stage.recoveries,
      previousInstruction, instruction: args.instruction, qwenReason: args.reason,
      evidence: feedback, navigationScope: scope,
      sessionBefore: observation.control.navigation_session_id,
      sessionResetExpected: observation.control.navigation_scope !== scope,
      historyPolicy: observation.control.navigation_scope === scope ? "preserve_same_target_history" : "qwen_selected_target_scope",
    });
    await this.commitMovement({ maneuver: "lightnav", instruction: args.instruction,
      expectedEffect: args.reason }, work, observation, signal);
  }
  private targetMovement(stage: Stage, pose: number[], reason: string, instruction: string): {
    maneuver: string; instruction: string; expectedEffect: string; goalPointOdom: number[]; goalHeadingRad: number;
  } | null {
    let point: number[], heading: number;
    if (stage.kind === "pass_doorway") {
      if (!stage.portal) throw new Error("A measured doorway is required for follow_target");
      if (portalClearance(stage.portal, pose) >= navigationContract.doorway_exit_margin_m) return null;
      point = stage.portal.exitPointOdomM;
      heading = Math.atan2(stage.portal.normal[1], stage.portal.normal[0]);
    } else {
      if (!stage.anchor?.surface) throw new Error("A measured target is required for follow_target");
      if (stage.distanceM === undefined && !waitingZoneConditions(stage.anchor, pose).length) return null;
      point = targetGeometry(stage.anchor, pose, stage.distanceM).standingPointOdomM;
      heading = Math.atan2(-stage.anchor.surface.normal_odom[1], -stage.anchor.surface.normal_odom[0]);
    }
    return { maneuver: "follow_target", instruction,
      expectedEffect: reason, goalPointOdom: point, goalHeadingRad: heading };
  }
  private checkSpatial(spatial: SpatialEvidence, observation: Observation): void {
    if (spatial.epoch !== observation.frame.robot_state.epoch || spatial.frame_id !== "odom") throw new Error("Spatial evidence frame changed");
    if (spatial.map_stamp_s < observation.frame.robot_state.stamp_s) throw new Error("Spatial evidence predates the observation");
  }
  private async declareCompletion(args: { reason: string; evidence: string }, work: Work,
    observation: Observation, signal: AbortSignal): Promise<void> {
    const reason = `Qwen声明任务完成：${args.reason}（物理结果由离线评估独立判定）`;
    this.control = await this.client.action(this.id, observation.control.generation, "complete", undefined, reason, signal);
    this.check(work, signal);
    this.task!.completionDeclaration = { authority: "qwen", reason: args.reason, evidence: args.evidence,
      observationId: observation.id, physicalOutcome: "not_evaluated" };
    this.save("qwen_completion_declared", { ...this.task!.completionDeclaration,
      observerEvidence: this.monitorFeedback(this.stage()!, observation), control: this.control });
    this.finish("COMPLETED", reason);
  }
  private async handle(work: Work, signal: AbortSignal): Promise<void> {
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
        const result = await this.propose(work, observation, "plan_task", [tools.plan, tools.help], signal,
          { rejectedPlanReasons: errors, rejectedPlan: errors.length ? stages : undefined });
        if (result.name === "request_help") { await this.help(result.args, work, observation); return; }
        stages = (result.args as { stages: Stage[] }).stages;
        const normalized = (s: string) => s.replace(/[\s，。！？、,.!?；;：:]/g, "");
        errors = [];
        const submittedClauses = stages.map(s => s.clause).join("");
        if (normalized(submittedClauses) !== normalized(this.task.originalGoal)) errors.push(
          `clause遗漏或改写了用户要求。完整原文=${JSON.stringify(this.task.originalGoal)}；本次拼接=${JSON.stringify(submittedClauses)}。单阶段请将clause完整复制为originalGoal，多阶段请逐段完整覆盖。`);
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
        navigationInstruction: stage.instruction, strategyRevision: 0, recoveries: 0,
        status: "PENDING", attempts: [], unmet: [], evidenceIds: [] }));
      this.task.currentStage = 0; this.task.status = "RUNNING";
      this.save("task_planned", { stages: this.task.stages, originalGoal: this.task.originalGoal,
        navigationContract, control: observation.control });
      this.say(`任务已分为 ${stages.length} 个阶段：${stages.map(s => s.clause).join(" → ")}`);
      this.schedule("stage", "plan_ready"); return;
    }
    this.task.status = "RUNNING";
    const stage = this.stage()!;
    if (stage.status !== "COMPLETED") stage.status = "ACTIVE";
    let feedback: unknown = {};
    let recalled: { context: unknown; images: ObservationImage[] } | undefined;
    if (work.kind === "recover") {
      feedback = { controllerEvent: work.reason, control: observation.control,
        instruction: "Choose the next strategy from the measured control outcome. No recovery or instruction has been selected by Monitor." };
    }
    for (let round = 0; round < 5; round++) {
      const available = this.decisionTools(work);
      if (!observation.reviewed && !(stage.kind === "pass_doorway" && stage.portal)
        && observation.frame.semantic_depth.instances.length) available.unshift(tools.selectSemanticFor(stage.kind,
          observation.frame.semantic_depth.instances.map(instance => instance.id)));
      const hint = observation.frame.lightnav_hint;
      const scope = this.navigationScope();
      if (stage.kind === "approach_target" && !observation.reviewed && hint?.navigation_scope === scope && hint.in_view && hint.point_1000) {
        available.unshift(tools.inspectHint);
      }
      if (work.kind === "recover") available.push(tools.navigate);
      else if (observation.spatialEvidence?.footprint_states.some(value => value !== "free")) available.push(tools.navigate);
      if (!recalled && this.task.memory) available.push(tools.recall);
      if (!observation.reviewed && !(stage.kind === "pass_doorway" && stage.portal)) available.unshift(tools.selectFor(stage.kind));
      const result = await this.propose(work, observation,
        work.kind === "recover" ? "error_recovery" : work.kind === "verify" ? "verify_arrival" : "stage_decision",
        available, signal, { feedback, recalledEvidence: recalled?.context }, recalled?.images);
      if (result.name === "recall_memory") { recalled = this.recall(result.args); continue; }
      if (await this.executeDecision(result, work, observation, signal)) return;
      if (result.name === "inspect_target_hint") {
        const [u, v] = hint!.point_1000!;
        const { width, height } = observation.frame.camera;
        const x = u*width/1000, y = v*height/1000;
        const region: Region = { role: "target", description: "Candidate patch around the reprojected LightNav semantic target; identity still requires current RGB review.",
          bbox_px: [Math.max(0, Math.floor(x-width*.025)), Math.max(0, Math.floor(y-height*.04)),
            Math.min(width, Math.ceil(x+width*.025)), Math.min(height, Math.ceil(y+height*.04))] };
        this.record("target_hint_inspected", { observationId: observation.id, hint, region });
        if (!await this.reviewRegions(work, observation, [region], signal)) return;
        feedback = { observationMeasurementCompleted: true, next: "候选测量已审核。根据currentStep继续；由你决定继续导航或complete_task；测量仅供参考。" };
        continue;
      }
      if (result.name === "select_semantic_instances") {
        const regions = this.semanticRegions(observation, result.args);
        if (!await this.reviewRegions(work, observation, regions, signal)) return;
        feedback = { observationMeasurementCompleted: true, next: "Detector labels and geometric measurements are candidate evidence. Qwen decides navigation and completion from the full observation and task history." };
        continue;
      }
      if (result.name === "select_regions") {
        const regions = tools.originalPixels((result.args as { regions: Parameters<typeof tools.originalPixels>[0] }).regions,
          observation.frame.camera.width, observation.frame.camera.height, stage.kind);
        if (!await this.reviewRegions(work, observation, regions, signal)) return;
        feedback = { observationMeasurementCompleted: true, next: "测量已审核。参考当前空间边界证据和currentStep继续；由你决定继续导航或complete_task；测量仅供参考。" };
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
        feedback = { rejected: "Use continue_navigation for continuous local search/approach. A recovery advance requires bounded advanceMeters and a specific observed reason." };
        this.record("action_rejected", feedback); continue;
      }
      if (!await this.consumeRecovery(work, observation)) return;
      this.save("navigation_decision", { authority: "qwen", tool: result.name, args,
        stageIndex: this.task.currentStage, monitorFeedback: this.monitorFeedback(stage, observation) });
      await this.commitMovement(args, work, observation, signal);
      delete stage.stopCandidate;
      return;
    }
    const decision = await this.propose(work, observation, "progress_reassessment", this.decisionTools(work), signal,
      { feedback, instruction: "Observation round finished. Decide completion, continue navigation, select a stage, change view or explicitly request help; no measurement certificate is required." });
    if (!await this.executeDecision(decision, work, observation, signal)) throw new Error("Progress reassessment must select an executable Qwen decision");
  }
}
