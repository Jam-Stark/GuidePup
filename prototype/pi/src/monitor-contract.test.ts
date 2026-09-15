import assert from "node:assert/strict";
import { once } from "node:events";
import { mkdtempSync, readFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { setImmediate as nextTurn } from "node:timers/promises";
import test, { type TestContext } from "node:test";
import { validateToolCall } from "@earendil-works/pi-ai";
import type { DecisionInput, DecisionResult } from "./decision.js";
import { SimClient, type SimStatus, type SceneFrame, type RegionFrame, type ObservationImage, type MotionSegment } from "./sim-client.js";
import { TaskMonitor, type ProposalDriver } from "./task-monitor.js";

class Inbox<T> {
  private values: T[] = [];
  private receivers: ((value: T) => void)[] = [];
  push(value: T): void {
    const receiver = this.receivers.shift();
    if (receiver) receiver(value); else this.values.push(value);
  }
  take(): Promise<T> {
    return this.values.length ? Promise.resolve(this.values.shift()!)
      : new Promise(resolve => this.receivers.push(resolve));
  }
}

interface Proposal {
  input: DecisionInput;
  reply(name: string, args: Record<string, unknown>): void;
}

/** Deliberately ignores abort, as an already-running model request may do. */
class ControlledDriver implements ProposalDriver {
  readonly calls: Proposal[] = [];
  private pending = new Inbox<Proposal>();
  decide(input: DecisionInput): Promise<DecisionResult> {
    return new Promise(resolve => {
      const proposal = { input, reply(name: string, args: Record<string, unknown>): void {
        assert.ok(input.tools.some(tool => tool.name === name), `${name} is unavailable during ${input.purpose}`);
        const validated: unknown = validateToolCall(input.tools, {
          type: "toolCall", id: "contract-proposal", name, arguments: args,
        });
        resolve({ name, args: validated, text: "", usage: null, elapsedMs: 0 });
      } };
      this.calls.push(proposal); this.pending.push(proposal);
    });
  }
  next(): Promise<Proposal> { return this.pending.take(); }
}

const pixelPng = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jHuoAAAAASUVORK5CYII=";
const image = (kind: string, label: string): ObservationImage => ({
  kind, label, image: { type: "image", mimeType: "image/png", data: pixelPng },
});
type ActionName = "navigate" | "pause" | "invalidate" | "complete";
interface ActionCall { generation: number; action: ActionName; instruction?: string; reason?: string }

/** A deterministic control protocol, without physics, network, or model calls. */
class FakeSim extends SimClient {
  current: SimStatus = {
    instance_id: "contract-simulator", generation: 0, event_id: 0, task_id: null,
    state: "IDLE", instruction: "", reason: "", pose_world: [0, 0, 0],
    actual_velocity_body: [0, 0, 0], effective_cmd: [0, 0, 0],
    scene_frame_id: "sse-frame-never-returned-by-scene", motion_owner: null,
    sim_time_s: 0,
  };
  readonly actions: ActionCall[] = [];
  readonly captures: SceneFrame[] = [];
  readonly measurements: { frameId: string; bboxPx: number[] }[] = [];
  failMeasurement = false;
  private monitorId?: string;
  private updates = new Inbox<SimStatus | undefined>();
  private motionRecord: MotionSegment | null = null;
  private snapshot(): SimStatus { return structuredClone(this.current); }
  transition(patch: Partial<SimStatus>): SimStatus {
    this.current = { ...this.current, ...patch,
      generation: this.current.generation + 1, event_id: this.current.event_id + 1,
      sim_time_s: (this.current.sim_time_s as number) + 0.1 };
    if (this.motionRecord) {
      const actual = this.motionRecord.actual;
      const previous = actual.samples.at(-1)!.pose_world;
      actual.path_length_m += Math.hypot(this.current.pose_world[0]-previous[0], this.current.pose_world[1]-previous[1]);
      actual.samples.push({ sim_time_s: this.current.sim_time_s as number, pose_world: [...this.current.pose_world] });
      actual.sample_count++;
      this.motionRecord.end_sim_time_s = this.current.sim_time_s as number;
      this.motionRecord.events.push({ sim_time_s: this.current.sim_time_s as number, state: this.current.state, reason: this.current.reason });
    }
    const status = this.snapshot(); this.updates.push(status); return status;
  }
  expireLease(): void { this.monitorId = undefined; }
  override async status(): Promise<SimStatus> { return this.snapshot(); }
  override async motion(): Promise<MotionSegment | null> { return structuredClone(this.motionRecord); }
  override async claim(id: string): Promise<SimStatus> {
    assert.ok(!this.monitorId || this.monitorId === id, "claim must not steal an active monitor lease");
    this.monitorId = id;
    return this.transition({ state: this.current.state === "HUMAN_PAUSED" ? "HUMAN_PAUSED" : "PAUSED",
      reason: this.current.state === "HUMAN_PAUSED" ? this.current.reason : "monitor_claim",
      instruction: "", task_id: null, effective_cmd: [0, 0, 0] });
  }
  override async heartbeat(id: string): Promise<SimStatus> {
    assert.equal(id, this.monitorId); return this.snapshot();
  }
  override async action(id: string, generation: number, action: ActionName, instruction?: string,
    reason?: string, signal?: AbortSignal, navigationScope?: string,
    movement?: { maneuver: string; turnDegrees?: number; advanceMeters?: number }): Promise<SimStatus> {
    signal?.throwIfAborted();
    assert.equal(id, this.monitorId, "monitor lease");
    assert.equal(generation, this.current.generation, "stale generation reached simulator");
    this.actions.push({ generation, action, instruction, reason });
    if (action === "navigate") {
      assert.notEqual(this.current.state, "HUMAN_PAUSED");
      assert.ok(navigationScope);
      assert.ok(movement?.maneuver);
      const previousMotion = await this.motion();
      const result = this.transition({ state: "NAVIGATING", reason: "", instruction: instruction!,
        task_id: `local-${generation + 1}`, motion_owner: "monitor", effective_cmd: [0.2, 0, 0] });
      this.motionRecord = { control_task_id: result.task_id!, start_sim_time_s: result.sim_time_s as number,
        end_sim_time_s: result.sim_time_s as number, source: "simulator_pose",
        actual: { samples: [{ sim_time_s: result.sim_time_s as number, pose_world: [...result.pose_world] }],
          sample_count: 1, path_length_m: 0, net_displacement_m: 0, yaw_change_rad: 0 },
        plans: [], plan_count: 0, events: [], log_paths: { telemetry: "fixture", lightnav: "fixture" } };
      return { ...result, previous_motion: previousMotion };
    }
    if (action === "complete") {
      assert.ok(["PAUSED", "ARRIVAL_CANDIDATE"].includes(this.current.state));
      return this.transition({ state: "LOCAL_DONE", reason: reason ?? "local_complete", effective_cmd: [0, 0, 0] });
    }
    const human = this.current.state === "HUMAN_PAUSED";
    return this.transition({ state: human ? "HUMAN_PAUSED" : "PAUSED",
      reason: human ? this.current.reason : reason ?? action, effective_cmd: [0, 0, 0],
      ...(action === "invalidate" ? { instruction: "", task_id: null } : {}) });
  }
  override async pause(reason = "human"): Promise<SimStatus> {
    return this.transition({ state: "HUMAN_PAUSED", reason, motion_owner: "human", effective_cmd: [0, 0, 0] });
  }
  override async resume(): Promise<SimStatus> {
    return this.transition({ state: "IDLE", reason: "human_resume", instruction: "", task_id: null,
      effective_cmd: [0, 0, 0] });
  }
  override async captureScene(): Promise<SceneFrame> {
    const frameId = `actual-scene-${this.captures.length + 1}`;
    const frame: SceneFrame = {
      frame_id: frameId, sim_time_s: this.captures.length / 10,
      pose_world: [...this.current.pose_world], depth: { valid_ratio: 1, cells: [] },
      semantic_depth: { frame_id: frameId, instances: [], mask_relations: [] },
      camera: { width: 640, height: 360, hfov_deg: 69, fovy_deg: 42,
        position_base_m: [0.3, 0, 0.1], pitch_down_deg: 5 },
      images: [image("rgb", `Current original RGB ${frameId}`), image("grid", `Depth overview ${frameId}`)],
    };
    this.captures.push(frame); this.lastScene = frame; return frame;
  }
  override async measureRegion(frameId: string, bboxPx: number[]): Promise<RegionFrame> {
    this.measurements.push({ frameId, bboxPx });
    if (this.failMeasurement) throw new Error("/measure_region: HTTP 409: frame is not cached");
    assert.ok(this.captures.some(frame => frame.frame_id === frameId), "measure must reference an actually read scene");
    const point = bboxPx[0] > 100 ? [5.35, 0, 0.6] : [2, bboxPx[0] === 10 ? -1.1 : 1.1, 0.6];
    return { frame_id: frameId, status: "measured", bbox_px: bboxPx,
      layers: [{ layer_id: "D1", median_m: 2, p10_m: 1.98, p90_m: 2.02, count: 100,
        sample_world_point_m: point, sample_base_point_m: point,
        surface: { centroid_world_m: point, normal_world: [-1, 0, 0], rms_m: 0, point_count: 100, in_plane_std_m: [0.03, 0.2] } }],
      images: [image("selection", "Selection context"), image("crop", `RGB crop ${frameId}`),
        image("distance_layers", `Distance membership ${frameId}`)] };
  }
  override async *events(signal: AbortSignal): AsyncGenerator<SimStatus> {
    const end = (): void => this.updates.push(undefined);
    signal.addEventListener("abort", end, { once: true });
    try {
      while (!signal.aborted) {
        const status = await this.updates.take();
        if (signal.aborted || status === undefined) return;
        yield status;
      }
    } finally { signal.removeEventListener("abort", end); }
  }
}

interface JournalEvent { seq: number; task_id: string | null; kind: string; data: Record<string, unknown> }
async function harness(t: TestContext) {
  const directory = mkdtempSync(join(tmpdir(), "a2-monitor-contract-"));
  const logPath = join(directory, "events.jsonl");
  const options = { logPath, dbPath: join(directory, "tasks.sqlite"), timeoutS: 60,
    maxRecoveries: 3, decisionTimeoutS: 60 };
  const sim = new FakeSim();
  const monitors: TaskMonitor[] = [];
  t.after(async () => {
    try { for (const monitor of monitors.reverse()) await monitor.close(); }
    finally { rmSync(directory, { recursive: true, force: true }); }
  });
  const create = async () => {
    const driver = new ControlledDriver();
    const monitor = new TaskMonitor(sim, options, driver); monitors.push(monitor);
    await monitor.connect();
    return { monitor, driver };
  };
  const journal = (): JournalEvent[] => readFileSync(logPath, "utf8").trim().split("\n").map(line => JSON.parse(line));
  return { sim, create, journal, ...(await create()) };
}

const approach = { kind: "approach_target", clause: "走到电梯门口", target: "elevator doors", instruction: "Approach the elevator doors." };
const doorway = { kind: "pass_doorway", clause: "出门", target: "open doorway", instruction: "Go through the open doorway." };
const selectedTarget = { regions: [{ role: "target", bbox_1000: [785, 24, 933, 507], description: "Metal elevator door panel" }] };
const acceptedTarget = { selections: [{ regionIndex: 0, layerId: "D1", semanticClaim: "Crop and D1 pixels show the selected metal panel.", association: "first_observation" }] };
const selectedJambs = { regions: [
  { role: "left_jamb", bbox_1000: [16, 28, 31, 83], description: "Left solid jamb" },
  { role: "right_jamb", bbox_1000: [47, 28, 62, 83], description: "Right solid jamb" },
] };
const acceptedJambs = { selections: [0, 1].map(regionIndex => ({
  regionIndex, layerId: "D1", semanticClaim: "The selected crop and D1 layer show this solid jamb.",
  association: "first_observation",
})) };
const advance = (instruction: string) => ({ maneuver: "advance", instruction, expectedEffect: "Approach the visible target." });
const help = { reason: "The observation is insufficient.", needed: "Please confirm the target." };
const names = (proposal: Proposal): string[] => proposal.input.tools.map(tool => tool.name).sort();

async function plan(monitor: TaskMonitor, driver: ControlledDriver, stages = [approach], goal = "走到电梯门口") {
  await monitor.startGoal(goal);
  const proposal = await driver.next(); assert.equal(proposal.input.purpose, "plan_task");
  assert.deepEqual(proposal.input.images, [], "planning must preserve the text request independently of the current view");
  proposal.reply("plan_task", { stages });
  return driver.next();
}

async function finishDoorway(monitor: TaskMonitor, driver: ControlledDriver, sim: FakeSim): Promise<Proposal> {
  const stage = await plan(monitor, driver, [doorway, approach], "出门走到电梯门口");
  stage.reply("select_regions", selectedJambs);
  const review = await driver.next(); review.reply("accept_measurements", acceptedJambs);
  const navigation = await driver.next();
  const executed = once(monitor, "message"); navigation.reply("follow_target", { reason: "Pass through the measured doorway." }); await executed;
  sim.transition({ state: "ARRIVAL_CANDIDATE", reason: "lightnav_stop", pose_world: [3, 0, 0], effective_cmd: [0, 0, 0] });
  const verification = await driver.next(); assert.equal(verification.input.purpose, "verify_arrival");
  verification.reply("propose_stage_complete", { reason: "Fresh observation and measured portal show the complete A2 footprint beyond the doorway, with zero motion." });
  return driver.next();
}

test("a plan omitting the elevator and inventing 4.5 m is rejected, then a corrected complete plan may execute", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  await h.monitor.startGoal("出门走到电梯门口");
  const initial = await h.driver.next();
  assert.equal(initial.input.purpose, "plan_task");
  assert.deepEqual(initial.input.images, []);
  initial.reply("plan_task", { stages: [{ kind: "approach_target", clause: "靠近门框", target: "door frame",
    instruction: "Approach the door frame.", distanceM: 4.5 }] });
  const correction = await h.driver.next();
  assert.equal(correction.input.purpose, "plan_task");
  assert.deepEqual(correction.input.images, []);
  const context = correction.input.context as { originalGoal: string; extra: { rejectedPlanReasons: string[] } };
  assert.equal(context.originalGoal, "出门走到电梯门口");
  assert.ok(context.extra.rejectedPlanReasons.some(reason => reason.includes("clause")));
  assert.ok(context.extra.rejectedPlanReasons.some(reason => reason.includes("distanceM")));
  assert.equal(h.sim.actions.filter(call => ["navigate", "complete"].includes(call.action)).length, 0);
  assert.ok(!h.journal().some(event => event.kind === "task_planned"));
  correction.reply("plan_task", { stages: [doorway, approach] });
  const stage = await h.driver.next();
  assert.equal(stage.input.purpose, "stage_decision");
  const planned = h.journal().find(event => event.kind === "task_planned")!;
  const stages = planned.data.stages as { clause: string; distanceM?: number }[];
  assert.deepEqual(stages.map(item => item.clause), ["出门", "走到电梯门口"]);
  assert.ok(stages.every(item => item.distanceM === undefined));
  const executed = once(h.monitor, "message");
  stage.reply("navigate", { maneuver: "turn_right", instruction: "Turn right.", expectedEffect: "Bring both doorway jambs into view." });
  await executed;
  const events = h.journal();
  assert.ok(events.find(event => event.kind === "plan_rejected")!.seq < planned.seq);
  assert.ok(planned.seq < events.find(event => event.kind === "action_executed")!.seq);
});

test("normalized ROI becomes original pixels on the actual /scene frame, never the SSE frame", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const stage = await plan(h.monitor, h.driver);
  stage.reply("select_regions", selectedTarget);
  const review = await h.driver.next(); assert.equal(review.input.purpose, "review_measurements");
  const frame = h.sim.captures.at(-1)!;
  assert.notEqual(frame.frame_id, h.sim.current.scene_frame_id);
  assert.equal(h.sim.measurements[0].frameId, frame.frame_id);
  assert.deepEqual(h.sim.measurements[0].bboxPx, [502, 8, 598, 183]);
  assert.equal(h.journal().find(event => event.kind === "measurement_requested")!.data.frameId, frame.frame_id);
  assert.ok(review.input.images.some(entry => entry.label.includes(`RGB crop ${frame.frame_id}`)));
});

test("a failed measurement permits reselection or a new observation but cannot advance or complete", { timeout: 10_000 }, async t => {
  const h = await harness(t); h.sim.failMeasurement = true;
  const stage = await plan(h.monitor, h.driver); stage.reply("select_regions", selectedTarget);
  const failure = await h.driver.next(); assert.equal(failure.input.purpose, "measurement_failure");
  assert.deepEqual(names(failure), ["change_view", "request_help", "select_regions"]);
  const stopped = once(h.monitor, "message"); failure.reply("request_help", help); await stopped;
  assert.equal(h.sim.actions.filter(call => ["navigate", "complete"].includes(call.action)).length, 0);
  assert.equal(h.journal().find(event => event.kind === "measurement_result")!.data.success, false);
  assert.ok(!h.journal().some(event => event.kind === "evidence_reviewed" || event.kind === "stage_completed"));
});

test("actual crop review finishes before any selected-region navigation is submitted", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const stage = await plan(h.monitor, h.driver); stage.reply("select_regions", selectedTarget);
  const review = await h.driver.next();
  assert.deepEqual(names(review), ["accept_measurements", "change_view", "request_help", "select_regions"]);
  assert.ok(review.input.images.some(entry => entry.kind === "crop"));
  assert.ok(review.input.images.some(entry => entry.kind === "distance_layers"));
  await nextTurn();
  assert.equal(h.sim.actions.filter(call => call.action === "navigate").length, 0);
  review.reply("accept_measurements", acceptedTarget);
  const action = await h.driver.next();
  const executed = once(h.monitor, "message"); action.reply("follow_target", { reason: "Approach the reviewed elevator panel." }); await executed;
  const events = h.journal();
  assert.ok(events.find(event => event.kind === "measurement_result")!.seq < events.find(event => event.kind === "evidence_reviewed")!.seq);
  assert.ok(events.find(event => event.kind === "evidence_reviewed")!.seq < events.find(event => event.kind === "action_executed")!.seq);
  assert.equal(h.sim.actions.filter(call => call.action === "navigate").length, 1);
});

test("the actual one-region review schema rejects a duplicate-index layer batch", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const stage = await plan(h.monitor, h.driver); stage.reply("select_regions", selectedTarget);
  const review = await h.driver.next(); assert.equal(review.input.purpose, "review_measurements");
  assert.throws(() => review.reply("accept_measurements", { selections: [
    ...acceptedTarget.selections,
    { regionIndex: 0, layerId: "D2", semanticClaim: "A second band proposed for the same region." },
  ] }));
  assert.equal(h.sim.actions.filter(call => ["navigate", "complete"].includes(call.action)).length, 0);
  assert.ok(!h.journal().some(event => event.kind === "evidence_reviewed"));
  review.reply("accept_measurements", acceptedTarget);
  const decision = await h.driver.next();
  assert.equal(h.journal().filter(event => event.kind === "evidence_reviewed").length, 1);
  const stopped = once(h.monitor, "message"); decision.reply("request_help", help); await stopped;
});

test("dialogue takes fresh scene images on every question while HUMAN_PAUSED", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  await h.monitor.humanPause(); await h.monitor.startGoal("出门走到电梯门口");
  h.sim.transition({ state: "HUMAN_PAUSED", reason: "force_brake" });
  for (const question of ["现在在哪里？", "现在能看到电梯吗？"]) {
    const count = h.sim.captures.length;
    const answer = h.monitor.question(question);
    const proposal = await h.driver.next();
    assert.equal(proposal.input.purpose, "user_dialogue");
    assert.deepEqual(names(proposal), ["answer_user", "recall_memory"]);
    assert.equal(h.sim.captures.length, count + 1);
    assert.equal(proposal.input.images[0], h.sim.captures.at(-1)!.images[0]);
    assert.equal((proposal.input.context as { control: SimStatus }).control.state, "HUMAN_PAUSED");
    const fact = (proposal.input.context as { controlEventFact: { source: string; code: string; resumeAuthority: string } }).controlEventFact;
    assert.equal(fact.source, "control_service");
    assert.equal(fact.code, "force_brake");
    assert.equal(fact.resumeAuthority, "human");
    proposal.reply("answer_user", { text: `根据本次画面回答：${question}` });
    assert.equal(await answer, `根据本次画面回答：${question}`);
  }
  assert.equal(h.sim.current.state, "HUMAN_PAUSED");
  assert.equal(h.sim.actions.filter(call => ["navigate", "complete"].includes(call.action)).length, 0);
});

test("a question interrupts a stationary stage decision, then the same stage resumes with fresh RGB", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const interrupted = await plan(h.monitor, h.driver);
  const before = h.monitor.snapshot() as { task: { id: string; currentStage: number } };
  const originalFrame = h.sim.captures.at(-1)!;
  const captureCount = h.sim.captures.length;
  assert.equal(h.sim.current.state, "PAUSED");
  const answer = h.monitor.question("现在前方能看到什么？");
  const dialogue = await h.driver.next();
  assert.equal(dialogue.input.purpose, "user_dialogue");
  assert.equal(h.sim.captures.length, captureCount + 1);
  const dialogueFrame = h.sim.captures.at(-1)!;
  assert.notEqual(dialogueFrame.frame_id, originalFrame.frame_id);
  assert.equal(dialogue.input.images[0], dialogueFrame.images[0]);
  assert.ok(interrupted.input.signal.aborted);
  interrupted.reply("navigate", advance("Obsolete action proposed before the question."));
  await nextTurn();
  assert.equal(h.sim.actions.filter(call => call.action === "navigate").length, 0);
  dialogue.reply("answer_user", { text: "本次画面中能看到前方的门；当前仍然暂停。" });
  assert.equal(await answer, "本次画面中能看到前方的门；当前仍然暂停。");
  const resumed = await h.driver.next();
  assert.equal(resumed.input.purpose, "stage_decision");
  assert.equal(h.sim.captures.length, captureCount + 2);
  assert.notEqual(h.sim.captures.at(-1)!.frame_id, dialogueFrame.frame_id);
  assert.equal(resumed.input.images[0], h.sim.captures.at(-1)!.images[0]);
  const after = h.monitor.snapshot() as { task: { id: string; currentStage: number } };
  assert.equal(after.task.id, before.task.id);
  assert.equal(after.task.currentStage, before.task.currentStage);
  const executed = once(h.monitor, "message");
  resumed.reply("navigate", advance("Approach the elevator after the fresh observation.")); await executed;
  assert.deepEqual(h.sim.actions.filter(call => call.action === "navigate").map(call => call.instruction),
    ["Approach the elevator after the fresh observation."]);
  const events = h.journal();
  const answered = events.find(event => event.kind === "proposal" && event.data.purpose === "user_dialogue")!;
  const renewed = events.find(event => event.kind === "decision_context" && event.data.purpose === "stage_decision" && event.seq > answered.seq)!;
  assert.ok(renewed.seq < events.find(event => event.kind === "action_executed")!.seq);
});

test("a late proposal is discarded after human pause and resume even when the driver ignores abort", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const old = await plan(h.monitor, h.driver);
  await h.monitor.humanPause(); await h.monitor.humanResume();
  const fresh = await h.driver.next();
  assert.ok(old.input.signal.aborted);
  old.reply("navigate", advance("OLD instruction before the human pause."));
  await nextTurn();
  assert.equal(h.sim.actions.filter(call => call.action === "navigate").length, 0);
  const executed = once(h.monitor, "message"); fresh.reply("navigate", advance("Fresh instruction after human resume.")); await executed;
  assert.deepEqual(h.sim.actions.filter(call => call.action === "navigate").map(call => call.instruction), ["Fresh instruction after human resume."]);
  assert.ok(h.journal().some(event => event.kind === "stale_decision_discarded"));
});

test("goal replacement invalidates the old proposal before the new goal executes", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const old = await plan(h.monitor, h.driver);
  const oldGeneration = h.sim.current.generation;
  await h.monitor.startGoal("走到普通木门前");
  old.reply("navigate", advance("OLD elevator instruction."));
  const replacement = await h.driver.next(); assert.equal(replacement.input.purpose, "plan_task");
  assert.equal((replacement.input.context as { originalGoal: string }).originalGoal, "走到普通木门前");
  replacement.reply("plan_task", { stages: [{ ...approach, clause: "走到普通木门前", target: "wooden door",
    instruction: "Approach the wooden door." }] });
  const action = await h.driver.next();
  const executed = once(h.monitor, "message"); action.reply("navigate", advance("Approach the wooden door.")); await executed;
  const calls = h.sim.actions.filter(call => call.action === "navigate");
  assert.deepEqual(calls.map(call => call.instruction), ["Approach the wooden door."]);
  assert.ok(calls[0].generation > oldGeneration);
  assert.equal(h.sim.actions.filter(call => call.action === "invalidate").length, 2);
  const events = h.journal();
  assert.ok(events.find(event => event.kind === "task_replaced")!.seq < events.find(event => event.kind === "action_executed")!.seq);
});

test("SQLite restores the completed stage and waits for human resume before continuing", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  await finishDoorway(h.monitor, h.driver, h.sim); // Leave stage 2's proposal pending at shutdown.
  await h.monitor.close(); h.sim.expireLease();
  const beforeCaptures = h.sim.captures.length;
  const restored = await h.create(); await nextTurn();
  const snapshot = restored.monitor.snapshot() as { task: { currentStage: number; status: string; stages: { status: string }[] } };
  assert.equal(snapshot.task.currentStage, 1);
  assert.equal(snapshot.task.stages[0].status, "COMPLETED");
  assert.equal(snapshot.task.status, "WAITING_HUMAN");
  assert.equal(restored.driver.calls.length, 0);
  assert.equal(h.sim.captures.length, beforeCaptures);
  await restored.monitor.humanResume();
  const resumed = await restored.driver.next(); assert.equal(resumed.input.purpose, "stage_decision");
  assert.equal((resumed.input.context as { currentStage: number }).currentStage, 1);
  const events = h.journal();
  assert.ok(events.find(event => event.kind === "stage_completed")!.seq < events.find(event => event.kind === "task_restored")!.seq);
  assert.ok(events.find(event => event.kind === "task_restored")!.seq < events.find(event => event.kind === "human_resumed")!.seq);
});

test("an approach stage cannot complete from the previous LOCAL_DONE without its own ended navigation", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const approachStage = await finishDoorway(h.monitor, h.driver, h.sim);
  assert.equal(h.sim.current.state, "PAUSED");
  assert.equal(h.sim.current.reason, "stage_begin");
  assert.equal(h.sim.current.task_id, null);
  const previousTaskId = h.sim.current.task_id;
  approachStage.reply("select_regions", selectedTarget);
  const firstReview = await h.driver.next(); firstReview.reply("accept_measurements", acceptedTarget);
  const beforeNavigation = await h.driver.next();
  assert.ok(!names(beforeNavigation).includes("propose_stage_complete"));
  const context = beforeNavigation.input.context as { stage: { unmetConditions: string[] } };
  assert.ok(context.stage.unmetConditions.some(reason => reason.includes("THIS approach stage")));
  assert.equal(h.sim.actions.filter(call => call.action === "complete").length, 1);
  const executed = once(h.monitor, "message");
  beforeNavigation.reply("follow_target", { reason: "Approach the elevator door panel." }); await executed;
  assert.notEqual(h.sim.current.task_id, previousTaskId);
  h.sim.transition({ state: "ARRIVAL_CANDIDATE", reason: "lightnav_stop", pose_world: [4, 0, 0], effective_cmd: [0, 0, 0] });
  const arrival = await h.driver.next(); assert.equal(arrival.input.purpose, "verify_arrival");
  arrival.reply("select_regions", selectedTarget);
  const finalReview = await h.driver.next(); finalReview.reply("accept_measurements", acceptedTarget);
  const completion = await h.driver.next();
  assert.ok(names(completion).includes("propose_stage_complete"));
  const finished = once(h.monitor, "message");
  completion.reply("propose_stage_complete", { reason: "This approach navigation has ended; the current target ROI was reviewed and motion is zero." }); await finished;
  assert.equal(h.sim.actions.filter(call => call.action === "complete").length, 2);
  const events = h.journal();
  const results = events.filter(event => event.kind === "action_result");
  const completed = events.filter(event => event.kind === "stage_completed");
  assert.equal(results.length, 2); assert.equal(completed.length, 2);
  assert.ok(results[1].seq < completed[1].seq);
});

test("rewording the same failed advance at the same pose is rejected as a repeated strategy", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const stage = await plan(h.monitor, h.driver);
  const executed = once(h.monitor, "message"); stage.reply("navigate", advance("Move forward toward the elevator.")); await executed;
  h.sim.transition({ state: "PAUSED", reason: "mpc_blocked", pose_world: [0.02, 0, 0], effective_cmd: [0, 0, 0] });
  const recovery = await h.driver.next(); assert.equal(recovery.input.purpose, "error_recovery");
  recovery.reply("navigate", advance("Continue straight ahead to the lift doors."));
  const reconsider = await h.driver.next();
  assert.equal(h.sim.actions.filter(call => call.action === "navigate").length, 1);
  const rejected = h.journal().find(event => event.kind === "repeated_strategy_rejected");
  assert.ok(rejected);
  assert.equal((rejected.data.proposal as { instruction: string }).instruction, "Continue straight ahead to the lift doors.");
  const stopped = once(h.monitor, "message"); reconsider.reply("request_help", help); await stopped;
  assert.equal(h.sim.current.state, "PAUSED");
});

test("one decision buffer survives dialogue, then rotates only after a new movement is accepted", { timeout: 10_000 }, async t => {
  const h = await harness(t);
  const first = await plan(h.monitor, h.driver);
  let executed = once(h.monitor, "message");
  first.reply("navigate", advance("Approach the elevator.")); await executed;
  const firstBuffer = (h.monitor.snapshot() as { task: { memory: { id: string } } }).task.memory.id;
  h.sim.transition({ state: "PAUSED", reason: "mpc_blocked", pose_world: [0.2, 0, 0], effective_cmd: [0, 0, 0] });
  const recovery = await h.driver.next();
  const context = recovery.input.context as { buffer: { id: string; actualMotion: { path_length_m: number } } };
  assert.equal(context.buffer.id, firstBuffer);
  assert.equal(context.buffer.actualMotion.path_length_m, 0.2);
  assert.deepEqual(recovery.input.images.map(entry => entry.kind), ["rgb", "previous_decision"]);

  const answer = h.monitor.question("刚才实际走了多远？");
  const dialogue = await h.driver.next();
  assert.equal((dialogue.input.context as { buffer: { id: string } }).buffer.id, firstBuffer);
  recovery.reply("navigate", advance("Discard this interrupted proposal."));
  dialogue.reply("answer_user", { text: "实际移动了0.2米，随后控制器受阻。" }); await answer;
  const resumed = await h.driver.next();
  assert.equal((resumed.input.context as { buffer: { id: string } }).buffer.id, firstBuffer);
  executed = once(h.monitor, "message");
  resumed.reply("navigate", { maneuver: "turn_left", turnDegrees: 15, instruction: "Turn left.", expectedEffect: "Observe a different direction." });
  await executed;
  const nextBuffer = (h.monitor.snapshot() as { task: { memory: { id: string } } }).task.memory.id;
  assert.notEqual(nextBuffer, firstBuffer);
  const archives = h.journal().filter(event => event.kind === "decision_segment_archived");
  assert.equal(archives.length, 1);
  assert.equal(archives[0].data.id, firstBuffer);
  h.sim.transition({ state: "PAUSED", reason: "mpc_blocked", pose_world: [0.2, 0, 0.1], effective_cmd: [0, 0, 0] });
  const next = await h.driver.next();
  const buffer = (next.input.context as { buffer: { id: string } }).buffer;
  assert.equal(buffer.id, nextBuffer);
  assert.ok(!JSON.stringify(buffer).includes(firstBuffer), "older buffers must not be nested in the new buffer");
  assert.equal(next.input.images.filter(entry => entry.kind === "previous_decision").length, 1);
});
