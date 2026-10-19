import type { ImageContent } from "@earendil-works/pi-ai";

export interface RobotState {
  stamp_s: number; epoch: string; frame_id: "odom"; child_frame_id: "base_link";
  odom_from_base: number[][]; pose_odom: number[]; twist_base: number[]; planar_twist: number[];
  tracking: string; source: string; quality: Record<string, unknown>;
}
export interface SpatialEvidence {
  epoch: string; stamp_s: number; frame_id: "odom"; map_stamp_s: number;
  point_states: ("free" | "occupied" | "unknown")[];
  footprint_states: ("free" | "occupied" | "unknown")[];
}

export interface SimStatus {
  event_id: number;
  generation: number;
  instance_id: string;
  task_id: string | null;
  state: string;
  instruction: string;
  reason: string;
  robot_state: RobotState;
  effective_cmd: number[];
  [key: string]: unknown;
}

export interface ObservationImage {
  kind: string;
  label: string;
  image: ImageContent;
}

export interface MotionSegment {
  control_task_id: string;
  start_stamp_s: number;
  end_stamp_s: number;
  source: string;
  epoch: string;
  estimated: {
    samples: { stamp_s: number; pose_odom: number[] }[];
    sample_count: number;
    path_length_m: number;
    net_displacement_m: number;
    yaw_change_rad: number;
  };
  plans: {
    sequence: number; capture_time_s: number; capture_pose_odom: number[];
    received_stamp_s: number; waypoints_odom: number[][];
    visible: boolean | null; stop: boolean; pointing: Record<string, unknown> | null;
  }[];
  plan_count: number;
  events: { stamp_s: number; state: string; reason: string }[];
  control_execution: { control_ticks: number; nonzero_ticks: number; first_stamp_s: number | null;
    last_stamp_s: number | null; last_effective_cmd: number[]; command_min: number[]; command_max: number[]; meaning: string;
    [key: string]: unknown };
  log_paths: { telemetry: string; lightnav: string };
}

export interface SceneFrame {
  images: ObservationImage[];
  frame_id: string;
  robot_state: RobotState;
  depth: Record<string, unknown>;
  semantic_depth: {
    frame_id: string;
    instances: { id: string; label: string; confidence: number; bbox_px: number[]; [key: string]: unknown }[];
    mask_relations: { instance_ids: string[]; intersection_px: number; iou: number;
      overlap_fraction_of_smaller_mask: number; representative_xy_distance_m: number | null;
      support_xy_gap_m: number | null }[];
    [key: string]: unknown;
  };
  lightnav_hint?: { navigation_scope: string; source: string; point_1000: number[] | null; in_view: boolean; [key: string]: unknown } | null;
  camera: {
    width: number;
    height: number;
    hfov_deg: number;
    fovy_deg: number;
    position_base_m: number[];
    pitch_down_deg: number;
  };
}

export interface RegionFrame {
  images: ObservationImage[];
  [key: string]: unknown;
}

export class SimClient {
  lastScene?: SceneFrame;

  constructor(readonly baseUrl = process.env.SIM_URL ?? "http://127.0.0.1:8766") {}

  private async request(path: string, body?: object, signal?: AbortSignal): Promise<Response> {
    const timeout = AbortSignal.timeout(15_000);
    const response = await fetch(new URL(path, this.baseUrl), {
      method: body === undefined ? "GET" : "POST",
      headers: body === undefined ? undefined : { "Content-Type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: signal ? AbortSignal.any([signal, timeout]) : timeout,
    });
    if (!response.ok) {
      throw new Error(`${path}: HTTP ${response.status}: ${await response.text()}`);
    }
    return response;
  }

  async status(signal?: AbortSignal): Promise<SimStatus> {
    return (await this.request("/status", undefined, signal)).json();
  }

  async motion(signal?: AbortSignal): Promise<MotionSegment | null> {
    return (await this.request("/motion", undefined, signal)).json();
  }

  async spatialEvidence(epoch: string, stampS: number, points: number[][], poses: number[][],
    signal?: AbortSignal): Promise<SpatialEvidence> {
    const evidence: SpatialEvidence = await (await this.request("/spatial_evidence", { epoch, stamp_s: stampS,
      points_odom: points, footprint_poses_odom: poses }, signal)).json();
    if (evidence.point_states.length !== points.length || evidence.footprint_states.length !== poses.length) {
      throw new Error("Spatial evidence does not cover the requested geometry");
    }
    return evidence;
  }

  async navigate(instruction: string, signal?: AbortSignal): Promise<unknown> {
    return (await this.request("/navigate", { instruction }, signal)).json();
  }

  async pause(reason = "agent", signal?: AbortSignal): Promise<unknown> {
    return (await this.request("/pause", { reason }, signal)).json();
  }

  async claim(monitorId: string): Promise<SimStatus> {
    return (await this.request("/monitor/claim", { monitor_id: monitorId })).json();
  }

  async heartbeat(monitorId: string): Promise<SimStatus> {
    return (await this.request("/monitor/heartbeat", { monitor_id: monitorId })).json();
  }

  async action(monitorId: string, generation: number, action: "navigate" | "pause" | "invalidate" | "complete",
    instruction?: string, reason?: string, signal?: AbortSignal, navigationScope?: string,
    movement?: { maneuver: string; turnDegrees?: number; advanceMeters?: number; semanticInstruction?: string;
      goalPointOdom?: number[]; goalHeadingRad?: number; positionToleranceM?: number; headingToleranceDeg?: number }): Promise<SimStatus> {
    return (await this.request("/monitor/action", { monitor_id: monitorId,
      expected_generation: generation, action, instruction, reason, navigation_scope: navigationScope,
      maneuver: movement?.maneuver, turn_degrees: movement?.turnDegrees, advance_m: movement?.advanceMeters,
      semantic_instruction: movement?.semanticInstruction,
      goal_point_odom: movement?.goalPointOdom, goal_heading_rad: movement?.goalHeadingRad,
      position_tolerance_m: movement?.positionToleranceM, heading_tolerance_deg: movement?.headingToleranceDeg }, signal)).json();
  }

  // Only the operator CLI calls this method. It is never registered as an AgentTool.
  async resume(): Promise<SimStatus> {
    const status = await this.status();
    return (await this.request("/resume", { source: "human", expected_generation: status.generation })).json();
  }

  async scene(signal?: AbortSignal): Promise<ImageContent> {
    return (await this.captureScene(signal)).images[0].image;
  }

  async captureScene(signal?: AbortSignal): Promise<SceneFrame> {
    const frame: SceneFrame = await (await this.request("/scene", undefined, signal)).json();
    this.lastScene = frame;
    return frame;
  }

  async measureRegion(frameId: string, bboxPx: number[], signal?: AbortSignal): Promise<RegionFrame> {
    return (await this.request("/measure_region", { frame_id: frameId, bbox_px: bboxPx }, signal)).json();
  }

  async measureSemanticInstance(frameId: string, instanceId: string, signal?: AbortSignal): Promise<RegionFrame> {
    return (await this.request("/measure_semantic_instance", { frame_id: frameId, instance_id: instanceId }, signal)).json();
  }

  async *events(signal: AbortSignal): AsyncGenerator<SimStatus> {
    const response = await fetch(new URL("/events", this.baseUrl), {
      headers: { Accept: "text/event-stream" }, signal,
    });
    if (!response.ok) throw new Error(`/events: HTTP ${response.status}: ${await response.text()}`);
    if (!response.body) throw new Error("/events has no response body");
    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = "";
    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) throw new Error("Simulator event stream closed");
        buffer += decoder.decode(value, { stream: true });
        let end: number;
        while ((end = buffer.indexOf("\n\n")) >= 0) {
          const frame = buffer.slice(0, end);
          buffer = buffer.slice(end + 2);
          const data = frame.split("\n").filter((line) => line.startsWith("data:"))
            .map((line) => line.slice(5).trimStart()).join("\n");
          if (data) yield JSON.parse(data);
        }
      }
    } finally { reader.releaseLock(); }
  }
}
