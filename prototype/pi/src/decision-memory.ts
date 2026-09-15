import type { MotionSegment } from "./sim-client.js";

/** One accepted movement decision. Never embed a previous request or buffer. */
export interface DecisionMemory {
  id: string;
  stageIndex: number;
  targetId: string;
  observation: {
    id: string; imagePath: string; simTimeS: number; poseWorld: number[];
  };
  decision: {
    maneuver: string; instruction: string; expectedEffect: string;
    controlTaskId: string; acceptedSimTimeS: number;
    turnDegrees?: number; advanceMeters?: number;
    semanticInstruction?: string; referenceSource?: string;
    goalPointWorld?: number[]; goalHeadingRad?: number;
  };
  motion: MotionSegment | null;
}

export function memoryContext(memory: DecisionMemory | undefined): unknown {
  if (!memory) return null;
  const { imagePath, ...observation } = memory.observation;
  const motion = memory.motion;
  const rounded = (values: number[]) => values.map(value => Math.round(value * 1000) / 1000);
  return {
    id: memory.id, stageIndex: memory.stageIndex, targetId: memory.targetId,
    observation, decision: memory.decision,
    interval: motion && { startSimTimeS: motion.start_sim_time_s, endSimTimeS: motion.end_sim_time_s },
    actualMotion: motion && { ...motion.actual, samples: motion.actual.samples.map(sample => ({
      sim_time_s: Math.round(sample.sim_time_s * 100) / 100, pose_world: rounded(sample.pose_world) })) },
    lightnavPlans: motion?.plans.map(plan => ({ ...plan, capture_pose_world: rounded(plan.capture_pose_world),
      waypoints_world: plan.waypoints_world.map(rounded) })),
    planCount: motion?.plan_count,
    controlEvents: motion?.events,
    interpretation: "Actual motion is measured simulator odometry. LightNav predicts the semantic stage goal continuously. Explicit metric maneuvers use their own MPC reference, so these LightNav chunks are advice, not executed motion. Chunks overlap and are not traveled distance.",
  };
}
