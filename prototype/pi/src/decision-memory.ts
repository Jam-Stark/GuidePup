import type { MotionSegment } from "./sim-client.js";

/** One accepted movement decision. Never embed a previous request or buffer. */
export interface DecisionMemory {
  id: string;
  stageIndex: number;
  targetId: string;
  observation: {
    id: string; imagePath: string; stampS: number; epoch: string; poseOdom: number[];
  };
  decision: {
    maneuver: string; instruction: string; expectedEffect: string;
    controlTaskId: string; acceptedStampS: number;
    turnDegrees?: number; advanceMeters?: number;
    semanticInstruction?: string; referenceSource?: string;
    goalPointOdom?: number[]; goalHeadingRad?: number;
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
    interval: motion && { startStampS: motion.start_stamp_s, endStampS: motion.end_stamp_s },
    estimatedMotion: motion && { ...motion.estimated, samples: motion.estimated.samples.map(sample => ({
      stamp_s: Math.round(sample.stamp_s * 100) / 100, pose_odom: rounded(sample.pose_odom) })) },
    lightnavPlans: motion?.plans.map(plan => ({ ...plan, capture_pose_odom: rounded(plan.capture_pose_odom),
      waypoints_odom: plan.waypoints_odom.map(rounded) })),
    planCount: motion?.plan_count,
    controlEvents: motion?.events,
    appliedControl: motion?.control_execution,
    interpretation: "Motion is estimated from depth/IMU/proprioception. referenceSource=lightnav means the local controller consumes model trajectories; metric sources mean geometry control or an explicit recovery/observation action. appliedControl summarizes commands actually sent, not physical displacement. Plans are predictions, may overlap, and are not traveled distance. Actual source/adoption is recorded in local controller logs.",
  };
}
