import { readFileSync } from "node:fs";
import type { SimStatus, SceneFrame, RegionFrame } from "./sim-client.js";

export interface Layer {
  layer_id: string; median_m: number; p10_m: number; p90_m: number;
  count: number; sample_world_point_m: number[]; sample_base_point_m: number[];
  support_world_bounds_m: number[][];
  surface: Surface | null;
}
export interface Surface {
  centroid_world_m: number[]; normal_world: number[]; rms_m: number;
  point_count: number; in_plane_std_m: number[];
}
export interface TargetAnchor {
  id: string; description: string; relation: "cross" | "wait_outside";
  observationId: string; bboxPx: number[]; pointWorldM: number[];
  surface?: Surface; semanticClaim: string;
  referencePointWorldM: number[];
}
export interface Region {
  role: "target" | "boundary" | "left_boundary" | "right_boundary";
  bbox_px: number[]; description: string;
  semanticInstanceId?: string;
  observationId?: string; poseWorld?: number[]; cropImagePath?: string;
  result?: Omit<RegionFrame, "images">; error?: string;
  selectedLayer?: Layer; semanticClaim?: string;
}
export interface BoundaryEvidence {
  id: string; region: Region; measurementCount: number; currentFrameMaskIds: string[];
}
export interface Portal {
  origin: number[]; normal: number[]; points: number[][];
  observationId: string; evidence: Region[];
}
export interface Observation {
  id: string; frame: SceneFrame; control: SimStatus;
  imagePaths: string[]; regions: Region[]; reviewed: boolean;
}
export const navigationContract = JSON.parse(readFileSync(new URL("../navigation-contract.json", import.meta.url), "utf8")) as {
  version: string; source: string; doorway_exit_margin_m: number;
  local_goal: { position_tolerance_m: number; heading_tolerance_deg: number };
  waiting_zone: { front_clearance_m: number[]; heading_tolerance_deg: number; lateral_tolerance_m: number; distance_definition: string };
  target_plane_max_rms_m: number; target_plane_max_vertical_tilt_deg: number;
};
const footprint = JSON.parse(readFileSync(new URL("../../control/footprint.json", import.meta.url), "utf8")) as {
  centers_base_m: number[][]; radii_m: number[];
};
export const bodyRadiusM = Math.max(...footprint.centers_base_m.map((center, i) => Math.hypot(...center)+footprint.radii_m[i]));
export const minimumOpeningSpanM = 0.7;
export function boundaryRelation(a: Layer, b: Layer) {
  const representativeSeparationM = Math.hypot(...a.sample_world_point_m.slice(0, 2).map((v, i) => v-b.sample_world_point_m[i]));
  const supportGapM = Math.hypot(...[0, 1].map(i => Math.max(0,
    a.support_world_bounds_m[0][i]-b.support_world_bounds_m[1][i],
    b.support_world_bounds_m[0][i]-a.support_world_bounds_m[1][i])));
  return { representativeSeparationM, supportGapM, minimumOpeningSpanM,
    canBoundUsableOpening: representativeSeparationM >= minimumOpeningSpanM,
    meaning: representativeSeparationM < minimumOpeningSpanM
      ? "These measurements cannot represent opposite boundaries of a usable opening; they may observe the same physical support."
      : "Spatially separated boundary evidence; RGB must still confirm the same opening between them." };
}
export function mergeBoundaryEvidence(previous: BoundaryEvidence[], regions: Region[]): BoundaryEvidence[] {
  const result = previous.map(jamb => ({ ...jamb, currentFrameMaskIds: [...jamb.currentFrameMaskIds] }));
  for (const region of regions) {
    const candidates = result.map(jamb => ({ jamb, relation: boundaryRelation(jamb.region.selectedLayer!, region.selectedLayer!) }))
      .filter(item => !item.relation.canBoundUsableOpening)
      .sort((a, b) => a.relation.representativeSeparationM-b.relation.representativeSeparationM);
    const matched = candidates[0]?.jamb;
    if (matched) {
      // One navigation boundary hypothesis, not a claim that geometry proves object identity.
      const sameFrame = matched.region.observationId === region.observationId;
      matched.currentFrameMaskIds = [...new Set([...(sameFrame ? matched.currentFrameMaskIds : []),
        ...(region.semanticInstanceId ? [region.semanticInstanceId] : [])])];
      matched.region = region; matched.measurementCount++;
    } else {
      result.push({ id: `B${result.length+1}`, region, measurementCount: 1,
        currentFrameMaskIds: region.semanticInstanceId ? [region.semanticInstanceId] : [] });
    }
  }
  return result;
}
export function makePortal(observation: Observation, jambs: BoundaryEvidence[]): Portal {
  const regions = jambs.map(jamb => jamb.region);
  const a = regions[0]?.selectedLayer?.sample_world_point_m;
  const b = regions[1]?.selectedLayer?.sample_world_point_m;
  if (!a || !b) throw new Error("Passage needs two successfully reviewed boundary measurements");
  const dx = b[0] - a[0], dy = b[1] - a[1], width = Math.hypot(dx, dy);
  if (jambs.length !== 2 || width < minimumOpeningSpanM) throw new Error("Doorway needs two spatially separated boundary hypotheses");
  const origin = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const normal = [-dy / width, dx / width];
  const start = observation.frame.pose_world;
  if ((origin[0] - start[0]) * normal[0] + (origin[1] - start[1]) * normal[1] < 0) {
    normal[0] *= -1; normal[1] *= -1;
  }
  const evidence = regions.map(region => ({ ...region, role:
    ((region.selectedLayer!.sample_world_point_m[0]-origin[0])*normal[1]
      -(region.selectedLayer!.sample_world_point_m[1]-origin[1])*normal[0]) < 0 ? "left_boundary" as const : "right_boundary" as const }));
  return { origin, normal, points: [a, b], observationId: observation.id, evidence };
}
export function usableVerticalSurface(layer: Layer): boolean {
  const plane = layer.surface;
  return Boolean(plane && plane.rms_m <= navigationContract.target_plane_max_rms_m
    && plane.in_plane_std_m[0] >= 0.01
    && Math.abs(plane.normal_world[2]) <= Math.sin(navigationContract.target_plane_max_vertical_tilt_deg * Math.PI / 180));
}

export function targetGeometry(anchor: TargetAnchor, pose: number[], baseDistanceM?: number): {
  frontClearanceM: number; headingErrorDeg: number; lateralOffsetM: number; bearingDeg: number;
  standingPointWorldM: number[]; standingDistanceM: number; standingBearingDeg: number;
} {
  if (!anchor.surface) throw new Error("Target has no measured surface plane");
  const [px, py] = anchor.surface.centroid_world_m;
  const norm = Math.hypot(...anchor.surface.normal_world.slice(0, 2));
  const nx = anchor.surface.normal_world[0] / norm, ny = anchor.surface.normal_world[1] / norm;
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  const frontClearanceM = Math.min(...footprint.centers_base_m.map(([x, y], i) =>
    (pose[0] + c*x - s*y - px)*nx + (pose[1] + s*x + c*y - py)*ny - footprint.radii_m[i]));
  const angle = (v: number) => Math.atan2(Math.sin(v), Math.cos(v)) * 180 / Math.PI;
  const frontExtent = Math.max(...footprint.centers_base_m.map(([x], i) => x+footprint.radii_m[i]));
  const desiredClearance = (navigationContract.waiting_zone.front_clearance_m[0]+navigationContract.waiting_zone.front_clearance_m[1])/2;
  const reference = anchor.referencePointWorldM;
  const normalOffset = (reference[0]-px)*nx+(reference[1]-py)*ny;
  const referenceOnPlane = [reference[0]-normalOffset*nx, reference[1]-normalOffset*ny];
  const planeOffset = baseDistanceM === undefined ? frontExtent+desiredClearance : baseDistanceM;
  const standingPointWorldM = [referenceOnPlane[0]+nx*planeOffset, referenceOnPlane[1]+ny*planeOffset];
  const sx = standingPointWorldM[0]-pose[0], sy = standingPointWorldM[1]-pose[1];
  return { frontClearanceM, headingErrorDeg: angle(Math.atan2(-ny, -nx) - pose[2]),
    lateralOffsetM: (pose[0] - referenceOnPlane[0])*(-ny) + (pose[1] - referenceOnPlane[1])*nx,
    bearingDeg: angle(Math.atan2(anchor.pointWorldM[1]-pose[1], anchor.pointWorldM[0]-pose[0])-pose[2]),
    standingPointWorldM, standingDistanceM: Math.hypot(sx, sy), standingBearingDeg: angle(Math.atan2(sy, sx)-pose[2]) };
}

export function waitingZoneConditions(anchor: TargetAnchor, pose: number[]): string[] {
  const g = targetGeometry(anchor, pose), zone = navigationContract.waiting_zone;
  const unmet: string[] = [];
  if (g.frontClearanceM < zone.front_clearance_m[0] || g.frontClearanceM > zone.front_clearance_m[1]) {
    unmet.push(`Prototype waiting zone requires footprint-to-target-plane clearance ${zone.front_clearance_m.join("–")} m; measured ${g.frontClearanceM.toFixed(3)} m`);
  }
  if (Math.abs(g.headingErrorDeg) > zone.heading_tolerance_deg) unmet.push(`Face target plane within ±${zone.heading_tolerance_deg}°; heading error ${g.headingErrorDeg.toFixed(1)}°`);
  if (Math.abs(g.lateralOffsetM) > zone.lateral_tolerance_m) unmet.push(`Align with the observed target patch within ${zone.lateral_tolerance_m} m; lateral offset ${g.lateralOffsetM.toFixed(3)} m`);
  return unmet;
}
export function portalClearance(portal: Portal, pose: number[]): number {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  return Math.min(...footprint.centers_base_m.map(([x, y], i) =>
    (pose[0] + c * x - s * y - portal.origin[0]) * portal.normal[0]
    + (pose[1] + s * x + c * y - portal.origin[1]) * portal.normal[1] - footprint.radii_m[i]));
}
export function isStopped(control: SimStatus): boolean {
  return control.state !== "NAVIGATING" && control.state !== "MANUAL"
    && control.effective_cmd.every(v => Math.abs(v) < 1e-6)
    && Math.hypot(...control.actual_velocity_body.slice(0, 2)) < 0.05;
}
export function motionDelta(a: number[], b: number[]): { distanceM: number; yawRad: number } {
  return { distanceM: Math.hypot(b[0] - a[0], b[1] - a[1]),
    yawRad: Math.abs(Math.atan2(Math.sin(b[2] - a[2]), Math.cos(b[2] - a[2]))) };
}

export function controlEventFact(control: SimStatus): unknown {
  if (control.state !== "HUMAN_PAUSED" || control.reason !== "force_brake") return undefined;
  return {
    source: "control_service", code: "force_brake", resumeAuthority: "human",
    explanation: "旧 policy 的后向外力估计满足后拉制动条件，控制服务已锁存人工暂停。外力撤去后暂停仍保留，只有人工恢复才能解除。该事件没有报告障碍检测或碰撞；不能将停下归因于前方障碍。",
  };
}
