import { readFileSync } from "node:fs";
import type { SimStatus, SceneFrame, RegionFrame, RobotState, SpatialEvidence } from "./sim-client.js";

export interface Layer {
  layer_id: string; median_m: number; p10_m: number; p90_m: number;
  count: number; sample_odom_point_m: number[]; sample_base_point_m: number[];
  support_odom_bounds_m: number[][];
  surface: Surface | null;
}
export interface Surface {
  centroid_odom_m: number[]; normal_odom: number[]; rms_m: number;
  point_count: number; in_plane_std_m: number[];
}
export interface TargetAnchor {
  id: string; description: string; relation: "cross" | "wait_outside";
  observationId: string; bboxPx: number[]; pointOdomM: number[];
  surface?: Surface; semanticClaim: string;
  referencePointOdomM: number[]; epoch: string; stampS: number;
}
export interface Region {
  role: "target" | "boundary" | "left_boundary" | "right_boundary";
  bbox_px: number[]; description: string;
  semanticInstanceId?: string;
  observationId?: string; robotState?: RobotState; poseOdom?: number[]; cropImagePath?: string;
  result?: Omit<RegionFrame, "images">; error?: string;
  selectedLayer?: Layer; semanticClaim?: string;
}
export interface BoundaryEvidence {
  id: string; region: Region; measurementCount: number; currentFrameMaskIds: string[];
}
export interface Portal {
  origin: number[]; normal: number[]; points: number[][];
  observationId: string; evidence: Region[]; epoch: string; stampS: number;
  entryOrigin: number[]; lateral: number[]; widthM: number; exitPointOdomM: number[];
  corridorPoses: number[][]; expansionPoses: number[][];
  spatialEvidence?: SpatialEvidence; entered: boolean; crossed: boolean;
}
export interface Observation {
  id: string; frame: SceneFrame; control: SimStatus;
  imagePaths: string[]; regions: Region[]; reviewed: boolean; spatialEvidence?: SpatialEvidence;
  identityReview?: { identities: string[]; sameOpening?: boolean };
  measurementIssues?: string[];
}
export const navigationContract = JSON.parse(readFileSync(new URL("../navigation-contract.json", import.meta.url), "utf8")) as {
  version: string; source: string; doorway_exit_margin_m: number;
  local_goal: { position_tolerance_m: number; heading_tolerance_deg: number };
  local_autonomy: { target_evidence_range_m: number; precision_handoff_range_m: number;
    new_evidence_translation_m: number; new_evidence_yaw_rad: number };
  waiting_zone: { front_clearance_m: number[]; heading_tolerance_deg: number; lateral_tolerance_m: number; distance_definition: string };
  evidence_max_age_s: number; stopped_duration_s: number;
  target_plane_max_rms_m: number; target_plane_max_vertical_tilt_deg: number;
};
const footprint = JSON.parse(readFileSync(new URL("../../control/footprint.json", import.meta.url), "utf8")) as {
  centers_base_m: number[][]; radii_m: number[];
};
export const bodyRadiusM = Math.max(...footprint.centers_base_m.map((center, i) => Math.hypot(...center)+footprint.radii_m[i]));
export const minimumOpeningSpanM = 0.7;
export function boundaryRelation(a: Layer, b: Layer) {
  const representativeSeparationM = Math.hypot(...a.sample_odom_point_m.slice(0, 2).map((v, i) => v-b.sample_odom_point_m[i]));
  const supportGapM = Math.hypot(...[0, 1].map(i => Math.max(0,
    a.support_odom_bounds_m[0][i]-b.support_odom_bounds_m[1][i],
    b.support_odom_bounds_m[0][i]-a.support_odom_bounds_m[1][i])));
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
function projectionBounds(bounds: number[][], axis: number[]): number[] {
  const values = [0, 1].flatMap(i => [0, 1].map(j => bounds[i][0]*axis[0]+bounds[j][1]*axis[1]));
  return [Math.min(...values), Math.max(...values)];
}
export class PassageObservationNeeded extends Error {}
export function makePortal(observation: Observation, jambs: BoundaryEvidence[]): Portal {
  if (jambs.length !== 2) throw new PassageObservationNeeded("Observe both sides of the passage");
  const regions = jambs.map(jamb => jamb.region);
  const [a, b] = regions.map(region => region.selectedLayer!);
  const state = observation.frame.robot_state;
  if (regions.some(region => region.robotState!.epoch !== state.epoch)) throw new PassageObservationNeeded("Passage evidence belongs to another odometry epoch");
  const [pa, pb] = [a, b].map(layer => layer.surface!.centroid_odom_m);
  const [na, nb] = [a, b].map(layer => {
    const n = layer.surface!.normal_odom; const length = Math.hypot(n[0], n[1]);
    return [n[0]/length, n[1]/length];
  });
  // Supporting side planes must face each other, not merely have separated sample points.
  if (na[0]*nb[0]+na[1]*nb[1] > -Math.cos(20*Math.PI/180)) {
    throw new PassageObservationNeeded("Need opposing passage side surfaces; these patches do not face each other. Observe the missing side.");
  }
  const lateral = na;
  const width = (pb[0]-pa[0])*lateral[0]+(pb[1]-pa[1])*lateral[1];
  if (width < minimumOpeningSpanM) throw new PassageObservationNeeded("The facing surfaces do not enclose a usable passage");
  let normal = [-lateral[1], lateral[0]];
  const center = [(pa[0]+pb[0])/2, (pa[1]+pb[1])/2];
  if ((center[0]-state.pose_odom[0])*normal[0]+(center[1]-state.pose_odom[1])*normal[1] < 0) normal = normal.map(v => -v);
  const [ra, rb] = [a, b].map(layer => projectionBounds(layer.support_odom_bounds_m, normal));
  if (Math.min(ra[1], rb[1]) <= Math.max(ra[0], rb[0])) throw new PassageObservationNeeded("Side supports do not overlap along the passage");
  const entry = Math.min(ra[0], rb[0]), exit = Math.max(ra[1], rb[1]);
  const lateralCenter = center[0]*lateral[0]+center[1]*lateral[1];
  const xy = (forward: number, sideways = lateralCenter) => normal.map((v, i) => v*forward+lateral[i]*sideways);
  const origin = xy(exit), entryOrigin = xy(entry);
  const heading = Math.atan2(normal[1], normal[0]);
  const targetForward = exit+bodyRadiusM+navigationContract.doorway_exit_margin_m+navigationContract.local_goal.position_tolerance_m;
  const corridorPoses: number[][] = [];
  for (let forward = entry; forward <= targetForward; forward += .04) corridorPoses.push([...xy(forward), heading]);
  corridorPoses.push([...xy(targetForward), heading]);
  // Entire body-volume queries straddle each side plane beyond the furthest observed
  // support. A selected depth layer's height range cannot certify an entire far edge.
  const expansionPoses = [-1, 1].map(side =>
    [...xy(exit+bodyRadiusM+.08, lateralCenter+side*width/2), heading]);
  return { origin, normal, points: [pa, pb], entryOrigin, lateral, widthM: width,
    exitPointOdomM: xy(targetForward), corridorPoses, expansionPoses, entered: false, crossed: false,
    observationId: observation.id, evidence: regions, epoch: state.epoch, stampS: state.stamp_s };
}
export function currentTargetGeometry(layer: Layer, state: RobotState, distanceM?: number) {
  const transform = state.odom_from_base;
  const plane = layer.surface!;
  const relative = plane.centroid_odom_m.map((v, i) => v-transform[i][3]);
  const point = [0, 1, 2].map(i => relative.reduce((sum, v, j) => sum+transform[j][i]*v, 0));
  const normal = [0, 1, 2].map(i => plane.normal_odom.reduce((sum, v, j) => sum+transform[j][i]*v, 0));
  const anchor: TargetAnchor = { id: "current", description: "current reviewed target", relation: "wait_outside",
    observationId: "current", bboxPx: [], pointOdomM: point, referencePointOdomM: point,
    surface: { ...plane, centroid_odom_m: point, normal_odom: normal }, semanticClaim: "current reviewed pixels",
    epoch: state.epoch, stampS: state.stamp_s };
  return { geometry: targetGeometry(anchor, [0, 0, 0], distanceM), unmet: waitingZoneConditions(anchor, [0, 0, 0]) };
}
export function usableVerticalSurface(layer: Layer): boolean {
  const plane = layer.surface;
  return Boolean(plane && plane.rms_m <= navigationContract.target_plane_max_rms_m
    && plane.in_plane_std_m[0] >= 0.01
    && Math.abs(plane.normal_odom[2]) <= Math.sin(navigationContract.target_plane_max_vertical_tilt_deg * Math.PI / 180));
}

export function targetGeometry(anchor: TargetAnchor, pose: number[], baseDistanceM?: number): {
  frontClearanceM: number; headingErrorDeg: number; lateralOffsetM: number; bearingDeg: number;
  standingPointOdomM: number[]; standingDistanceM: number; standingBearingDeg: number;
} {
  if (!anchor.surface) throw new Error("Target has no measured surface plane");
  const [px, py] = anchor.surface.centroid_odom_m;
  const norm = Math.hypot(...anchor.surface.normal_odom.slice(0, 2));
  const nx = anchor.surface.normal_odom[0] / norm, ny = anchor.surface.normal_odom[1] / norm;
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  const frontClearanceM = Math.min(...footprint.centers_base_m.map(([x, y], i) =>
    (pose[0] + c*x - s*y - px)*nx + (pose[1] + s*x + c*y - py)*ny - footprint.radii_m[i]));
  const angle = (v: number) => Math.atan2(Math.sin(v), Math.cos(v)) * 180 / Math.PI;
  const frontExtent = Math.max(...footprint.centers_base_m.map(([x], i) => x+footprint.radii_m[i]));
  const desiredClearance = (navigationContract.waiting_zone.front_clearance_m[0]+navigationContract.waiting_zone.front_clearance_m[1])/2;
  const reference = anchor.referencePointOdomM;
  const normalOffset = (reference[0]-px)*nx+(reference[1]-py)*ny;
  const referenceOnPlane = [reference[0]-normalOffset*nx, reference[1]-normalOffset*ny];
  const planeOffset = baseDistanceM === undefined ? frontExtent+desiredClearance : baseDistanceM;
  const standingPointOdomM = [referenceOnPlane[0]+nx*planeOffset, referenceOnPlane[1]+ny*planeOffset];
  const sx = standingPointOdomM[0]-pose[0], sy = standingPointOdomM[1]-pose[1];
  return { frontClearanceM, headingErrorDeg: angle(Math.atan2(-ny, -nx) - pose[2]),
    lateralOffsetM: (pose[0] - referenceOnPlane[0])*(-ny) + (pose[1] - referenceOnPlane[1])*nx,
    bearingDeg: angle(Math.atan2(anchor.pointOdomM[1]-pose[1], anchor.pointOdomM[0]-pose[0])-pose[2]),
    standingPointOdomM, standingDistanceM: Math.hypot(sx, sy), standingBearingDeg: angle(Math.atan2(sy, sx)-pose[2]) };
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
export function insidePortalSides(portal: Portal, pose: number[]): boolean {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]);
  return footprint.centers_base_m.every(([x, y], i) => {
    const lateral = (pose[0]+c*x-s*y-portal.origin[0])*portal.lateral[0]
      +(pose[1]+s*x+c*y-portal.origin[1])*portal.lateral[1];
    return Math.abs(lateral)+footprint.radii_m[i] < portal.widthM/2;
  });
}
export function isStopped(control: SimStatus): boolean {
  return control.state !== "NAVIGATING" && control.state !== "MANUAL"
    && control.effective_cmd.every(v => Math.abs(v) < 1e-6)
    && control.robot_state.tracking === "tracking"
    && control.robot_state.quality.velocity_estimated === true
    && Math.hypot(...control.robot_state.planar_twist.slice(0, 2)) < 0.05
    && Math.abs(control.robot_state.planar_twist[2]) < 0.05;
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
