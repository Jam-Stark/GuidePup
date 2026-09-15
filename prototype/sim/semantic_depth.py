"""YOLOE text-prompt masks measured against their own captured RGB-D frame.

No scene geometry IDs, simulator target poses, tracking, or navigation decisions.
"""
from __future__ import annotations

from pathlib import Path
from itertools import combinations

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from depth import DepthFrame

DEFAULT_CLASSES = ('door', 'door frame', 'elevator door', 'wall')


class YOLOESemanticDepth:
    """One model owner; callers serialize inference and cache by capture frame ID."""

    def __init__(self, checkpoint: str | Path, device: str, classes=DEFAULT_CLASSES,
                 confidence: float = 0.03, image_size: int = 640):
        from ultralytics import YOLOE

        self.checkpoint = str(checkpoint)
        self.device = device
        self.classes = list(classes)
        self.confidence = confidence
        self.image_size = image_size
        self.model = YOLOE(self.checkpoint).to(device)
        self.model.set_classes(self.classes)

    def predict(self, frame: DepthFrame) -> SemanticDepthFrame:
        # PIL explicitly denotes RGB; Ultralytics ndarray sources denote BGR.
        result = self.model.predict(Image.fromarray(frame.rgb), device=self.device,
                                    conf=self.confidence, imgsz=self.image_size,
                                    retina_masks=True, nms=True, verbose=False)[0]
        boxes = result.boxes
        detections = []
        if len(boxes):
            masks = result.masks.data.cpu().numpy().astype(bool)
            if masks.shape != (len(boxes), frame.height, frame.width):
                raise ValueError('YOLOE masks must share the captured RGB-D pixel grid')
            for i, (box, mask) in enumerate(zip(boxes, masks)):
                detections.append({
                    'id': f'S{i + 1}', 'label': result.names[int(box.cls.item())],
                    'confidence': float(box.conf.item()),
                    'detector_bbox_px': box.xyxy[0].cpu().tolist(), 'mask': mask,
                })
        return SemanticDepthFrame(frame, detections, {
            'model': self.checkpoint, 'classes': self.classes, 'device': self.device,
            'confidence_threshold': self.confidence, 'image_size': self.image_size,
            'nms': True, 'threshold_role': 'candidate generation only; never target acceptance',
        })


class SemanticDepthFrame:
    """CPU bool [N,H,W] masks and float32 depth from exactly one DepthFrame.

    Instance IDs only identify detections within frame_id, never persistent objects.
    Instances may overlap; mask depth includes every valid member without erosion,
    depth clipping, dominant-plane selection, or a semantic relabeling heuristic.
    """

    def __init__(self, frame: DepthFrame, detections: list[dict], model_info: dict):
        self.frame = frame
        self.model_info = model_info
        self.instances = []
        self._masks = {}
        for detection in detections:
            mask = detection['mask']
            if mask.dtype != np.bool_ or mask.shape != frame.depth_z_m.shape:
                raise ValueError('instance mask must be bool [height,width] on frame pixels')
            ys, xs = np.nonzero(mask)
            if not len(xs):
                raise ValueError('a reported instance must have mask pixels')
            instance_id = detection['id']
            self._masks[instance_id] = mask.copy()
            self.instances.append({
                **{key: value for key, value in detection.items() if key != 'mask'},
                'frame_id': frame.frame_id,
                'bbox_px': [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1],
                'pixel_count': int(mask.sum()), **self._measure_mask(mask),
            })

    def _measure_mask(self, mask: np.ndarray) -> dict:
        frame = self.frame
        ys, xs = np.nonzero(mask & frame.valid_mask)
        count = len(xs)
        result = {'valid_count': count, 'valid_depth_fraction': count / int(mask.sum()),
                  'status': 'measured' if count else 'unknown',
                  'optical_depth_m': None, 'base_planar_range_m': None,
                  'sample_pixel_px': None, 'sample_base_point_m': None,
                  'sample_world_point_m': None, 'surface_plane_world': None,
                  'support_world_bounds_m': None}
        if not count:
            return result
        z = frame.depth_z_m[ys, xs].astype(np.float64)
        k = frame.intrinsics
        optical = np.column_stack(((xs - k['cx']) * z / k['fx'],
                                   (ys - k['cy']) * z / k['fy'], z))
        base = optical @ frame.camera_to_base[:3, :3].T + frame.camera_to_base[:3, 3]
        world = base @ frame.world_from_base[:3, :3].T + frame.world_from_base[:3, 3]
        result['support_world_bounds_m'] = [world.min(axis=0).tolist(), world.max(axis=0).tolist()]
        ranges = np.hypot(base[:, 0], base[:, 1])
        for key, values in [('optical_depth_m', z), ('base_planar_range_m', ranges)]:
            result[key] = dict(zip(('p10_m', 'median_m', 'p90_m'),
                                   np.quantile(values, [0.1, 0.5, 0.9]).tolist()))
        # An actual member pixel nearest the valid-mask image centroid.
        sample = np.argmin((xs - xs.mean()) ** 2 + (ys - ys.mean()) ** 2)
        result.update(sample_pixel_px=[int(xs[sample]), int(ys[sample])],
                      sample_base_point_m=base[sample].tolist(),
                      sample_world_point_m=world[sample].tolist())
        if count >= 3:
            centroid = world.mean(axis=0)
            centered = world - centroid
            _, singular, vh = np.linalg.svd(centered, full_matrices=False)
            normal = vh[-1]
            camera_world = (frame.world_from_base @ frame.camera_to_base)[:3, 3]
            if np.dot(normal, camera_world - centroid) < 0:
                normal = -normal
            result['surface_plane_world'] = {
                'centroid_world_m': centroid.tolist(), 'normal_world': normal.tolist(),
                'rms_m': float(np.sqrt(np.mean((centered @ normal) ** 2))),
                'point_count': count, 'in_plane_std_m': (singular[:2][::-1] / np.sqrt(count)).tolist(),
                'definition': 'Least-squares fit of all valid mask pixels; mixed surfaces and nonplanarity remain in RMS. Not proof of object identity or a doorway plane.',
            }
        return result

    def _metadata(self):
        return {'frame_id': self.frame.frame_id, 'sim_time_s': self.frame.sim_time_s,
                'pose_world': self.frame.pose_world, **self.model_info,
                'width': self.frame.width, 'height': self.frame.height,
                'id_scope': 'frame-local mask ID, not entity ID or tracked object identity',
                'bbox_convention': '[x0,y0,x1,y1), original RGB pixels; detector_bbox_px is model xyxy',
                'distance_definition': 'base_planar_range_m = hypot(x_base,y_base)',
                'support_bounds_definition': 'Axis-aligned world XYZ minima/maxima of actual valid depth member pixels, not full physical object extent.',
                'mask_relations_definition': 'Unordered mask pairs. Pixel overlap uses complete masks. Representative XY distance uses measured sample points; support XY gap is the minimum Euclidean separation of observed axis-aligned XY bounds. Missing depth yields null distances. Overlap or coplanarity does not prove entity identity or semantic relationships.',
                'semantic_note': 'YOLOE vocabulary is task configuration. Labels and scores are predictions, not entity identities or task decisions. Use the user goal and original RGB to judge identity and relationships; exact mask membership and depth provide measured geometry.',
                'unknown_meaning': 'No valid depth, not free space. No detections is an actual empty list.'}

    def payload(self) -> dict:
        return {**self._metadata(), 'instances': self.instances, 'mask_relations': self.mask_relations(), 'images': [
            self.frame._packet('semantic_rgb', 'Same-frame RGB with YOLOE mask overlay and frame-local detection IDs.', self.annotated_png(), 'image/png'),
        ]}

    def mask_relations(self) -> list[dict]:
        """Pairwise observed pixel/support geometry, without entity association."""
        relations = []
        for a, b in combinations(self.instances, 2):
            intersection = int(np.count_nonzero(self._masks[a['id']] & self._masks[b['id']]))
            union = a['pixel_count'] + b['pixel_count'] - intersection
            representative_distance = None
            support_gap = None
            if a['sample_world_point_m'] is not None and b['sample_world_point_m'] is not None:
                representative_distance = float(np.linalg.norm(
                    np.asarray(a['sample_world_point_m'][:2]) - b['sample_world_point_m'][:2]))
            if a['support_world_bounds_m'] is not None and b['support_world_bounds_m'] is not None:
                a_bounds = np.asarray(a['support_world_bounds_m'])[:, :2]
                b_bounds = np.asarray(b['support_world_bounds_m'])[:, :2]
                # Each axis contributes zero for overlapping intervals; otherwise their actual gap.
                axis_gaps = np.maximum(np.maximum(a_bounds[0] - b_bounds[1],
                                                 b_bounds[0] - a_bounds[1]), 0.0)
                support_gap = float(np.linalg.norm(axis_gaps))
            relations.append({
                'instance_ids': [a['id'], b['id']], 'intersection_px': intersection,
                'iou': intersection / union,
                'overlap_fraction_of_smaller_mask': intersection / min(a['pixel_count'], b['pixel_count']),
                'representative_xy_distance_m': representative_distance,
                'support_xy_gap_m': support_gap,
            })
        return relations

    def save_masks(self, path: str | Path):
        np.savez_compressed(path, **self._masks)

    def measurement(self, instance_id: str) -> dict:
        """Existing depth layer schema, restricted to this exact instance mask."""
        mask = self._masks[instance_id]
        instance = next(item for item in self.instances if item['id'] == instance_id)
        frame = self.frame
        selected = DepthFrame(frame.frame_id, frame.sim_time_s, frame.pose_world, frame.rgb,
                              np.where(mask, frame.depth_z_m, np.nan), frame.intrinsics,
                              frame.camera_to_base, frame.world_from_base, frame.valid_range_m)
        measurement = selected.measure_region(instance['bbox_px'])
        measurement.update(instance_id=instance_id, label=instance['label'], confidence=instance['confidence'],
                           pixel_count=instance['pixel_count'], valid_ratio=instance['valid_depth_fraction'],
                           unknown_ratio=1 - instance['valid_depth_fraction'],
                           selection_kind='semantic_instance_mask',
                           selection_note='Statistics use only this instance mask, not all pixels inside its bounding box.')
        return {**measurement, 'images': [
            frame._packet('semantic_selection', f'Frame {frame.frame_id}, {instance_id}: original RGB and exact mask; black mask panel pixels are outside the selection.', self.annotated_png(instance_id), 'image/png'),
            frame._packet('crop', 'Original RGB bounding crop; selection itself is the semantic mask, not this rectangle.', frame.region_jpeg(instance['bbox_px'])),
            frame._packet('layers', 'Mask-only depth bands. Gray is outside the selected mask OR invalid depth. Band colors are distances, not semantic classes.', selected.layers_png(instance['bbox_px'], measurement), 'image/png'),
        ]}

    def annotated_png(self, instance_id: str | None = None) -> bytes:
        frame = self.frame
        overlay = frame.rgb.copy()
        membership = np.zeros_like(overlay)
        chosen = self.instances if instance_id is None else [item for item in self.instances if item['id'] == instance_id]
        for instance in chosen:
            mask = self._masks[instance['id']]
            color = np.array(frame._layer_color(int(instance['id'][1:])), dtype=np.uint8)
            overlay[mask] = (overlay[mask] * 0.6 + color * 0.4).astype(np.uint8)
            membership[mask] = color
        canvas = Image.new('RGB', (frame.width * 2, frame.height + 28), 'black')
        canvas.paste(Image.fromarray(overlay), (0, 28))
        canvas.paste(Image.fromarray(membership), (frame.width, 28))
        draw = ImageDraw.Draw(canvas)
        font = ImageFont.load_default(size=14)
        draw.text((4, 4), f'frame {frame.frame_id}: RGB + predictions | exact masks (overlaps painted in ID order)', font=font, fill='white')
        for instance in chosen:
            x0, y0, x1, y1 = instance['bbox_px']
            label = f"{instance['id']} {instance['label']} {instance['confidence']:.2f}"
            for offset in (0, frame.width):
                draw.rectangle((x0 + offset, y0 + 28, x1 - 1 + offset, y1 + 27), outline='white')
                draw.text((x0 + offset, y0 + 28), label, font=font, fill='white', stroke_width=1, stroke_fill='black')
        return frame._encode(canvas, 'PNG')
