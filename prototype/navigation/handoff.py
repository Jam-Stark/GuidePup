"""Image-bound target evidence. Identity decisions belong to the planner."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .planner_contract import LIMITS

# This RGB-only path has no position correction. Integrated WBC velocity drifts
# even while physically standing, so XY is evidence, not a view-change veto.
# IMU yaw reuses the existing observation alignment tolerance.
MAX_VIEW_YAW_DEG = LIMITS['turn_tolerance_deg']


def view_freshness(frame, state):
    source = frame['frame'].state
    delta = state.pose_odom-source.pose_odom
    translation = float(np.linalg.norm(delta[:2]))
    yaw = math.degrees(math.atan2(math.sin(delta[2]), math.cos(delta[2])))
    reference_valid = source.epoch == state.epoch and state.tracking == 'tracking'
    return dict(source_frame_id=frame['frame'].frame_id,
        source_capture_s=float(frame['stamp_s']), checked_at_s=float(state.stamp_s),
        image_age_sensor_s=float(state.stamp_s-frame['stamp_s']),
        estimated_translation_m=translation, estimated_yaw_change_deg=yaw,
        translation_validity='unverified_uncorrected_dead_reckoning',
        translation_role='Read-only estimate; cannot independently reject a view or establish physical translation',
        max_yaw_change_deg=MAX_VIEW_YAW_DEG,
        reference_valid=reference_valid,
        fresh=bool(reference_valid and abs(yaw) <= MAX_VIEW_YAW_DEG),
        limitation='IMU rotation/reference screen only. Physical translation and visual identity remain unverified; '
            'GPT compares the source and current RGB. This is not visual registration or a clearance certificate. '
            'There is no image-age or GPT thinking-time deadline.')


def target_evidence(frame, target, result):
    if target['frame_id'] != frame['frame'].frame_id:
        raise ValueError('Navigation target must refer to the supplied current RGB frame')
    height, width = frame['frame'].rgb.shape[:2]
    box = [float(v)*s for v, s in zip(target['bbox_xyxy'], [width, height, width, height])]
    pointing = result.get('pointing')
    if pointing is not None and pointing['frame_size'] != [width, height]:
        raise RuntimeError('Navigation pointing frame size differs from its source RGB')
    point = None if pointing is None else pointing['opos_px']
    inside = None if point is None else bool(box[0] <= point[0] <= box[2] and box[1] <= point[1] <= box[3])
    return dict(source_frame_id=frame['frame'].frame_id, capture_time_s=float(frame['stamp_s']),
        image_size=[width, height], target=target, selected_box_px=box,
        pointing=pointing, object_point_inside_selected_box=inside,
        model_stop=bool(result['stop']), model_visible=result['visible'],
        role='Read-only comparison; overlap and visibility do not certify identity. GPT decides the next action.')


def draw_target_review(frame, evidence, path):
    """Annotate a copy for GPT. The navigation model receives the original JPEG."""
    with Image.open(frame['path']) as original:
        image = original.convert('RGB')
    width, height = image.size
    sheet = Image.new('RGB', (width, height+64), '#f2f5f8')
    sheet.paste(image, (0, 0))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype('DejaVuSans.ttf', 13)
    draw.rectangle(evidence['selected_box_px'], outline='#21bf73', width=3)
    pointing = evidence['pointing']
    if pointing is not None:
        for key, color in [('opos_px', '#ef4353'), ('apos_px', '#f4bb36')]:
            point = pointing[key]
            if point is not None:
                x, y = point
                draw.ellipse((x-6, y-6, x+6, y+6), fill=color, outline='white', width=2)
    draw.text((8, height+5), f"SAME SOURCE: {evidence['source_frame_id']} | t={evidence['capture_time_s']:.3f}s", fill='#173750', font=font)
    draw.text((8, height+25), 'GREEN: GPT selected region | RED: model object point | YELLOW: model action point', fill='#173750', font=font)
    draw.text((8, height+44), 'No movement executed. Point overlap is evidence, not verified target identity.', fill='#4a6074', font=font)
    path = Path(path)
    sheet.save(path, quality=95, subsampling=0)
    return path
