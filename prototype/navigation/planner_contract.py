"""One provider-independent robot decision/observation-skill contract."""
from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTRACT = json.loads((HERE/'planner-contract.json').read_text())
SCHEMA = CONTRACT['schema']
SKILLS = {k: v for k, v in CONTRACT['actions'].items() if k.endswith('_observe')}
LIMITS = CONTRACT['observation_limits']
SYSTEM = (HERE/'planner-system.txt').read_text()+'\n'+(
    HERE.parent/'pi/prompts/lightnav-instruction-rules.txt').read_text()


def validate_decision(result):
    if not isinstance(result, dict) or set(result) != set(SCHEMA['required']):
        raise ValueError('Planner decision must use the exact robot action schema')
    action = result['action']
    if action not in CONTRACT['actions']:
        raise ValueError(f'Unknown planner action {action!r}')
    if not all(isinstance(result[k], str) and result[k].strip() for k in ('reason', 'evidence', 'uncertainty')):
        raise ValueError('Planner must provide reason, evidence and uncertainty')
    used = {'navigate': {'instruction', 'target'}, 'confirm_navigation': set(), 'turn_observe': {'angle_deg'},
            'advance_observe': {'speed_m_s', 'duration_s'}, 'hold_observe': {'duration_s'},
            'complete': set(), 'needs_user': set()}[action]
    for key in ('instruction', 'target', 'angle_deg', 'speed_m_s', 'duration_s'):
        value = result[key]
        if key not in used:
            if value is not None:
                raise ValueError(f'{action} requires unused {key} to be null')
        elif key == 'instruction':
            if not isinstance(value, str) or not value.strip():
                raise ValueError('Navigation requires the actual full instruction')
        elif key == 'target':
            if not isinstance(value, dict) or set(value) != {'frame_id', 'bbox_xyxy'}:
                raise ValueError('Navigation target must contain exactly frame_id and bbox_xyxy')
            if not isinstance(value['frame_id'], str) or not value['frame_id'].strip():
                raise ValueError('Navigation target requires a nonempty frame_id')
            box = value['bbox_xyxy']
            if not isinstance(box, list) or len(box) != 4 or any(
                isinstance(v, bool) or not isinstance(v, (float, int))
                or not math.isfinite(v) or not 0 <= v <= 1 for v in box
            ):
                raise ValueError('Navigation target bbox_xyxy requires four finite normalized coordinates')
            if not box[0] < box[2] or not box[1] < box[3]:
                raise ValueError('Navigation target bbox_xyxy must have positive width and height')
        elif isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f'{action} requires finite numeric {key}')
    if action == 'turn_observe' and not LIMITS['min_turn_angle_deg'] <= abs(result['angle_deg']) <= LIMITS['max_turn_angle_deg']:
        raise ValueError('Observation turn magnitude must be 10 to 180 degrees')
    if action == 'advance_observe' and not LIMITS['min_forward_speed_m_s'] <= result['speed_m_s'] <= LIMITS['max_forward_speed_m_s']:
        raise ValueError('Observation forward speed must be 0.10 to 0.20 m/s')
    if 'duration_s' in used and not LIMITS['min_duration_s'] <= result['duration_s'] <= LIMITS['max_duration_s']:
        raise ValueError('Observation duration must be 0.5 to 4.0 seconds')
    return result
