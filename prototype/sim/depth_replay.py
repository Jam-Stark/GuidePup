"""Read-only, fixed-frame RGB-D replay for matched model comparisons."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Literal

import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot.state import RobotState
from depth import DepthFrame, DepthImage


class Region(BaseModel):
    frame_id: str
    bbox_px: list[int]


def load_frame(path: Path) -> DepthFrame:
    with np.load(path) as data:
        state = RobotState.from_payload(json.loads(data['robot_state_json'].item()))
        geometry = DepthImage(data['frame_id'].item(), state, data['native_depth_z_m'],
            dict(zip(('fx', 'fy', 'cx', 'cy'), data['native_depth_intrinsics'])),
            data['native_depth_camera_to_base'])
        return DepthFrame(data['frame_id'].item(), state, data['rgb'], data['depth_z_m'],
            dict(zip(('fx', 'fy', 'cx', 'cy'), data['intrinsics'])),
            data['camera_to_base'], geometry=geometry)


def baseline_statistics(metadata: dict) -> None:
    """Reproduce the previous presentation, only as the experiment baseline."""
    metadata.pop('layer_note')
    regions = metadata['cells'] if 'cells' in metadata else [metadata]
    for region in regions:
        for layer in region['layers']:
            layer.pop('layer_id')
            for key in ('color_rgb', 'pixel_bbox_px', 'sample_pixel_px'):
                layer.pop(key, None)


def make_app(manifest_path: Path) -> FastAPI:
    manifest = json.loads(manifest_path.read_text())
    frames = {case['case_id']: load_frame(Path(case['frame_npz'])) for case in manifest['cases']}
    profile_path = Path(__file__).resolve().parents[1] / 'camera/profiles.json'
    camera = json.loads(profile_path.read_text())['profiles']['d435i_rgb_nominal']
    app = FastAPI()

    def get_frame(case_id: str) -> DepthFrame:
        if case_id not in frames:
            raise HTTPException(404, f'Unknown replay case: {case_id}')
        return frames[case_id]

    @app.get('/cases/{case_id}/status')
    def status(case_id: str):
        frame = get_frame(case_id)
        return {'state': 'FRAME_REPLAY', 'live_control': False,
                'scene_frame_id': frame.frame_id, 'sim_time_s': frame.stamp_s,
                'robot_state': frame.state.payload(), 'camera': 'd435i_rgb_nominal',
                'observation_note': 'Frozen RGB-D observation; no live motion or arrival state is asserted.'}

    @app.get('/cases/{case_id}/scene')
    def scene(case_id: str, presentation: Literal['baseline', 'enhanced']):
        payload = get_frame(case_id).scene_payload(camera)
        if presentation == 'baseline':
            payload['images'] = [view for view in payload['images'] if view['kind'] == 'grid']
            baseline_statistics(payload['depth'])
        return payload

    @app.post('/cases/{case_id}/measure_region')
    def measure_region(case_id: str, body: Region, presentation: Literal['baseline', 'enhanced']):
        frame = get_frame(case_id)
        if body.frame_id != frame.frame_id:
            raise HTTPException(409, 'Use the exact frame_id returned by this replay scene.')
        try:
            payload = frame.measurement_payload(body.bbox_px)
        except ValueError as error:
            raise HTTPException(400, str(error)) from error
        if presentation == 'baseline':
            payload['images'] = [view for view in payload['images'] if view['kind'] == 'crop']
            baseline_statistics(payload)
        return payload

    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8767)
    args = parser.parse_args()
    uvicorn.run(make_app(args.manifest), host='127.0.0.1', port=args.port, log_level='info')
