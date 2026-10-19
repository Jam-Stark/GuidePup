"""Run one archived, same-frame RGB-D observation through YOLOE."""
from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'sim'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot.state import RobotState
from depth import DepthFrame, DepthImage
from semantic_depth import YOLOESemanticDepth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame', type=Path, required=True)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--device', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confidence', type=float, default=0.03)
    args = parser.parse_args()
    with np.load(args.frame) as archive:
        state = RobotState.from_payload(json.loads(archive['robot_state_json'].item()))
        geometry = DepthImage(archive['frame_id'].item(), state, archive['native_depth_z_m'],
            dict(zip(('fx', 'fy', 'cx', 'cy'), archive['native_depth_intrinsics'])),
            archive['native_depth_camera_to_base'])
        frame = DepthFrame(archive['frame_id'].item(), state, archive['rgb'], archive['depth_z_m'],
            dict(zip(('fx', 'fy', 'cx', 'cy'), archive['intrinsics'])),
            archive['camera_to_base'], geometry=geometry)
    detector = YOLOESemanticDepth(args.checkpoint, args.device, confidence=args.confidence)
    result = detector.predict(frame)
    args.output.mkdir(parents=True, exist_ok=False)
    payload = result.payload()
    (args.output / 'semantic_depth.json').write_text(json.dumps(payload, indent=2, allow_nan=False))
    (args.output / 'semantic_depth.png').write_bytes(result.annotated_png())
    result.save_masks(args.output / 'semantic_masks.npz')
    for instance in result.instances:
        measurement = result.measurement(instance['id'])
        (args.output / f"{instance['id']}.json").write_text(json.dumps(measurement, indent=2, allow_nan=False))
        for packet in measurement['images']:
            extension = 'png' if packet['image']['mimeType'] == 'image/png' else 'jpg'
            (args.output / f"{instance['id']}_{packet['kind']}.{extension}").write_bytes(base64.b64decode(packet['image']['data']))
    print(json.dumps({**result._metadata(), 'instances': result.instances}, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
