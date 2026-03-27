#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import platform
import sys
from pathlib import Path

from model_loader import load_mujoco_model_from_path


def has_module(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check whether the local MuJoCo sim2sim runtime is usable.")
    parser.add_argument(
        "--model-path",
        type=Path,
        default=Path("resources/aliengoZ1/mujoco/aliengoZ1.xml"),
        help="Path to a MuJoCo-loadable URDF or MJCF model.",
    )
    parser.add_argument(
        "--policy-path",
        type=Path,
        default=None,
        help="Optional path to the exported TorchScript policy.",
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    print(f"Python: {platform.python_version()} ({sys.executable})")
    print(f"numpy : {has_module('numpy')}")
    print(f"mujoco: {has_module('mujoco')}")
    print(f"torch : {has_module('torch')}")

    if not has_module("mujoco"):
        print("MuJoCo is missing. Install scripts/mujoco/requirements.txt and rerun.")
        return 1

    import mujoco

    model_path = args.model_path.resolve()
    if not model_path.exists():
        print(f"Model path does not exist: {model_path}")
        return 2

    try:
        model = load_mujoco_model_from_path(mujoco, model_path)
    except Exception as exc:
        print(f"MuJoCo failed to load the model: {exc}")
        return 3

    print(f"Loaded model: {model_path}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody} njnt={model.njnt}")
    if model.nu == 0:
        print("Warning: model has no actuators. Policy playback will not work until actuators are added.")

    if args.policy_path is not None:
        policy_path = args.policy_path.resolve()
        if not policy_path.exists():
            print(f"Policy path does not exist: {policy_path}")
            return 4
        if not has_module("torch"):
            print("Torch is missing. The policy file cannot be checked in this environment.")
            return 5
        import torch

        try:
            policy = torch.jit.load(str(policy_path), map_location="cpu")
            print(f"Loaded TorchScript policy: {policy_path}")
            print(policy)
        except Exception as exc:
            print(f"Failed to load TorchScript policy: {exc}")
            return 6

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
