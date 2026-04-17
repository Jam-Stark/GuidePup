#!/usr/bin/env python3

from __future__ import annotations

"""Export IO descriptors for a registered IsaacLab environment.

This script is based on the official IsaacLab exporter at
`../IsaacLab/scripts/environments/export_IODescriptors.py`, with two local
adaptations for this repository:

1. Append the editable `acc` extension source and import `acc.tasks` so custom
   tasks such as `Template-UniFPAliengo-v0` are registered before `gym.make()`.
2. Write a single `io.yaml` file by default for easier downstream comparison.

The descriptor payload itself still comes from IsaacLab's official environment
API via `env.unwrapped.get_IO_descriptors`.
"""

import argparse
import os
from pathlib import Path
import sys

from isaaclab.app import AppLauncher


SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir


parser = argparse.ArgumentParser(description="Export IO descriptors for a registered IsaacLab environment.")
parser.add_argument(
    "--task",
    type=str,
    default=None,
    help="Registered task name, required when --env-cfg=registered_task.",
)
parser.add_argument(
    "--env-cfg",
    type=str,
    default="registered_task",
    choices=("registered_task", "reverse_push_eval"),
    help="How to construct the Isaac environment before exporting descriptors.",
)
parser.add_argument(
    "--descriptor-name",
    type=str,
    default=None,
    help="Optional label used for the default output directory when --output-dir is omitted.",
)
parser.add_argument(
    "--output-dir",
    type=str,
    default=None,
    help="Directory that will receive the YAML file. Defaults to io_descriptors/<task_name>/.",
)
parser.add_argument(
    "--output-file",
    type=str,
    default="io.yaml",
    help="Output YAML filename written under --output-dir.",
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments used for task instantiation.")
parser.add_argument(
    "--policy-history-length",
    type=int,
    default=None,
    help="Optional override for observations.policy.history_length before exporting descriptors.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


append_acc_source_dir()

import gymnasium as gym
import yaml

import isaaclab_tasks  # noqa: F401
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab_tasks.utils import parse_env_cfg

import acc.tasks  # noqa: F401
from acc.tasks.manager_based.uniFP_aliengo.unifpaliengo_eval_cfg import UniFPAliengoReversePushEvalCfg


def export_articulations_data_compat(env) -> dict[str, dict[str, list[float]]]:
    """Compatibility copy of IsaacLab articulation export.

    Some installed IsaacLab builds do not expose
    `isaaclab.envs.utils.io_descriptors`, while the current upstream source does.
    To keep Phase A exporter usable across those builds, we inline the minimal
    articulation export used by the official implementation.
    """

    articulation_joint_data = {}
    for articulation_name, articulation in env.scene.articulations.items():
        articulation_joint_data[articulation_name] = {}
        articulation_joint_data[articulation_name]["joint_names"] = articulation.joint_names
        articulation_joint_data[articulation_name]["default_joint_pos"] = (
            articulation.data.default_joint_pos[0].detach().cpu().numpy().tolist()
        )
        articulation_joint_data[articulation_name]["default_joint_vel"] = (
            articulation.data.default_joint_vel[0].detach().cpu().numpy().tolist()
        )
        articulation_joint_data[articulation_name]["default_joint_pos_limits"] = (
            articulation.data.default_joint_pos_limits[0].detach().cpu().numpy().tolist()
        )
        articulation_joint_data[articulation_name]["default_joint_damping"] = (
            articulation.data.default_joint_damping[0].detach().cpu().numpy().tolist()
        )
        articulation_joint_data[articulation_name]["default_joint_stiffness"] = (
            articulation.data.default_joint_stiffness[0].detach().cpu().numpy().tolist()
        )
        articulation_joint_data[articulation_name]["default_joint_friction"] = (
            articulation.data.default_joint_friction[0].detach().cpu().numpy().tolist()
        )
        articulation_joint_data[articulation_name]["default_joint_armature"] = (
            articulation.data.default_joint_armature[0].detach().cpu().numpy().tolist()
        )
    return articulation_joint_data


def export_scene_data_compat(env) -> dict[str, float | int]:
    """Compatibility copy of IsaacLab scene export."""

    return {"physics_dt": env.physics_dt, "dt": env.step_dt, "decimation": env.cfg.decimation}


def make_yaml_safe(value):
    """Recursively convert descriptor payloads into YAML-safe builtins."""

    try:
        import torch
    except Exception:  # pragma: no cover
        torch = None

    if value is None or isinstance(value, (str, int, float, bool)):
        return value

    if isinstance(value, Path):
        return str(value)

    if torch is not None and isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy().tolist()

    if isinstance(value, dict):
        return {str(key): make_yaml_safe(val) for key, val in value.items()}

    if isinstance(value, (list, tuple, set)):
        return [make_yaml_safe(item) for item in value]

    if hasattr(value, "func") and hasattr(value, "params"):
        func = getattr(value, "func")
        if hasattr(func, "__module__") and hasattr(func, "__name__"):
            func_name = f"{func.__module__}.{func.__name__}"
        else:
            func_name = str(func)
        return {
            "__class__": f"{value.__class__.__module__}.{value.__class__.__name__}",
            "func": func_name,
            "params": make_yaml_safe(dict(getattr(value, "params", {}))),
        }

    if hasattr(value, "tolist"):
        try:
            return value.tolist()
        except Exception:
            pass

    if hasattr(value, "__dict__"):
        public_items = {
            key: make_yaml_safe(val)
            for key, val in vars(value).items()
            if not key.startswith("_")
        }
        if public_items:
            public_items["__class__"] = f"{value.__class__.__module__}.{value.__class__.__name__}"
            return public_items

    return str(value)


def collect_observation_group_descriptors(env, group_names: list[str]) -> dict[str, list[dict]]:
    """Collect IO descriptors for selected observation groups only.

    This mirrors IsaacLab's formatting logic but avoids the upstream getter's current
    behavior of inspecting all groups before filtering. That matters here because we
    only want the Phase-A `policy` group and some non-policy custom terms may still be
    incomplete or slow to inspect.
    """

    import torch

    manager = env.unwrapped.observation_manager
    formatted_data: dict[str, list[dict]] = {}

    def _clean_description(func) -> str:
        doc = getattr(func, "__doc__", None)
        if not doc:
            return ""
        return " ".join(doc.split())

    def _descriptor_template(func, shape: tuple[int, ...], dtype: str, observation_type: str, *, extras=None) -> dict:
        return {
            "name": func.__name__,
            "mdp_type": "Observation",
            "full_path": f"{func.__module__}.{func.__name__}",
            "description": _clean_description(func),
            "shape": shape,
            "dtype": dtype,
            "observation_type": observation_type,
            "extras": {} if extras is None else dict(extras),
        }

    def _manual_descriptor(term_name, term_cfg, shape: tuple[int, ...], dtype: str) -> dict:
        func = term_cfg.func
        func_name = func.__name__

        if func_name == "base_orientation_2d":
            return _descriptor_template(
                func,
                shape,
                dtype,
                "RootState",
                extras={"units": "unit", "axes": ["projected_gravity_x", "projected_gravity_y"]},
            )
        if func_name == "base_ang_vel":
            return _descriptor_template(
                func,
                shape,
                dtype,
                "RootState",
                extras={"units": "rad/s", "axes": ["X", "Y", "Z"]},
            )
        if func_name == "joint_pos_rel":
            return _descriptor_template(func, shape, dtype, "JointState", extras={"units": "rad"})
        if func_name == "joint_vel_rel":
            return _descriptor_template(func, shape, dtype, "JointState", extras={"units": "rad/s"})
        if func_name == "last_action":
            return _descriptor_template(func, shape, dtype, "Action")
        if func_name == "gait_clock_signal":
            return _descriptor_template(
                func,
                shape,
                dtype,
                "Command",
                extras={"units": "unit", "axes": ["sin_phase", "cos_phase"]},
            )
        if func_name == "all_commands_with_scales":
            return _descriptor_template(
                func,
                shape,
                dtype,
                "Command",
                extras={
                    "units": "mixed",
                    "axes": [
                        "base_lin_vel_x",
                        "base_lin_vel_y",
                        "base_ang_vel_z",
                        "ee_sphere_radius",
                        "ee_sphere_pitch",
                        "ee_sphere_yaw",
                        "ee_orient_roll",
                        "ee_orient_pitch",
                        "ee_orient_yaw",
                        "gripper_force_x",
                        "gripper_force_y",
                        "gripper_force_z",
                        "base_force_x",
                        "base_force_y",
                        "base_force_z",
                    ],
                },
            )
        if func_name == "locomotion_commands_with_scales":
            return _descriptor_template(
                func,
                shape,
                dtype,
                "Command",
                extras={
                    "units": "mixed",
                    "axes": [
                        "base_lin_vel_x",
                        "base_lin_vel_y",
                        "base_ang_vel_z",
                    ],
                },
            )

        return _descriptor_template(func, shape, dtype, "Observation")

    def _normalize_single_frame_shape(term_cfg, stored_dims: tuple[int, ...]) -> tuple[int, ...]:
        if getattr(term_cfg, "history_length", 0) > 0:
            history_length = int(term_cfg.history_length)
            if getattr(term_cfg, "flatten_history_dim", False):
                if len(stored_dims) == 1:
                    if stored_dims[0] % history_length != 0:
                        raise RuntimeError(
                            f"Unable to infer single-frame shape from flattened dims={stored_dims} "
                            f"and history_length={history_length}."
                        )
                    return (stored_dims[0] // history_length,)
            else:
                if len(stored_dims) >= 2:
                    return tuple(stored_dims[1:])
        return tuple(stored_dims)

    def _joint_names_from_asset_cfg(asset_cfg) -> list[str]:
        asset = manager._env.scene[asset_cfg.name]
        joint_ids = asset_cfg.joint_ids
        if joint_ids == slice(None, None, None):
            joint_ids = list(range(len(asset.joint_names)))
        return [asset.joint_names[i] for i in joint_ids]

    def _joint_pos_offsets_from_asset_cfg(asset_cfg):
        asset = manager._env.scene[asset_cfg.name]
        ids = asset_cfg.joint_ids
        return asset.data.default_joint_pos[:, ids][0].detach().cpu().numpy().tolist()

    def _joint_vel_offsets_from_asset_cfg(asset_cfg):
        asset = manager._env.scene[asset_cfg.name]
        ids = asset_cfg.joint_ids
        return asset.data.default_joint_vel[:, ids][0].detach().cpu().numpy().tolist()

    for group_name in group_names:
        if group_name not in manager._group_obs_term_names:
            raise ValueError(
                f"Unable to find observation group '{group_name}'. "
                f"Available groups: {list(manager._group_obs_term_names.keys())}"
            )

        print(f"[INFO] Inspecting observation group: {group_name}", flush=True)
        group_term_names = manager._group_obs_term_names[group_name]
        group_term_dims = manager.group_obs_term_dim[group_name]
        obs_terms = zip(group_term_names, manager._group_obs_term_cfgs[group_name], group_term_dims)

        formatted_group: list[dict] = []
        for term_name, term_cfg, stored_dims in obs_terms:
            print(f"[INFO]   Inspecting observation term: {term_name}", flush=True)
            normalized_shape = _normalize_single_frame_shape(term_cfg, stored_dims)
            dtype = str(env.unwrapped.obs_buf[group_name].dtype)
            print(f"[INFO]   Shape normalized: {term_name} -> {normalized_shape}", flush=True)
            print(f"[INFO]   Dtype recorded: {term_name} -> {dtype}", flush=True)

            item = _manual_descriptor(term_name, term_cfg, normalized_shape, dtype)
            print(f"[INFO]   Manual descriptor built: {term_name}", flush=True)

            asset_cfg = term_cfg.params.get("asset_cfg") if hasattr(term_cfg, "params") else None
            print(
                f"[INFO]   Asset cfg resolved: {term_name} -> "
                f"{getattr(asset_cfg, 'name', None)} / obs_type={item.get('observation_type')}",
                flush=True,
            )
            if asset_cfg is not None and item.get("observation_type") == "JointState":
                item["joint_names"] = _joint_names_from_asset_cfg(asset_cfg)
                if term_cfg.func.__name__ == "joint_pos_rel":
                    item["joint_pos_offsets"] = _joint_pos_offsets_from_asset_cfg(asset_cfg)
                if term_cfg.func.__name__ == "joint_vel_rel":
                    item["joint_vel_offsets"] = _joint_vel_offsets_from_asset_cfg(asset_cfg)

            if term_cfg.func.__name__ == "gait_clock_signal":
                item["command_name"] = term_cfg.params.get("gait_command_name", "gait_phase")

            if term_cfg.func.__name__ == "all_commands_with_scales":
                item["command_term_names"] = {
                    "locomotion": term_cfg.params.get("locomotion_command_name", "locomotion_commands"),
                    "target_position": term_cfg.params.get("target_position_command_name", "target_position_commands"),
                    "gripper_force": term_cfg.params.get("gripper_force_command_name", "gripper_force_command"),
                    "base_force": term_cfg.params.get("base_force_command_name", "base_force_command"),
                }
                item["command_scales"] = list(term_cfg.params.get("scales", []))
            if term_cfg.func.__name__ == "locomotion_commands_with_scales":
                item["command_term_names"] = {
                    "locomotion": term_cfg.params.get("locomotion_command_name", "locomotion_commands"),
                }
                item["command_scales"] = list(term_cfg.params.get("scales", []))

            print(f"[INFO]   Observation term metadata assembled from cached config: {term_name}", flush=True)
            overloads = {}
            for key, value in term_cfg.__dict__.items():
                if key in ["modifiers", "clip", "scale", "history_length", "flatten_history_dim"]:
                    overloads[key] = value
            item.update(overloads)

            name = item.pop("name")
            formatted_item = {"name": name, "overloads": {}, "extras": item.pop("extras")}
            for key, value in item.items():
                if isinstance(value, tuple):
                    value = list(value)
                if isinstance(value, torch.Tensor):
                    value = value.detach().cpu().numpy().tolist()
                if key in ["scale", "clip", "history_length", "flatten_history_dim"]:
                    formatted_item["overloads"][key] = value
                elif key in ["modifiers", "description", "units"]:
                    formatted_item["extras"][key] = value
                else:
                    formatted_item[key] = value
            formatted_group.append(formatted_item)
            print(f"[INFO]   Observation term formatted: {term_name}", flush=True)

        formatted_data[group_name] = formatted_group

    return formatted_data


def collect_action_descriptors(env) -> list[dict]:
    """Collect Phase-A action descriptors without calling the upstream getter.

    Similar to observations, this avoids depending on installed IsaacLab versions or
    custom term IO paths during the minimal export pass. For Phase A we only need the
    policy-controlled joint position actions.
    """

    import torch

    manager = env.unwrapped.action_manager
    formatted_data: list[dict] = []

    for term_name, term in manager._terms.items():
        print(f"[INFO] Inspecting action term: {term_name}", flush=True)

        action_dim = int(getattr(term, "action_dim", 0))
        print(f"[INFO]   Action dim: {term_name} -> {action_dim}", flush=True)
        if action_dim <= 0:
            print(f"[INFO]   Skipping non-policy action term: {term_name}", flush=True)
            continue

        formatted_item = {
            "name": term_name,
            "mdp_type": "Action",
            "full_path": f"{term.__class__.__module__}.{term.__class__.__name__}",
            "shape": (action_dim,),
            "dtype": str(term.raw_actions.dtype),
            "extras": {
                "description": " ".join((term.__class__.__doc__ or "").split()),
            },
        }

        if hasattr(term, "_joint_names"):
            formatted_item["action_type"] = "JointAction"
            formatted_item["joint_names"] = list(term._joint_names)
        else:
            formatted_item["action_type"] = term.__class__.__name__

        if hasattr(term, "_scale"):
            scale = term._scale
            if isinstance(scale, torch.Tensor):
                scale = scale.detach().cpu().numpy().tolist()
            formatted_item["scale"] = scale

        if hasattr(term, "_offset"):
            offset = term._offset
            if isinstance(offset, torch.Tensor):
                offset = offset[0].detach().cpu().numpy().tolist()
            formatted_item["offset"] = offset

        if hasattr(term, "_clip"):
            clip = term._clip
            if isinstance(clip, torch.Tensor):
                clip = clip[0].detach().cpu().numpy().tolist()
            formatted_item["clip"] = clip
        else:
            formatted_item["clip"] = None

        if hasattr(term.cfg, "min_delay_steps") and hasattr(term.cfg, "max_delay_steps"):
            formatted_item["delay_range_steps"] = [int(term.cfg.min_delay_steps), int(term.cfg.max_delay_steps)]

        formatted_data.append(formatted_item)
        print(f"[INFO]   Action term formatted: {term_name}", flush=True)

    return formatted_data


def resolve_output_path(descriptor_name: str, output_dir_arg: str | None, output_file: str) -> Path:
    task_stem = descriptor_name.lower().replace("-", "_").replace(" ", "_")
    if output_dir_arg:
        output_dir = Path(output_dir_arg).expanduser().resolve()
    else:
        output_dir = (Path("io_descriptors") / task_stem).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / output_file


def build_env() -> tuple[ManagerBasedRLEnv, str]:
    if args_cli.policy_history_length is not None and int(args_cli.policy_history_length) <= 0:
        raise ValueError(f"--policy-history-length must be positive, got {args_cli.policy_history_length}.")

    if args_cli.env_cfg == "reverse_push_eval":
        env_cfg = UniFPAliengoReversePushEvalCfg()
        env_cfg.scene.num_envs = args_cli.num_envs
        if args_cli.device is not None:
            env_cfg.sim.device = args_cli.device
        if args_cli.policy_history_length is not None:
            env_cfg.observations.policy.history_length = int(args_cli.policy_history_length)
        env = ManagerBasedRLEnv(cfg=env_cfg)
        descriptor_name = args_cli.descriptor_name or "unifp_reverse_push_eval"
        return env, descriptor_name

    if not args_cli.task:
        raise ValueError("--task is required when --env-cfg=registered_task.")

    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=True,
    )
    if args_cli.policy_history_length is not None:
        env_cfg.observations.policy.history_length = int(args_cli.policy_history_length)
    env = gym.make(args_cli.task, cfg=env_cfg)
    descriptor_name = args_cli.descriptor_name or args_cli.task
    return env, descriptor_name


def main() -> None:
    env, descriptor_name = build_env()

    print(f"[INFO] Gym observation space: {env.observation_space}")
    print(f"[INFO] Gym action space: {env.action_space}")

    print("[INFO] Resetting environment...", flush=True)
    env.reset()
    print("[INFO] Environment reset complete.", flush=True)

    print("[INFO] Collecting observation IO descriptors for group: policy", flush=True)
    observation_descriptors = collect_observation_group_descriptors(env, ["policy"])
    print("[INFO] Observation IO descriptors collected.", flush=True)

    print("[INFO] Collecting action IO descriptors...", flush=True)
    action_descriptors = collect_action_descriptors(env)
    print("[INFO] Action IO descriptors collected.", flush=True)

    descriptors = {
        "observations": observation_descriptors,
        "actions": action_descriptors,
        "articulations": export_articulations_data_compat(env.unwrapped),
        "scene": export_scene_data_compat(env.unwrapped),
    }
    output_path = resolve_output_path(descriptor_name, args_cli.output_dir, args_cli.output_file)

    with output_path.open("w") as f:
        print(f"[INFO] Exporting IO descriptors to: {output_path}", flush=True)
        yaml.safe_dump(make_yaml_safe(descriptors), f, sort_keys=False, allow_unicode=True)
        f.flush()
        os.fsync(f.fileno())
    print(f"[INFO] YAML write complete: {output_path}", flush=True)

    observation_groups = descriptors.get("observations", {})
    action_terms = descriptors.get("actions", [])
    print(f"[INFO] Observation groups exported: {list(observation_groups.keys())}", flush=True)
    print(f"[INFO] Action terms exported: {[term.get('name') for term in action_terms]}", flush=True)

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
