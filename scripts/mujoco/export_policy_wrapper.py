#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import replace
from pathlib import Path
import sys
from typing import Any

import torch

from isaaclab.app import AppLauncher

SCRIPTS_DIR = Path(__file__).resolve().parents[1]
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from _extension_source import append_acc_source_dir

parser = argparse.ArgumentParser(description="Export the UniFP adaptation policy as a MuJoCo-ready TorchScript module.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to an RSL-RL checkpoint.")
parser.add_argument("--output-dir", type=str, default=None, help="Optional output directory.")
parser.add_argument("--num-envs", type=int, default=2, help="Number of Isaac envs used for export validation.")
parser.add_argument("--seed", type=int, default=0, help="Environment seed.")
parser.add_argument(
    "--policy-history-length",
    type=int,
    default=None,
    help="Optional override for the policy observation history length used during checkpoint reconstruction/export.",
)
parser.add_argument(
    "--walking-gait-frequency",
    type=float,
    default=None,
    help="Optional override for the gait frequency recorded in the exported smart-gait metadata.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if hasattr(args_cli, "headless"):
    args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

append_acc_source_dir()

import acc  # noqa: F401

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

from acc.algorithms.rsl_rl.runner import OnPolicyRunnerAdaptation
from acc.tasks.manager_based.uniFP_aliengo.agents.rsl_rl_ppo_cfg import PPORunnerCfg
from acc.tasks.manager_based.uniFP_aliengo.unifpaliengo_eval_cfg import UniFPAliengoReversePushEvalCfg

from runtime import default_smart_gait_metadata

MUJOCO_STIFFNESS_COMPENSATION = 0.6
MUJOCO_DAMPING_COMPENSATION = 0.7


def _construct_python_yaml_tag(loader, tag_suffix, node):
    if tag_suffix == "tuple":
        return tuple(loader.construct_sequence(node))
    if tag_suffix == "object/apply:builtins.slice":
        return slice(*loader.construct_sequence(node))
    if hasattr(node, "value"):
        if node.id == "sequence":
            return loader.construct_sequence(node)
        if node.id == "mapping":
            return loader.construct_mapping(node)
    return loader.construct_scalar(node)


def _load_checkpoint_env_params(checkpoint_path: Path) -> dict[str, Any] | None:
    env_params_path = checkpoint_path.parent / "params" / "env.yaml"
    if not env_params_path.exists():
        return None
    try:
        import yaml

        class EnvParamsLoader(yaml.SafeLoader):
            pass

        EnvParamsLoader.add_multi_constructor("tag:yaml.org,2002:python/", _construct_python_yaml_tag)
        loaded = yaml.load(env_params_path.read_text(), Loader=EnvParamsLoader)
    except Exception as exc:
        print(f"[WARN] Failed to read training env params from {env_params_path}: {exc}")
        return None
    if not isinstance(loaded, dict):
        print(f"[WARN] Training env params did not contain a mapping: {env_params_path}")
        return None
    return loaded


def _nested_mapping_get(mapping: Mapping[str, Any], keys: tuple[str, ...]) -> Any | None:
    value: Any = mapping
    for key in keys:
        if not isinstance(value, Mapping) or key not in value:
            return None
        value = value[key]
    return value


def _extract_mujoco_joint_actuator_gains(
    env_params: Mapping[str, Any],
    joint_names: tuple[str, ...],
) -> dict[str, tuple[float, float]] | None:
    actuators = _nested_mapping_get(env_params, ("scene", "robot", "actuators"))
    if not isinstance(actuators, Mapping):
        return None

    controlled_joint_names = set(joint_names)
    gains: dict[str, tuple[float, float]] = {}
    for actuator_cfg in actuators.values():
        if not isinstance(actuator_cfg, Mapping):
            continue
        stiffness = actuator_cfg.get("stiffness")
        damping = actuator_cfg.get("damping")
        actuator_joint_names = actuator_cfg.get("joint_names_expr")
        if stiffness is None or damping is None or not isinstance(actuator_joint_names, (list, tuple)):
            continue
        kp = float(stiffness) * MUJOCO_STIFFNESS_COMPENSATION
        kv = float(damping) * MUJOCO_DAMPING_COMPENSATION
        for joint_name in actuator_joint_names:
            joint_name = str(joint_name)
            if joint_name in controlled_joint_names:
                gains[joint_name] = (kp, kv)

    missing = controlled_joint_names.difference(gains)
    if missing:
        print(f"[WARN] Training env params did not provide gains for joints: {sorted(missing)}")
        return None
    return gains


def apply_checkpoint_metadata(metadata, checkpoint_path: Path):
    env_params = _load_checkpoint_env_params(checkpoint_path)
    if env_params is None:
        return metadata, False

    updates: dict[str, Any] = {}
    gait_frequency = _nested_mapping_get(env_params, ("commands", "gait_phase", "gait_frequency"))
    if gait_frequency is not None:
        updates["gait_frequency_hz"] = float(gait_frequency)

    gains = _extract_mujoco_joint_actuator_gains(env_params, metadata.joint_names)
    if gains is not None:
        updates["joint_actuator_gains"] = gains

    if not updates:
        return metadata, False
    print(f"[INFO] Loaded metadata overrides from training env params: {checkpoint_path.parent / 'params' / 'env.yaml'}")
    return replace(metadata, **updates), True


class AdaptationPolicyWrapper(torch.nn.Module):
    def __init__(self, policy_model):
        super().__init__()
        self.curr_obs_dim = int(policy_model.curr_obs_dim)
        self.history_obs_dim = int(policy_model.history_obs_dim)
        self.flattened_obs_dim = int(policy_model.history_obs_dim)
        self.encoder = policy_model.encoder
        self.actor = policy_model.actor
        self.state_estimator = policy_model.state_estimator
        self.curr_obs_normalizer = policy_model.curr_obs_normalizer
        self.history_obs_normalizer = policy_model.history_obs_normalizer

    def forward(self, policy_obs_flat: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        curr = policy_obs_flat[:, -self.curr_obs_dim :]
        history = policy_obs_flat[:, -self.history_obs_dim :]
        curr = self.curr_obs_normalizer(curr)
        history = self.history_obs_normalizer(history)
        latent = self.encoder(history)
        actor_input = torch.cat((curr, latent), dim=-1)
        action = self.actor(actor_input)
        pred_est = self.state_estimator(latent)
        return action, pred_est


def build_env_cfg() -> UniFPAliengoReversePushEvalCfg:
    env_cfg = UniFPAliengoReversePushEvalCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    if args_cli.walking_gait_frequency is not None:
        env_cfg.commands.gait_phase.gait_frequency = float(args_cli.walking_gait_frequency)
    if args_cli.policy_history_length is not None:
        if int(args_cli.policy_history_length) <= 0:
            raise ValueError(
                f"--policy-history-length must be positive, got {args_cli.policy_history_length}."
            )
        env_cfg.observations.policy.history_length = int(args_cli.policy_history_length)
    return env_cfg


def resolve_output_paths(checkpoint_path: Path, output_dir_arg: str | None) -> tuple[Path, Path]:
    if output_dir_arg:
        output_dir = Path(output_dir_arg).expanduser().resolve()
    else:
        output_dir = (checkpoint_path.parent / "mujoco_exported").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / "policy.pt", output_dir / "policy.json"


def validate_export(policy_model, exported_policy, obs, label: str) -> tuple[float, float]:
    obs_policy = obs["policy"].detach().cpu()
    with torch.inference_mode():
        expected_action = policy_model.act_inference(obs.to("cpu"))
        expected_pred = policy_model.predict(obs.to("cpu"))
        actual_action, actual_pred = exported_policy(obs_policy)
    action_err = torch.max(torch.abs(expected_action - actual_action)).item()
    pred_err = torch.max(torch.abs(expected_pred - actual_pred)).item()
    print(f"[CHECK][{label}] action_max_abs_err={action_err:.3e} pred_max_abs_err={pred_err:.3e}")
    return action_err, pred_err


def main() -> None:
    checkpoint_path = Path(retrieve_file_path(args_cli.checkpoint)).resolve()
    policy_output_path, metadata_output_path = resolve_output_paths(checkpoint_path, args_cli.output_dir)

    env_cfg = build_env_cfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)

    agent_cfg = PPORunnerCfg()
    agent_cfg.device = env.device
    if args_cli.policy_history_length is not None:
        agent_cfg.policy.history_num_frames = int(args_cli.policy_history_length)
    wrapped_env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunnerAdaptation(wrapped_env, agent_cfg.to_dict(), log_dir=None, device=env.device)
    print(f"[INFO] Loading checkpoint: {checkpoint_path}")
    runner.load(str(checkpoint_path))

    obs0 = wrapped_env.get_observations()
    with torch.inference_mode():
        actions = runner.get_inference_policy(device=env.device)(obs0)
        obs1, _, _, _ = wrapped_env.step(actions)

    policy_model = runner.alg.policy
    policy_model.eval()
    policy_model.to("cpu")
    obs0_cpu = obs0.to("cpu")
    obs1_cpu = obs1.to("cpu")

    wrapper = AdaptationPolicyWrapper(policy_model).cpu().eval()
    example_obs = obs0_cpu["policy"]
    try:
        exported_policy = torch.jit.script(wrapper)
        export_mode = "script"
    except Exception as exc:
        print(f"[WARN] torch.jit.script failed, falling back to trace: {exc}")
        exported_policy = torch.jit.trace(wrapper, example_obs)
        export_mode = "trace"
    exported_policy.save(str(policy_output_path))
    exported_policy = torch.jit.load(str(policy_output_path), map_location="cpu")
    exported_policy.eval()

    err0 = validate_export(policy_model, exported_policy, obs0_cpu, "obs0")
    err1 = validate_export(policy_model, exported_policy, obs1_cpu, "obs1")
    max_err = max(*err0, *err1)
    if max_err > 1.0e-5:
        raise RuntimeError(f"Export validation failed: max absolute error {max_err:.3e} exceeds tolerance 1e-5.")

    metadata, has_checkpoint_metadata = apply_checkpoint_metadata(default_smart_gait_metadata(), checkpoint_path)
    gait_frequency_hz = (
        metadata.gait_frequency_hz
        if has_checkpoint_metadata and args_cli.walking_gait_frequency is None
        else float(env_cfg.commands.gait_phase.gait_frequency)
    )
    metadata = replace(
        metadata,
        action_dim=int(actions.shape[-1]),
        per_frame_obs_dim=int(policy_model.curr_obs_dim),
        history_length=int(policy_model.history_num_frames),
        gait_frequency_hz=gait_frequency_hz,
    )
    metadata.write_json(metadata_output_path)

    print(f"[INFO] Export mode: {export_mode}")
    print(f"[INFO] TorchScript policy written to: {policy_output_path}")
    print(f"[INFO] Metadata written to: {metadata_output_path}")

    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
