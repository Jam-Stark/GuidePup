#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys

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

from runtime import default_staged_eval_metadata


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

    metadata = replace(
        default_staged_eval_metadata(),
        action_dim=int(actions.shape[-1]),
        per_frame_obs_dim=int(policy_model.curr_obs_dim),
        history_length=int(policy_model.history_num_frames),
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
