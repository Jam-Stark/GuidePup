#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import re
from typing import Iterable

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit(
        "matplotlib is required for plotting. Please run this script inside the isaaclab conda environment."
    ) from exc


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
PHASE_COLORS = {
    "idle": "#d9d9d9",
    "disturb": "#fff2a8",
    "ramp": "#ffd1a8",
    "peak": "#ffb3b3",
    "recover": "#c9f2c7",
}


@dataclass
class RunData:
    label: str
    csv_path: Path
    output_dir: Path
    env_id: int
    numeric: dict[str, list[float]]
    phases: list[str]
    phase_segments: list[tuple[str, float, float]]

    @property
    def time_s(self) -> list[float]:
        return self.numeric["time_s"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize UniFP guide-pull CSV logs and automatically group related signals into line charts."
    )
    parser.add_argument(
        "csv_paths",
        nargs="+",
        help="One or more CSV paths. Relative paths are resolved from the workspace root.",
    )
    parser.add_argument("--env-id", type=int, default=0, help="Environment id to visualize from the CSV.")
    parser.add_argument(
        "--run-labels",
        nargs="*",
        default=None,
        help="Optional labels matching the number of CSV inputs.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Optional root output directory. If omitted, plots are written next to each CSV.",
    )
    parser.add_argument(
        "--smoothing-window",
        type=int,
        default=1,
        help="Moving-average window for plotted curves. Use 1 to disable smoothing.",
    )
    parser.add_argument("--dpi", type=int, default=180, help="Output image DPI.")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display figures interactively in addition to saving PNG files.",
    )
    return parser.parse_args()


def resolve_path(path_str: str) -> Path:
    path = Path(path_str)
    if not path.is_absolute():
        path = WORKSPACE_ROOT / path
    path = path.resolve()
    if not path.exists():
        raise FileNotFoundError(f"CSV file does not exist: {path}")
    return path


def slugify(text: str) -> str:
    slug = re.sub(r"[^0-9A-Za-z._-]+", "_", text).strip("_")
    return slug or "run"


def safe_float(value: str | None) -> float:
    if value is None:
        return float("nan")
    value = value.strip()
    if value == "":
        return float("nan")
    try:
        return float(value)
    except ValueError:
        return float("nan")


def vector_norm(xs: Iterable[float], ys: Iterable[float], zs: Iterable[float]) -> list[float]:
    norms: list[float] = []
    for x, y, z in zip(xs, ys, zs):
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            norms.append(math.sqrt(x * x + y * y + z * z))
        else:
            norms.append(float("nan"))
    return norms


def moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1:
        return list(values)

    result: list[float] = []
    queue: deque[float] = deque()
    sum_value = 0.0
    count = 0
    for value in values:
        queue.append(value)
        if math.isfinite(value):
            sum_value += value
            count += 1

        if len(queue) > window:
            old = queue.popleft()
            if math.isfinite(old):
                sum_value -= old
                count -= 1

        result.append(sum_value / count if count > 0 else float("nan"))
    return result


def build_phase_segments(times: list[float], phases: list[str]) -> list[tuple[str, float, float]]:
    if not times or not phases:
        return []

    if len(times) > 1:
        dt = max(times[1] - times[0], 0.0)
    else:
        dt = 0.0

    segments: list[tuple[str, float, float]] = []
    start_idx = 0
    current_phase = phases[0]
    for idx in range(1, len(phases)):
        if phases[idx] != current_phase:
            segments.append((current_phase, times[start_idx], times[idx - 1] + dt))
            start_idx = idx
            current_phase = phases[idx]
    segments.append((current_phase, times[start_idx], times[-1] + dt))
    return segments


def shade_phases(ax, run: RunData):
    for phase_name, start_t, end_t in run.phase_segments:
        color = PHASE_COLORS.get(phase_name, "#efefef")
        ax.axvspan(start_t, end_t, color=color, alpha=0.18, lw=0)


def get_series(run: RunData, key: str) -> list[float]:
    if key in run.numeric:
        return run.numeric[key]
    return [float("nan")] * len(run.time_s)


def plot_metric(ax, run: RunData, key: str, label: str, *, color: str | None = None, linestyle: str = "-", window: int = 1):
    values = moving_average(get_series(run, key), window)
    ax.plot(run.time_s, values, label=label, linewidth=1.8, color=color, linestyle=linestyle)


def configure_axis(ax, ylabel: str, title: str | None = None):
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)


def save_figure(fig, output_path: Path, show: bool, dpi: int):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    if show:
        fig.show()
    plt.close(fig)


def available_env_ids(csv_path: Path) -> list[int]:
    ids: set[int] = set()
    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            env_raw = row.get("env_id")
            if env_raw is None:
                continue
            try:
                ids.add(int(float(env_raw)))
            except ValueError:
                continue
    return sorted(ids)


def load_run(csv_path: Path, env_id: int, label: str, output_dir: Path) -> RunData:
    numeric: dict[str, list[float]] = defaultdict(list)
    phases: list[str] = []

    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {csv_path}")

        for row in reader:
            row_env_raw = row.get("env_id")
            if row_env_raw is None:
                continue
            try:
                row_env_id = int(float(row_env_raw))
            except ValueError:
                continue
            if row_env_id != env_id:
                continue

            phase_name = row.get("phase_name", "unknown")
            phases.append(phase_name)
            for field in reader.fieldnames:
                if field == "phase_name":
                    continue
                numeric[field].append(safe_float(row.get(field)))

    if not phases:
        raise ValueError(
            f"No rows found for env_id={env_id} in {csv_path}. Available env_ids: {available_env_ids(csv_path)}"
        )

    numeric["speed_tracking_err"] = [
        abs(forward - cmd)
        for forward, cmd in zip(numeric["forward_speed_proj"], numeric["sampled_cmd_vx"])
    ]
    numeric["force_mag_world"] = [
        math.hypot(force_x, force_y)
        for force_x, force_y in zip(numeric["force_x"], numeric["force_y"])
    ]
    numeric["base_force_tgt_norm"] = vector_norm(
        numeric["base_force_tgt_local_x"],
        numeric["base_force_tgt_local_y"],
        numeric["base_force_tgt_local_z"],
    )
    numeric["base_force_est_norm"] = vector_norm(
        numeric["base_force_est_local_x"],
        numeric["base_force_est_local_y"],
        numeric["base_force_est_local_z"],
    )
    numeric["gripper_force_tgt_norm"] = vector_norm(
        numeric["gripper_force_tgt_local_x"],
        numeric["gripper_force_tgt_local_y"],
        numeric["gripper_force_tgt_local_z"],
    )
    numeric["gripper_force_est_norm"] = vector_norm(
        numeric["gripper_force_est_local_x"],
        numeric["gripper_force_est_local_y"],
        numeric["gripper_force_est_local_z"],
    )

    return RunData(
        label=label,
        csv_path=csv_path,
        output_dir=output_dir,
        env_id=env_id,
        numeric=dict(numeric),
        phases=phases,
        phase_segments=build_phase_segments(numeric["time_s"], phases),
    )


def plot_velocity_bundle(run: RunData, *, window: int, show: bool, dpi: int):
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    shade_phases(axes[0], run)
    plot_metric(axes[0], run, "sampled_cmd_vx", "sampled_cmd_vx", color="black", linestyle="--", window=1)
    plot_metric(axes[0], run, "cmd_vx", "cmd_vx", color="tab:gray", linestyle=":", window=1)
    plot_metric(axes[0], run, "forward_speed_proj", "forward_speed_proj", color="tab:blue", window=window)
    plot_metric(axes[0], run, "speed_xy", "speed_xy", color="tab:orange", window=window)
    configure_axis(axes[0], "velocity (m/s)", f"Velocity tracking | {run.label} | env {run.env_id}")
    axes[0].legend(loc="upper right")

    shade_phases(axes[1], run)
    plot_metric(axes[1], run, "base_vy", "base_vy", color="tab:green", window=window)
    plot_metric(axes[1], run, "lateral_speed_proj", "lateral_speed_proj", color="tab:red", window=window)
    plot_metric(axes[1], run, "speed_tracking_err", "speed_tracking_err", color="tab:purple", window=window)
    configure_axis(axes[1], "velocity / error")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(loc="upper right")

    save_figure(fig, run.output_dir / "velocity_tracking.png", show, dpi)


def plot_force_bundle(run: RunData, *, window: int, show: bool, dpi: int):
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)

    shade_phases(axes[0], run)
    plot_metric(axes[0], run, "force_mag_target", "force_mag_target", color="tab:blue", window=1)
    plot_metric(axes[0], run, "force_mag_applied", "force_mag_applied", color="tab:orange", window=window)
    plot_metric(axes[0], run, "force_mag_world", "sqrt(force_x^2 + force_y^2)", color="tab:green", window=window)
    configure_axis(axes[0], "force magnitude (N)", f"Force profile | {run.label} | env {run.env_id}")
    axes[0].legend(loc="upper right")

    shade_phases(axes[1], run)
    plot_metric(axes[1], run, "force_x", "force_x", color="tab:red", window=window)
    plot_metric(axes[1], run, "force_y", "force_y", color="tab:purple", window=window)
    configure_axis(axes[1], "force component (N)")
    axes[1].legend(loc="upper right")

    shade_phases(axes[2], run)
    plot_metric(axes[2], run, "force_angle_target_deg", "force_angle_target_deg", color="tab:brown", window=1)
    plot_metric(axes[2], run, "force_angle_applied_deg", "force_angle_applied_deg", color="tab:pink", window=window)
    configure_axis(axes[2], "angle (deg)")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")

    save_figure(fig, run.output_dir / "force_profile.png", show, dpi)


def plot_estimation_bundle(
    run: RunData,
    *,
    prefix: str,
    title_prefix: str,
    error_key: str,
    norm_target_key: str,
    norm_est_key: str,
    show: bool,
    dpi: int,
):
    fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    component_keys = (
        (f"{prefix}_x", f"{prefix}_est_local_x", f"{prefix}_tgt_local_x", "x"),
        (f"{prefix}_y", f"{prefix}_est_local_y", f"{prefix}_tgt_local_y", "y"),
        (f"{prefix}_z", f"{prefix}_est_local_z", f"{prefix}_tgt_local_z", "z"),
    )

    for ax, (_, est_key, tgt_key, axis_name) in zip(axes[:3], component_keys):
        shade_phases(ax, run)
        plot_metric(ax, run, tgt_key, f"target_{axis_name}", color="tab:blue", window=1)
        plot_metric(ax, run, est_key, f"estimate_{axis_name}", color="tab:orange", window=1)
        configure_axis(ax, f"{axis_name}-component")
        ax.legend(loc="upper right")

    shade_phases(axes[3], run)
    plot_metric(axes[3], run, error_key, error_key, color="tab:red", window=1)
    plot_metric(axes[3], run, norm_target_key, f"{norm_target_key}", color="tab:green", window=1)
    plot_metric(axes[3], run, norm_est_key, f"{norm_est_key}", color="tab:purple", window=1)
    configure_axis(axes[3], "norm / error", f"{title_prefix} estimation | {run.label} | env {run.env_id}")
    axes[3].set_xlabel("time (s)")
    axes[3].legend(loc="upper right")

    save_figure(fig, run.output_dir / f"{prefix}_estimation.png", show, dpi)


def plot_motion_bundle(run: RunData, *, window: int, show: bool, dpi: int):
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    shade_phases(axes[0], run)
    plot_metric(axes[0], run, "phase_proj_disp", "phase_proj_disp", color="tab:blue", window=window)
    plot_metric(axes[0], run, "phase_lateral_disp", "phase_lateral_disp", color="tab:orange", window=window)
    configure_axis(axes[0], "phase displacement (m)", f"Displacement and root motion | {run.label} | env {run.env_id}")
    axes[0].legend(loc="upper right")

    shade_phases(axes[1], run)
    plot_metric(axes[1], run, "root_x", "root_x", color="tab:green", window=window)
    plot_metric(axes[1], run, "root_y", "root_y", color="tab:red", window=window)
    configure_axis(axes[1], "root position (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(loc="upper right")

    save_figure(fig, run.output_dir / "motion_displacement.png", show, dpi)


def plot_xy_path(run: RunData, *, show: bool, dpi: int):
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    root_x = get_series(run, "root_x")
    root_y = get_series(run, "root_y")
    start_idx = 0
    for phase_name, _, _ in run.phase_segments:
        end_idx = start_idx
        while end_idx < len(run.phases) and run.phases[end_idx] == phase_name:
            end_idx += 1
        color = PHASE_COLORS.get(phase_name, "#808080")
        ax.plot(root_x[start_idx:end_idx], root_y[start_idx:end_idx], label=phase_name, color=color, linewidth=2.0)
        start_idx = end_idx
    ax.scatter([root_x[0]], [root_y[0]], color="black", marker="o", label="start")
    ax.scatter([root_x[-1]], [root_y[-1]], color="black", marker="x", label="end")
    ax.set_title(f"Root XY path | {run.label} | env {run.env_id}")
    ax.set_xlabel("root_x (m)")
    ax.set_ylabel("root_y (m)")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.axis("equal")
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc="best")
    save_figure(fig, run.output_dir / "root_xy_path.png", show, dpi)


def plot_run_figures(run: RunData, *, window: int, show: bool, dpi: int):
    plot_velocity_bundle(run, window=window, show=show, dpi=dpi)
    plot_force_bundle(run, window=window, show=show, dpi=dpi)
    plot_estimation_bundle(
        run,
        prefix="base_force",
        title_prefix="Base force",
        error_key="base_force_err_l2",
        norm_target_key="base_force_tgt_norm",
        norm_est_key="base_force_est_norm",
        show=show,
        dpi=dpi,
    )
    plot_estimation_bundle(
        run,
        prefix="gripper_force",
        title_prefix="Gripper force",
        error_key="gripper_force_err_l2",
        norm_target_key="gripper_force_tgt_norm",
        norm_est_key="gripper_force_est_norm",
        show=show,
        dpi=dpi,
    )
    plot_motion_bundle(run, window=window, show=show, dpi=dpi)
    plot_xy_path(run, show=show, dpi=dpi)


def build_comparison_dir(args: argparse.Namespace, csv_paths: list[Path]) -> Path:
    if args.output_dir:
        return (resolve_path(args.output_dir) if Path(args.output_dir).is_absolute() else (WORKSPACE_ROOT / args.output_dir).resolve()) / "comparison"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return csv_paths[0].parent / f"comparison_{stamp}"


def plot_comparison_velocity(runs: list[RunData], output_dir: Path, *, window: int, show: bool, dpi: int):
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    for ax in axes:
        shade_phases(ax, runs[0])

    for run in runs:
        axes[0].plot(run.time_s, moving_average(get_series(run, "forward_speed_proj"), window), label=f"{run.label} forward_speed")
        axes[0].plot(run.time_s, get_series(run, "sampled_cmd_vx"), linestyle="--", alpha=0.65, label=f"{run.label} sampled_cmd")
        axes[1].plot(run.time_s, moving_average(get_series(run, "speed_tracking_err"), window), label=f"{run.label} speed_err")

    configure_axis(axes[0], "velocity (m/s)", f"Cross-run velocity comparison | env {runs[0].env_id}")
    axes[0].legend(loc="upper right")
    configure_axis(axes[1], "speed error (m/s)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(loc="upper right")

    save_figure(fig, output_dir / "comparison_velocity.png", show, dpi)


def plot_comparison_force_and_error(runs: list[RunData], output_dir: Path, *, window: int, show: bool, dpi: int):
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)

    for ax in axes:
        shade_phases(ax, runs[0])

    for run in runs:
        axes[0].plot(run.time_s, moving_average(get_series(run, "force_mag_applied"), window), label=f"{run.label} applied_force")
        axes[1].plot(run.time_s, moving_average(get_series(run, "base_force_err_l2"), window), label=f"{run.label} base_force_err")
        axes[2].plot(run.time_s, moving_average(get_series(run, "gripper_force_err_l2"), window), label=f"{run.label} gripper_force_err")

    configure_axis(axes[0], "force (N)", f"Cross-run force/error comparison | env {runs[0].env_id}")
    axes[0].legend(loc="upper right")
    configure_axis(axes[1], "base force err")
    axes[1].legend(loc="upper right")
    configure_axis(axes[2], "gripper force err")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")

    save_figure(fig, output_dir / "comparison_force_error.png", show, dpi)


def plot_comparison_motion(runs: list[RunData], output_dir: Path, *, window: int, show: bool, dpi: int):
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    for ax in axes:
        shade_phases(ax, runs[0])

    for run in runs:
        axes[0].plot(run.time_s, moving_average(get_series(run, "phase_proj_disp"), window), label=f"{run.label} phase_proj_disp")
        axes[0].plot(
            run.time_s,
            moving_average(get_series(run, "phase_lateral_disp"), window),
            linestyle="--",
            label=f"{run.label} phase_lateral_disp",
        )
        axes[1].plot(run.time_s, moving_average(get_series(run, "lateral_speed_proj"), window), label=f"{run.label} lateral_speed")

    configure_axis(axes[0], "displacement (m)", f"Cross-run displacement comparison | env {runs[0].env_id}")
    axes[0].legend(loc="upper right")
    configure_axis(axes[1], "lateral speed (m/s)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(loc="upper right")

    save_figure(fig, output_dir / "comparison_motion.png", show, dpi)


def main():
    args = parse_args()
    csv_paths = [resolve_path(path_str) for path_str in args.csv_paths]

    if args.run_labels is not None and len(args.run_labels) not in (0, len(csv_paths)):
        raise ValueError("--run-labels must be omitted or match the number of CSV inputs.")

    root_output_dir = None
    if args.output_dir:
        root_output_dir = Path(args.output_dir)
        if not root_output_dir.is_absolute():
            root_output_dir = (WORKSPACE_ROOT / root_output_dir).resolve()
        root_output_dir.mkdir(parents=True, exist_ok=True)

    runs: list[RunData] = []
    for index, csv_path in enumerate(csv_paths):
        label = args.run_labels[index] if args.run_labels else csv_path.stem
        run_output_dir = (
            root_output_dir / slugify(label)
            if root_output_dir is not None
            else csv_path.parent / f"{csv_path.stem}_plots"
        )
        run = load_run(csv_path, env_id=args.env_id, label=label, output_dir=run_output_dir)
        plot_run_figures(run, window=max(args.smoothing_window, 1), show=args.show, dpi=args.dpi)
        runs.append(run)
        print(f"[INFO] Saved plots for {label} to: {run_output_dir}")

    if len(runs) > 1:
        comparison_dir = build_comparison_dir(args, csv_paths)
        comparison_dir.mkdir(parents=True, exist_ok=True)
        plot_comparison_velocity(runs, comparison_dir, window=max(args.smoothing_window, 1), show=args.show, dpi=args.dpi)
        plot_comparison_force_and_error(
            runs,
            comparison_dir,
            window=max(args.smoothing_window, 1),
            show=args.show,
            dpi=args.dpi,
        )
        plot_comparison_motion(runs, comparison_dir, window=max(args.smoothing_window, 1), show=args.show, dpi=args.dpi)
        print(f"[INFO] Saved cross-run comparison plots to: {comparison_dir}")


if __name__ == "__main__":
    main()
