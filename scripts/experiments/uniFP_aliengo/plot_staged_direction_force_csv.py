#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from collections import defaultdict, deque
from dataclasses import dataclass
import math
from pathlib import Path
import re

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit(
        "matplotlib is required for plotting. Please run this script inside the isaaclab conda environment."
    ) from exc


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SEGMENT_COLORS = {
    "idle_0": "#dddddd",
    "idle_1": "#d7d7d7",
    "idle_2": "#d1d1d1",
    "idle_3": "#cbcbcb",
    "forward_low": "#d7ebff",
    "forward_ramp": "#9cd0ff",
    "forward_high": "#5caeff",
    "left_low": "#d8f5d0",
    "left_ramp": "#8edb7a",
    "left_high": "#46b84f",
    "backward_low": "#ffe1c6",
    "backward_ramp": "#ffb878",
    "backward_high": "#f28b39",
}


@dataclass
class RunData:
    label: str
    csv_path: Path
    output_dir: Path
    env_id: int
    numeric: dict[str, list[float]]
    segment_names: list[str]
    phase_names: list[str]
    segment_spans: list[tuple[str, float, float]]

    @property
    def time_s(self) -> list[float]:
        return self.numeric["time_s"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize UniFP staged-direction-force CSV logs with segment shading."
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


def build_segment_spans(times: list[float], segment_names: list[str]) -> list[tuple[str, float, float]]:
    if not times or not segment_names:
        return []

    spans: list[tuple[str, float, float]] = []
    start_idx = 0
    current_segment = segment_names[0]
    for idx in range(1, len(segment_names)):
        if segment_names[idx] != current_segment:
            spans.append((current_segment, times[start_idx], times[idx]))
            start_idx = idx
            current_segment = segment_names[idx]
    spans.append((current_segment, times[start_idx], times[-1]))
    return spans


def shade_segments(ax, run: RunData):
    for segment_name, start_t, end_t in run.segment_spans:
        color = SEGMENT_COLORS.get(segment_name, "#efefef")
        ax.axvspan(start_t, end_t, color=color, alpha=0.18, lw=0)


def get_series(run: RunData, key: str) -> list[float]:
    if key in run.numeric:
        return run.numeric[key]
    return [float("nan")] * len(run.time_s)


def plot_metric(
    ax,
    run: RunData,
    key: str,
    label: str,
    *,
    color: str | None = None,
    linestyle: str = "-",
    window: int = 1,
):
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
    segment_names: list[str] = []
    phase_names: list[str] = []

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

            segment_names.append(row.get("segment_name", "unknown"))
            phase_names.append(row.get("phase_name", "unknown"))
            for field in reader.fieldnames:
                if field in ("segment_name", "direction_name", "phase_name"):
                    continue
                numeric[field].append(safe_float(row.get(field)))

    if not segment_names:
        raise ValueError(
            f"No rows found for env_id={env_id} in {csv_path}. Available env_ids: {available_env_ids(csv_path)}"
        )

    numeric["force_mag_world"] = [
        math.hypot(force_x, force_y)
        for force_x, force_y in zip(numeric["force_x"], numeric["force_y"])
    ]

    return RunData(
        label=label,
        csv_path=csv_path,
        output_dir=output_dir,
        env_id=env_id,
        numeric=dict(numeric),
        segment_names=segment_names,
        phase_names=phase_names,
        segment_spans=build_segment_spans(numeric["time_s"], segment_names),
    )


def build_output_dir(csv_path: Path, output_root: Path | None, label: str) -> Path:
    if output_root is None:
        return csv_path.parent / f"{slugify(label)}_plots"
    return output_root / slugify(label)


def render_force_figure(run: RunData, *, show: bool, dpi: int, window: int):
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    shade_segments(axes[0], run)
    plot_metric(axes[0], run, "force_target_n", "target |F|", color="#0b63a8", linestyle="--", window=window)
    plot_metric(axes[0], run, "force_mag_world", "applied |F|", color="#e07a1f", window=window)
    configure_axis(axes[0], "Force (N)", title=f"{run.label} | Env {run.env_id} | Force Magnitude")
    axes[0].legend(loc="upper right")

    shade_segments(axes[1], run)
    plot_metric(axes[1], run, "force_x", "force_x", color="#1f77b4", window=window)
    plot_metric(axes[1], run, "force_y", "force_y", color="#2ca02c", window=window)
    configure_axis(axes[1], "Force XY (N)")
    axes[1].set_xlabel("Time (s)")
    axes[1].legend(loc="upper right")

    save_figure(fig, run.output_dir / "force_overview.png", show=show, dpi=dpi)


def render_velocity_figure(run: RunData, *, show: bool, dpi: int, window: int):
    fig, ax = plt.subplots(figsize=(14, 4.8))
    shade_segments(ax, run)
    plot_metric(ax, run, "proj_speed", "projected speed", color="#1f77b4", window=window)
    plot_metric(ax, run, "cross_speed", "cross-track speed", color="#d62728", window=window)
    plot_metric(ax, run, "cmd_vx", "cmd_vx", color="#444444", linestyle="--", window=window)
    configure_axis(ax, "Velocity (m/s)", title=f"{run.label} | Env {run.env_id} | Projected vs Cross-Track Velocity")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")
    save_figure(fig, run.output_dir / "velocity_projection.png", show=show, dpi=dpi)


def render_displacement_figure(run: RunData, *, show: bool, dpi: int, window: int):
    fig, ax = plt.subplots(figsize=(14, 4.8))
    shade_segments(ax, run)
    plot_metric(ax, run, "proj_disp", "projected displacement", color="#1f77b4", window=window)
    plot_metric(ax, run, "cross_disp", "cross-track displacement", color="#d62728", window=window)
    configure_axis(
        ax,
        "Displacement (m)",
        title=f"{run.label} | Env {run.env_id} | Projected vs Cross-Track Displacement",
    )
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")
    save_figure(fig, run.output_dir / "displacement_projection.png", show=show, dpi=dpi)


def main():
    args = parse_args()

    csv_paths = [resolve_path(path_str) for path_str in args.csv_paths]
    if args.run_labels is not None and len(args.run_labels) not in (0, len(csv_paths)):
        raise ValueError("--run-labels must be omitted or match the number of CSV inputs.")

    output_root = None
    if args.output_dir is not None:
        output_root = Path(args.output_dir)
        if not output_root.is_absolute():
            output_root = (WORKSPACE_ROOT / output_root).resolve()

    for idx, csv_path in enumerate(csv_paths):
        if args.run_labels:
            label = args.run_labels[idx]
        else:
            label = csv_path.stem
        output_dir = build_output_dir(csv_path, output_root, label)
        run = load_run(csv_path, args.env_id, label, output_dir)
        render_force_figure(run, show=args.show, dpi=args.dpi, window=args.smoothing_window)
        render_velocity_figure(run, show=args.show, dpi=args.dpi, window=args.smoothing_window)
        render_displacement_figure(run, show=args.show, dpi=args.dpi, window=args.smoothing_window)
        print(f"[INFO] Wrote plots for {run.label} to: {run.output_dir}")


if __name__ == "__main__":
    main()
