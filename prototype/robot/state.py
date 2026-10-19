"""The estimator-owned robot-state contract."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class RobotState:
    stamp_s: float
    epoch: str
    odom_from_base: np.ndarray
    twist_base: np.ndarray
    tracking: str
    source: str
    quality: dict[str, Any]

    def __post_init__(self) -> None:
        transform = np.asarray(self.odom_from_base, dtype=np.float64)
        twist = np.asarray(self.twist_base, dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError("odom_from_base must be finite 4x4")
        if twist.shape != (6,) or not np.isfinite(twist).all():
            raise ValueError("twist_base must be finite [vx, vy, vz, wx, wy, wz]")
        if not math.isfinite(float(self.stamp_s)):
            raise ValueError("stamp_s must be finite")
        if not self.epoch:
            raise ValueError("epoch must be nonempty")
        if self.tracking not in {"initializing", "tracking", "lost"}:
            raise ValueError(f"unsupported tracking state {self.tracking!r}")
        if not self.source:
            raise ValueError("source must be nonempty")
        object.__setattr__(self, "odom_from_base", transform.copy())
        object.__setattr__(self, "twist_base", twist.copy())
        object.__setattr__(self, "quality", dict(self.quality))

    @property
    def pose_odom(self) -> np.ndarray:
        rotation = self.odom_from_base[:3, :3]
        return np.array([
            self.odom_from_base[0, 3],
            self.odom_from_base[1, 3],
            math.atan2(rotation[1, 0], rotation[0, 0]),
        ], dtype=np.float64)

    @property
    def planar_twist(self) -> np.ndarray:
        rotation = self.odom_from_base[:3, :3]
        forward_odom = rotation[:, 0]
        heading = forward_odom[:2].copy()
        heading_norm_sq = float(heading @ heading)
        if heading_norm_sq == 0.0:
            raise RuntimeError("estimated base forward axis has no odom-plane projection")
        heading /= math.sqrt(heading_norm_sq)
        left = np.array([-heading[1], heading[0]])
        linear_odom = rotation @ self.twist_base[:3]
        angular_odom = rotation @ self.twist_base[3:]
        forward_derivative = np.cross(angular_odom, forward_odom)
        yaw_rate = (forward_odom[0] * forward_derivative[1] - forward_odom[1] * forward_derivative[0]) / heading_norm_sq
        return np.array([heading @ linear_odom[:2], left @ linear_odom[:2], yaw_rate], dtype=np.float64)

    def payload(self) -> dict[str, Any]:
        return {
            "stamp_s": float(self.stamp_s),
            "epoch": self.epoch,
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "odom_from_base": self.odom_from_base.tolist(),
            "twist_base": self.twist_base.tolist(),
            "pose_odom": self.pose_odom.tolist(),
            "planar_twist": self.planar_twist.tolist(),
            "tracking": self.tracking,
            "source": self.source,
            "quality": self.quality,
        }

    @classmethod
    def from_payload(cls, value: dict[str, Any]) -> "RobotState":
        expected = {
            "stamp_s", "epoch", "frame_id", "child_frame_id", "odom_from_base", "twist_base",
            "pose_odom", "planar_twist", "tracking", "source", "quality",
        }
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError("RobotState payload must use the exact current schema")
        if value["frame_id"] != "odom" or value["child_frame_id"] != "base_link":
            raise ValueError("RobotState payload frames must be odom and base_link")
        state = cls(
            stamp_s=value["stamp_s"], epoch=value["epoch"], odom_from_base=value["odom_from_base"],
            twist_base=value["twist_base"], tracking=value["tracking"], source=value["source"],
            quality=value["quality"],
        )
        if not np.array_equal(np.asarray(value["pose_odom"], dtype=np.float64), state.pose_odom):
            raise ValueError("RobotState payload pose_odom does not match odom_from_base")
        if not np.array_equal(np.asarray(value["planar_twist"], dtype=np.float64), state.planar_twist):
            raise ValueError("RobotState payload planar_twist does not match twist_base")
        return state
