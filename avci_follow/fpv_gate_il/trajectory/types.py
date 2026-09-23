"""Shared trajectory types (NED frame unless noted).

NED convention used throughout the planner:
  x / north  — metres North of home
  y / east   — metres East of home
  z / down   — metres Down (altitude_up = -z for display)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


class BehaviorMode(str, Enum):
    INTERCEPT = "intercept"
    FOLLOW = "follow"


class FollowPhase(str, Enum):
    """FOLLOW-only sub-phases until visual lock is allowed."""

    NONE = "none"  # INTERCEPT / not applicable
    JOIN = "join"  # getting onto course behind the gate
    ALIGN = "align"  # geometrically aligned; holding before chase
    CHASE = "chase"  # aligned + committed; bbox lock OK


class RealignPhase(str, Enum):
    """VISUAL→TRAJ realign: velocity brake/yaw, then GUIDED FOLLOW path."""

    NONE = "none"
    BRAKE = "brake"  # v=0 bleed + yaw/GO along gate course (no position WP)
    PATH = "path"  # GUIDED carrot FOLLOW


class MissionPhase(str, Enum):
    """High-level engagement phase."""

    # Receding INTERCEPT/FOLLOW path; GUIDED follows a never-reached carrot
    # until the session hands off to visual (HCAir).
    TRACK_PLAN = "track_plan"


@dataclass
class NedPosition:
    north_m: float
    east_m: float
    down_m: float

    def as_array(self) -> np.ndarray:
        return np.array([self.north_m, self.east_m, self.down_m], dtype=float)

    @classmethod
    def from_array(cls, xyz: np.ndarray) -> NedPosition:
        return cls(float(xyz[0]), float(xyz[1]), float(xyz[2]))

    @property
    def alt_up_m(self) -> float:
        return -self.down_m


@dataclass
class NedVelocity:
    vn_mps: float
    ve_mps: float
    vd_mps: float

    def as_array(self) -> np.ndarray:
        return np.array([self.vn_mps, self.ve_mps, self.vd_mps], dtype=float)

    @classmethod
    def from_array(cls, v: np.ndarray) -> NedVelocity:
        return cls(float(v[0]), float(v[1]), float(v[2]))

    @property
    def horizontal_speed_mps(self) -> float:
        return float(np.hypot(self.vn_mps, self.ve_mps))

    @property
    def course_rad(self) -> float:
        """Ground course from North toward East (atan2(ve, vn))."""
        return float(np.arctan2(self.ve_mps, self.vn_mps))


@dataclass
class TargetState:
    """Filtered kinematic state of the gate/target."""

    position: NedPosition
    velocity: NedVelocity
    timestamp_s: float
    speed_mps: float = 0.0
    course_rad: float = 0.0
    # Optional: nose direction if radar provides it (not used for v1 pathing).
    heading_rad: Optional[float] = None
    # Reserved for constant-acceleration / turn models later.
    acceleration: Optional[np.ndarray] = None  # shape (3,)
    turn_rate_rps: Optional[float] = None
    covariance: Optional[np.ndarray] = None  # shape (6, 6) if KF provides it


@dataclass
class DroneState:
    position: NedPosition
    velocity: NedVelocity
    timestamp_s: float
    speed_mps: float = 0.0
    heading_rad: float = 0.0  # yaw / nose


@dataclass
class Waypoint:
    position: NedPosition
    desired_speed_mps: float
    estimated_arrival_time_s: float


@dataclass
class Path:
    waypoints: list[Waypoint] = field(default_factory=list)
    behavior: BehaviorMode = BehaviorMode.FOLLOW
    feasible: bool = False
    travel_time_s: float = float("inf")
    t_400_s: float = float("inf")
    final_position: Optional[NedPosition] = None
    cost: float = float("inf")
    notes: str = ""


@dataclass
class BehaviorCandidate:
    behavior: BehaviorMode
    path: Path
    cost: float
    metrics: dict[str, float] = field(default_factory=dict)


@dataclass
class PlannerOutput:
    phase: MissionPhase
    path: Optional[Path]
    behavior: Optional[BehaviorMode]
    t_400_s: Optional[float]
    distance_to_target_m: float
    target: Optional[TargetState] = None
    # Unit NE rabbit aim (filled by apply_planner_output / pipeline).
    carrot_dir_n: float = 1.0
    carrot_dir_e: float = 0.0
    # Visual lock: FOLLOW needs CHASE; INTERCEPT needs seat or visual_range.
    follow_phase: FollowPhase = FollowPhase.NONE
    allow_visual_lock: bool = True
    xt_m: float = 0.0
    along_track_m: float = 0.0
    course_err_deg: float = 0.0
    # True while VISUAL→TRAJ realign is active (speed/aim/lock gate).
    realign_active: bool = False
    # Latched yaw side while realigning: +1 right/CW, -1 left/CCW, 0 unlocked.
    realign_turn_sign: float = 0.0
    # True when dive arrest holds heading (no turn) this tick.
    realign_arrest: bool = False
    realign_phase: RealignPhase = RealignPhase.NONE
