"""Modular radar → Kalman → prediction → path/speed planner stack.

Local frame is NED (x=North, y=East, z=Down). INTERCEPT/FOLLOW choose a
course-aligned engagement path each radar cycle; GUIDED follows a sliding
carrot until visual handoff (HCAir).
"""

from fpv_gate_il.trajectory.behaviors import BehaviorSelector
from fpv_gate_il.trajectory.config import CostWeights, TrajectoryConfig, default_config
from fpv_gate_il.trajectory.coords import (
    distance_m,
    enu_xyz_to_ned,
    horizontal_distance_m,
    latlonalt_to_local,
    local_to_latlonalt,
    ned_to_enu_xyz,
)
from fpv_gate_il.trajectory.kalman import KalmanTargetTracker
from fpv_gate_il.trajectory.path_planner import PathPlanner
from fpv_gate_il.trajectory.mav_bridge import (
    CarrotDirSlew,
    CatchupSpeedLatch,
    apply_planner_output,
    closing_speed_mps,
    direction_carrot_setpoint,
    drone_dict_to_state,
    fly_aim_direction_ne,
    follow_aim_direction_ne,
    gate_sample_to_radar,
    path_carrot_setpoint,
    path_remaining_length_m,
    traj_hold_alt_m,
)
from fpv_gate_il.trajectory.pipeline import TrajectoryPipeline
from fpv_gate_il.trajectory.predict import TrajectoryPredictor, TrajectorySample
from fpv_gate_il.trajectory.radar import (
    RadarMeasurement,
    add_gaussian_position_noise,
    derive_speed_course_from_fixes,
    velocity_from_fix_window,
)
from fpv_gate_il.trajectory.speed_planner import (
    SpeedPlanner,
    calculate_minimum_travel_time,
    cruise_speed_for_deadline,
)
from fpv_gate_il.trajectory.types import (
    BehaviorCandidate,
    BehaviorMode,
    DroneState,
    FollowPhase,
    MissionPhase,
    NedPosition,
    NedVelocity,
    Path,
    PlannerOutput,
    RealignPhase,
    TargetState,
    Waypoint,
)

__all__ = [
    "BehaviorCandidate",
    "BehaviorMode",
    "BehaviorSelector",
    "CarrotDirSlew",
    "CatchupSpeedLatch",
    "CostWeights",
    "FollowPhase",
    "DroneState",
    "KalmanTargetTracker",
    "MissionPhase",
    "NedPosition",
    "NedVelocity",
    "Path",
    "PathPlanner",
    "PlannerOutput",
    "RadarMeasurement",
    "RealignPhase",
    "SpeedPlanner",
    "add_gaussian_position_noise",
    "closing_speed_mps",
    "derive_speed_course_from_fixes",
    "velocity_from_fix_window",
    "TargetState",
    "TrajectoryConfig",
    "TrajectoryPipeline",
    "TrajectoryPredictor",
    "TrajectorySample",
    "Waypoint",
    "apply_planner_output",
    "calculate_minimum_travel_time",
    "cruise_speed_for_deadline",
    "default_config",
    "direction_carrot_setpoint",
    "distance_m",
    "drone_dict_to_state",
    "enu_xyz_to_ned",
    "fly_aim_direction_ne",
    "follow_aim_direction_ne",
    "gate_sample_to_radar",
    "horizontal_distance_m",
    "latlonalt_to_local",
    "local_to_latlonalt",
    "ned_to_enu_xyz",
    "path_carrot_setpoint",
    "path_remaining_length_m",
    "traj_hold_alt_m",
]
