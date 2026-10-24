"""Trajectory planner configuration.

Primary pre-visual goal: seat on the gate course line (XT≈0), then hand off.
400 m is an INTERCEPT planning / speed-schedule hint, not a hard station.
FOLLOW trail standoff is a soft approach aid on the *live* gate course —
aim stays on-line and ahead of the drone.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TrajectoryConfig:
    # --- Frame / reference (local NED: x=North, y=East, z=Down) ---
    # Origin is typically ArduPilot home / SITL spherical_coordinates.
    home_lat_deg: float = -35.3632621
    home_lon_deg: float = 149.1652374
    home_alt_m: float = 10.0

    # --- Radar ---
    radar_rate_hz: float = 2.0
    radar_position_noise_std_m: float = 5.0
    # If radar omits speed/course, derive from successive fixes.

    # --- Kalman (constant-velocity) ---
    kf_process_noise_pos: float = 0.5
    kf_process_noise_vel: float = 1.0
    kf_measurement_noise_pos: float = 5.0

    # --- Prediction ---
    predict_horizon_s: float = 30.0
    predict_dt_s: float = 1.0
    # T_400 search horizon (long-range closing can be minutes, not 30 s).
    t_400_horizon_s: float = 600.0
    # Cap CV extrapolation used for INTERCEPT engagement geometry.
    # Large T_400 (minutes) must not place the engagement behind the drone.
    # FOLLOW finals use the live gate trail (no CV predict).
    engagement_predict_horizon_s: float = 60.0

    # --- Drone kinematics ---
    # Peak GUIDED horizontal speed far from the gate (schedule ceiling).
    max_speed_mps: float = 20.0
    # Soft cruise hint / docs; schedule peak is max_speed_mps.
    track_cruise_mps: float = 20.0
    # Baseline: match WP_ACC (parm / configure_guided_speed_limits).
    max_acceleration_mps2: float = 4.0
    max_deceleration_mps2: float = 4.0
    # GUIDED speed-command slew (applied every control tick).
    # Keep ≤ WP_ACC: 0.15 m/s every 50 ms ⇒ 3.0 m/s².
    speed_cmd_slew_up_mps2: float = 3.0
    speed_cmd_slew_dn_mps2: float = 3.0
    # Cap commanded speed at actual_gs + lead (Ardu still limited by WP_ACC).
    speed_cmd_lead_mps: float = 3.0
    # Min spacing / quantum for DO_CHANGE_SPEED (avoid MAVLink flood).
    speed_cmd_min_interval_s: float = 0.05
    speed_cmd_min_delta_mps: float = 0.15

    # --- 400 m deadline (INTERCEPT geometry + speed schedule end) ---
    deadline_range_m: float = 400.0
    # FOLLOW soft trail hint behind the gate (m). Not a hard station: if the
    # drone is already closer, shrink so the final stays ahead on the line.
    follow_standoff_m: float = 50.0
    # Smallest behind-gate trail when shrinking (m).
    follow_standoff_min_m: float = 15.0
    time_margin_s: float = 2.5
    # Do not trust target course below this speed (m/s).
    # Cold-start KF with vn≈0 falls back to LOS bearing; that makes FOLLOW
    # look shorter and wrongly win before velocity is observed.
    min_course_speed_mps: float = 2.0

    # --- Speed schedule floor near the gate (not a flight phase) ---
    close_approach_speed_mps: float = 10.0
    # Start decelerating from peak toward close_approach when
    # distance(drone, gate) falls below this (m). At deadline_range_m → floor.
    speed_decel_start_range_m: float = 1000.0
    # Hold peak speed while cross-track to target course exceeds this (m),
    # even if slant range is already in the decel band (FOLLOW join).
    speed_decel_max_xt_m: float = 80.0
    # XT peak boost only while range is still at least this (m).
    speed_xt_boost_min_range_m: float = 800.0
    # INTERCEPT catch-up when range is opening / not closing enough.
    # closing = −d(range)/dt (m/s); ≤ enter arms latch, ≥ exit + hold clears it.
    # While latched: speed ≥ min(peak, gate_speed + catchup_lead_mps).
    # lead ≤ 0 disables catch-up.
    catchup_enter_closing_mps: float = 0.0
    catchup_exit_closing_mps: float = 1.5
    catchup_hold_s: float = 1.5
    catchup_lead_mps: float = 5.0
    # Only arm catch-up at/above this range (m). 0 = any range.
    catchup_min_range_m: float = 100.0

    # --- Final geometry (v1) ---
    # Same altitude as target; collinear with target course (no lateral offset).
    final_lateral_offset_m: float = 0.0

    # --- Behavior selection costs (weights; lower candidate cost wins) ---
    # Prefer fast engagement, not shortest/slowest path.
    cost_travel_time: float = 1.0
    cost_path_length: float = 0.005
    cost_deadline_slack: float = 2.0
    cost_heading_mismatch: float = 0.5
    cost_final_error: float = 0.1
    # Final on the opposite side of the drone from the live target (retreat).
    cost_retreat_penalty: float = 1.0e6
    # Infeasible candidates get a large additive penalty instead of silent drop.
    cost_infeasible_penalty: float = 1.0e6
    # FOLLOW only when target aspect (course vs LOS) is at least this (deg).
    # Small aspect = closing head-on → INTERCEPT. Large = crossing → FOLLOW OK.
    follow_min_aspect_deg: float = 50.0
    # --- INTERCEPT visual-lock / centerline seat ---
    # Force allow_visual_lock at/under this range even if still off-line (m).
    intercept_visual_range_m: float = 100.0
    # Seated on line: |XT| ≤ this (m) and LOS heading error ≤ los deg.
    intercept_align_xt_m: float = 15.0
    intercept_align_los_deg: float = 15.0

    # --- FOLLOW visual-lock gate ---
    # Cross-track to target course must be ≤ this to count as aligned (m).
    follow_align_xt_m: float = 15.0
    # Drone must sit at least this far *behind* the gate along course (m).
    # Keep ≤ follow_standoff_m so the trail station still counts as aligned.
    follow_behind_min_m: float = 30.0
    # |drone_course − target_course| ≤ this (deg).
    follow_align_course_deg: float = 10.0
    # Stay ALIGN this long after geometry OK before CHASE / allow lock (s).
    # Short hold handed off to VISUAL while still sideways (diag 122116).
    follow_align_hold_s: float = 3.0
    # Cap GUIDED speed while far JOIN (m/s). Near the trail station, speed
    # blends down to gate + lead (adapts if the gate is fast). 0 = no far cap.
    follow_align_speed_mps: float = 20.0
    # When |XT| ≤ this and along-track is inside the brake window, blend toward
    # gate speed so we do not overrun a slow gate (diag 131602).
    follow_trail_brake_xt_m: float = 100.0
    # Start trail-speed blend this far behind the gate along course (m).
    follow_trail_brake_along_m: float = 150.0
    # FOLLOW trail / CHASE: command gate_speed + this (m/s), capped at peak.
    follow_trail_match_lead_mps: float = 5.0
    # Only aim along course û once |XT| is within this (m). Larger values
    # caused parallel flight at ~40 m offset (diag 120730). Until then aim
    # at a *nearby* on-centerline point ahead of the drone (INTERCEPT-style
    # XT null — never a distant trail that yields a shallow merge).
    follow_course_aim_xt_m: float = 12.0
    # On-line aim must stay at least this far ahead of the drone along û (m).
    # Also floors XT-null lookahead so the carrot never reverses.
    follow_aim_min_ahead_m: float = 40.0
    # Drop CHASE latch if cross-track exceeds align_xt * this factor.
    follow_chase_break_xt_factor: float = 3.0

    # --- VISUAL→TRAJ REALIGN (GPS/KF) ---
    # After bbox_lost: BRAKE (GUIDED v=0 → go along gate course, no WP),
    # then PATH carrot FOLLOW. Visual lock stays blocked until PATH + CHASE.
    realign_speed_mps: float = 10.0
    # INTERCEPT REALIGN (legacy path if still INTERCEPT): LOS course threshold.
    realign_course_deg: float = 15.0
    # INTERCEPT: hold aligned this long before clearing realign / allow lock (s).
    realign_hold_s: float = 2.0
    # BRAKE velocity: STOP then GO along gate course (not attitude).
    # Keep go ≤ realign_speed so STOP→GO does not oscillate.
    realign_brake_go_mps: float = 10.0
    realign_brake_climb_mps: float = 2.5
    realign_brake_exit_course_deg: float = 40.0
    realign_brake_max_s: float = 4.0
    # Skip BRAKE (no STOP/yaw) when gate is still ahead: LOS small and
    # along-track not past the gate by more than this (m).
    realign_ahead_los_deg: float = 55.0
    realign_ahead_along_m: float = 15.0
    # PATH aim yaw rate. Bind below a_lat/v at realign_speed (3.5/10 ≈ 20°/s).
    realign_carrot_rate_deg_s: float = 40.0
    # Tighter aim cone around live gate LOS while realigning (deg).
    realign_max_offset_from_los_deg: float = 8.0
    # GUIDED rabbit during realign PATH (m). 0 = use path_carrot_lookahead_m.
    realign_carrot_lookahead_m: float = 120.0
    # Lateral-accel cap while realigning PATH (overrides carrot_lat_accel_mps2).
    realign_carrot_lat_accel_mps2: float = 3.5
    # Hold heading / stay in STOP while sinking faster than this (m/s up).
    realign_arrest_vz_mps: float = 1.5
    # Ignore GUIDED retargets smaller than this while realigning PATH (m).
    realign_setpoint_retarget_min_m: float = 8.0
    # Last bbox |ang_x| at/above this (and ≥ |ang_y|) counts as a side exit (deg).
    realign_turn_lateral_deg: float = 8.0
    # |signed XT| at/above this (m) picks the toward-line turn when ang_x is weak.
    realign_turn_xt_m: float = 8.0
    # Fallback turn when last ang is vertical/unknown and XT is small: +1 right/CW.
    realign_turn_default_sign: float = 1.0
    # Unlock shortest-path slew once remaining heading error is below this (deg).
    realign_turn_unlock_deg: float = 35.0

    # --- Smoothing / hysteresis ---
    # Minimum time between INTERCEPT↔FOLLOW switches (s).
    # FOLLOW itself is sticky once entered (see BehaviorSelector).
    behavior_min_hold_s: float = 20.0
    # After hold, only switch if the other mode beats current cost by this.
    # Stops cost flicker on steady-course gates (diag showed ~5 s flips).
    behavior_switch_cost_margin: float = 30.0
    # Aspect band hysteresis (deg): harder to enter FOLLOW / leave FOLLOW.
    # Enter FOLLOW needs aspect ≥ follow_min + hyst; stay FOLLOW until
    # aspect < follow_min − hyst.
    behavior_aspect_hysteresis_deg: float = 15.0
    final_pos_blend: float = 0.1  # gentle EMA between radar cycles (not a freeze)
    # Horizontal jump above this snaps to the new final (skip EMA). Blocks a
    # one-frame KF velocity spike from poisoning last_final for minutes.
    final_pos_blend_snap_m: float = 500.0
    # Ignore GUIDED retargets smaller than this (stops pitch-chop from micro moves).
    setpoint_retarget_min_m: float = 30.0
    # Until visual lock: if gate alt < match_above, fly gate+clearance
    # (capped at match_above). Gate ≥ match_above → fly gate altitude.
    # e.g. gate 30→40, 20→30, 45→50, 55→55. Inside near_range, use
    # traj_alt_clearance_near_m instead (e.g. +5 below 1000 m).
    traj_alt_match_above_m: float = 50.0
    traj_alt_clearance_m: float = 10.0
    traj_alt_clearance_near_m: float = 5.0
    # Range at/under which near clearance applies (m). 0 → always far clearance.
    traj_alt_near_range_m: float = 1000.0

    # --- Path / carrot ---
    # v1: at most one intermediate + final.
    max_waypoints: int = 2
    # Course-line merge angle (deg). 90° = perpendicular L-join (harsh).
    # ~30° places the join further along-track so the turn is shallow when
    # there is still a long approach (e.g. 2 km).
    course_merge_angle_deg: float = 30.0
    # GUIDED position is a sliding carrot this far ahead of the drone
    # (rabbit). Never treated as a stop WP; always recomputed ahead.
    path_carrot_lookahead_m: float = 250.0
    # Optional brake vs path remaining near final (0 = fly through; gate-range only).
    path_carrot_end_taper_m: float = 0.0
    # Engagement ref is abandoned (aim→gate) inside this capture radius.
    carrot_ref_capture_m: float = 80.0
    # Clamp engagement aim inside this cone around live gate LOS (deg).
    carrot_max_offset_from_los_deg: float = 25.0
    # Soften aim heading changes (start snap / late yaw).
    carrot_max_rate_deg_s: float = 15.0
    carrot_lat_accel_mps2: float = 3.5
    # Skip intermediate WP this far out when building the carrot polyline.
    wp_flyby_radius_m: float = 120.0
    # Legacy join taper (unused when path_carrot_lookahead_m > 0).
    wp_approach_taper_m: float = 250.0
    wp_approach_min_speed_mps: float = 12.0
    # Rebuild INTERCEPT/FOLLOW path this often (s). Same window is used to
    # estimate gate course/speed from recent GPS fixes (KF still updates
    # every sample). Independent of GPS sample rate.
    path_retarget_interval_s: float = 5.0


@dataclass
class CostWeights:
    """Optional explicit cost bundle (mirrors TrajectoryConfig fields)."""

    travel_time: float = 1.0
    path_length: float = 0.005
    deadline_slack: float = 2.0
    heading_mismatch: float = 0.5
    final_error: float = 0.1
    retreat_penalty: float = 1.0e6
    infeasible_penalty: float = 1.0e6

    @classmethod
    def from_config(cls, cfg: TrajectoryConfig) -> CostWeights:
        return cls(
            travel_time=cfg.cost_travel_time,
            path_length=cfg.cost_path_length,
            deadline_slack=cfg.cost_deadline_slack,
            heading_mismatch=cfg.cost_heading_mismatch,
            final_error=cfg.cost_final_error,
            retreat_penalty=cfg.cost_retreat_penalty,
            infeasible_penalty=cfg.cost_infeasible_penalty,
        )


def default_config(**overrides: float | int) -> TrajectoryConfig:
    cfg = TrajectoryConfig()
    for key, value in overrides.items():
        if not hasattr(cfg, key):
            raise AttributeError(f"Unknown TrajectoryConfig field: {key}")
        setattr(cfg, key, value)
    return cfg
