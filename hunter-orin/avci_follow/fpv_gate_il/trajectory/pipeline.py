"""End-to-end trajectory pipeline (receding horizon).

Radar → KF → prediction → T_400 → INTERCEPT/FOLLOW → path.

Path rebuilds every ``path_retarget_interval_s`` using gate course/speed
from GPS fixes in that same window (KF still updates every sample).
GUIDED follows a sliding carrot on the held path until the session hands
off to visual / HCAir. Primary pre-visual goal: seat on the gate course
line. INTERCEPT opens visual when seated (or by
``intercept_visual_range_m``). FOLLOW gates visual until behind-course
ALIGN/CHASE. After VISUAL bbox loss, ``soft_reset_guidance(realign=True)``
slows and re-points at the gate via GPS/KF before allowing lock again.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
from typing import Optional

from fpv_gate_il.trajectory.behaviors import (
    BehaviorSelector,
    along_track_m,
    closing_aspect_deg,
    final_retreats_from_target,
    follow_geometry_ok,
    intercept_centerline_final,
    intercept_geometry_ok,
    intercept_realign_ok,
    los_heading_error_deg,
    point_along_track_m,
    signed_cross_track_to_course_m,
    target_course_reliable,
)
from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.coords import distance_m
from fpv_gate_il.trajectory.kalman import KalmanTargetTracker
from fpv_gate_il.trajectory.mav_bridge import (
    follow_aim_direction_ne,
    intercept_aim_direction_ne,
    realign_brake_ready,
    realign_dive_arresting,
    resolve_realign_turn_sign,
)
from fpv_gate_il.trajectory.path_planner import PathPlanner
from fpv_gate_il.trajectory.predict import TrajectoryPredictor
from fpv_gate_il.trajectory.radar import (
    RadarMeasurement,
    derive_speed_course_from_fixes,
    velocity_from_fix_window,
)
from fpv_gate_il.trajectory.speed_planner import SpeedPlanner
from fpv_gate_il.trajectory.types import (
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
)


@dataclass
class TrajectoryPipeline:
    config: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    tracker: KalmanTargetTracker = field(init=False)
    predictor: TrajectoryPredictor = field(init=False)
    behaviors: BehaviorSelector = field(init=False)
    path_planner: PathPlanner = field(init=False)
    speed_planner: SpeedPlanner = field(init=False)

    # Smoothing / hysteresis state (not a geometry freeze)
    last_behavior: Optional[BehaviorMode] = None
    last_behavior_switch_s: Optional[float] = None
    last_final: Optional[NedPosition] = None
    last_path: Optional[Path] = None
    _plan_cycles: int = 0
    _last_replan_s: Optional[float] = None
    _last_raw: Optional[RadarMeasurement] = field(default=None, repr=False)
    # Timed GPS fixes for windowed course/speed at path rebuild.
    # Entries: (timestamp_s, north_m, east_m, down_m).
    _gps_window: deque[tuple[float, float, float, float]] = field(
        default_factory=deque, repr=False
    )
    # FOLLOW lock-gate state
    _follow_phase: FollowPhase = FollowPhase.NONE
    _follow_aligned_since_s: Optional[float] = None
    _follow_entered_s: Optional[float] = None
    _follow_chase_latched: bool = False
    # VISUAL→TRAJ realign (GPS/KF); set by soft_reset_guidance(realign=True).
    _realign_active: bool = False
    _realign_aligned_since_s: Optional[float] = None
    _realign_ang_x: Optional[float] = None
    _realign_ang_y: Optional[float] = None
    _realign_turn_sign: Optional[float] = None
    _realign_phase: RealignPhase = RealignPhase.NONE
    _realign_brake_since_s: Optional[float] = None

    def __post_init__(self) -> None:
        self.tracker = KalmanTargetTracker(self.config)
        self.predictor = TrajectoryPredictor(self.config)
        self.behaviors = BehaviorSelector(self.config)
        self.path_planner = PathPlanner(self.config)
        self.speed_planner = SpeedPlanner(self.config)

    def reset(self) -> None:
        self.tracker.reset()
        self.last_behavior = None
        self.last_behavior_switch_s = None
        self.last_final = None
        self.last_path = None
        self._plan_cycles = 0
        self._last_replan_s = None
        self._last_raw = None
        self._gps_window.clear()
        self._reset_follow_gate()
        self._realign_active = False
        self._realign_aligned_since_s = None
        self._realign_ang_x = None
        self._realign_ang_y = None
        self._realign_turn_sign = None
        self._realign_phase = RealignPhase.NONE
        self._realign_brake_since_s = None

    def soft_reset_guidance(
        self,
        *,
        realign: bool = False,
        last_ang_x: float | None = None,
        last_ang_y: float | None = None,
    ) -> None:
        """VISUAL→TRAJ re-entry: rebuild path/carrot; keep KF.

        When ``realign=True`` (after bbox_lost): force FOLLOW, start BRAKE
        (GUIDED velocity stop/go, no WP), block visual lock until PATH +
        FOLLOW CHASE geometry is seated.

        ``last_ang_x`` / ``last_ang_y`` latch the realign turn side.
        ``realign=False`` keeps the current behavior commitment.
        """
        self.last_final = None
        self.last_path = None
        self._last_replan_s = None
        self._reset_follow_gate()
        self._realign_active = bool(realign)
        self._realign_aligned_since_s = None
        self._realign_ang_x = None
        self._realign_ang_y = None
        self._realign_turn_sign = None
        self._realign_phase = RealignPhase.NONE
        self._realign_brake_since_s = None
        if realign:
            # After track-lost, re-engage from behind on the course line.
            self.last_behavior = BehaviorMode.FOLLOW
            self.last_behavior_switch_s = None
            self._follow_entered_s = None
            self._follow_phase = FollowPhase.JOIN
            self._realign_ang_x = last_ang_x
            self._realign_ang_y = last_ang_y
            self._realign_phase = RealignPhase.BRAKE

    def _reset_follow_gate(self) -> None:
        self._follow_phase = FollowPhase.NONE
        self._follow_aligned_since_s = None
        self._follow_entered_s = None
        self._follow_chase_latched = False

    def _append_gps_window(self, measurement: RadarMeasurement) -> None:
        cfg = self.config
        ned = measurement.to_ned(
            home_lat_deg=cfg.home_lat_deg,
            home_lon_deg=cfg.home_lon_deg,
            home_alt_m=cfg.home_alt_m,
        )
        t = float(measurement.timestamp_s)
        self._gps_window.append((t, ned.north_m, ned.east_m, ned.down_m))
        window_s = max(0.0, float(cfg.path_retarget_interval_s))
        cutoff = t - window_s
        while self._gps_window and self._gps_window[0][0] < cutoff:
            self._gps_window.popleft()

    def _plan_target_from_window(self, target: TargetState) -> TargetState:
        """Override KF velocity with window LS fit when the window is usable."""
        fit = velocity_from_fix_window(list(self._gps_window))
        if fit is None:
            return target
        vn, ve, vd = fit
        vel = NedVelocity(vn_mps=vn, ve_mps=ve, vd_mps=vd)
        if not target_course_reliable(
            replace(target, velocity=vel, speed_mps=vel.horizontal_speed_mps),
            float(self.config.min_course_speed_mps),
        ):
            return target
        return replace(
            target,
            velocity=vel,
            speed_mps=vel.horizontal_speed_mps,
            course_rad=vel.course_rad,
        )

    def ingest_radar(self, measurement: RadarMeasurement) -> TargetState:
        """KF-only update (visual phase). Does not rebuild GUIDED path/carrot."""
        meas = self._maybe_enrich_speed_course(measurement)
        target = self.tracker.update(meas)
        self._append_gps_window(meas)
        self._last_raw = meas
        return target

    def update(
        self,
        measurement: RadarMeasurement,
        drone: DroneState,
    ) -> PlannerOutput:
        """Radar sample: always update KF; rebuild path on time interval."""
        target = self.ingest_radar(measurement)

        distance = self.range_drone_target(drone, target.position)
        now_s = float(measurement.timestamp_s)
        self._plan_cycles += 1

        interval_s = max(0.0, float(self.config.path_retarget_interval_s))
        due = self._last_replan_s is None or (now_s - float(self._last_replan_s)) >= interval_s
        force = self._force_replan(drone, target)
        if force or self.last_path is None or due:
            plan_target = self._plan_target_from_window(target)
            out = self._track_plan(drone, plan_target, distance, now_s=now_s)
            self._last_replan_s = now_s
            return self._apply_realign(
                self._with_follow_gate(out, drone, target, now_s=now_s),
                drone,
                target,
                now_s=now_s,
            )

        # Hold engagement geometry; refresh range / aim from live KF + drone.
        t_400 = self.predictor.estimate_t_400(target, drone)
        cn, ce = self._aim_dirs(
            drone,
            target.position,
            self.last_final,
            target=target,
            behavior=self.last_behavior,
            path=self.last_path,
        )
        out = PlannerOutput(
            phase=MissionPhase.TRACK_PLAN,
            path=self.last_path,
            behavior=self.last_behavior,
            t_400_s=t_400,
            distance_to_target_m=distance,
            target=target,
            carrot_dir_n=cn,
            carrot_dir_e=ce,
        )
        return self._apply_realign(
            self._with_follow_gate(out, drone, target, now_s=now_s),
            drone,
            target,
            now_s=now_s,
        )

    def _with_follow_gate(
        self,
        out: PlannerOutput,
        drone: DroneState,
        target: TargetState,
        *,
        now_s: float,
    ) -> PlannerOutput:
        """Annotate FOLLOW JOIN/ALIGN/CHASE and allow_visual_lock."""
        if out.behavior is not BehaviorMode.FOLLOW:
            self._reset_follow_gate()
            out.follow_phase = FollowPhase.NONE
            seated, xt, along, los_err = intercept_geometry_ok(
                drone, target, self.config
            )
            out.xt_m = xt
            out.along_track_m = along
            out.course_err_deg = los_err
            # INTERCEPT: seat on line before visual; force lock by visual_range.
            range_m = float(out.distance_to_target_m)
            vis_r = float(self.config.intercept_visual_range_m)
            out.allow_visual_lock = bool(seated or range_m <= vis_r)
            return out

        aligned, xt, along, cerr = follow_geometry_ok(drone, target, self.config)
        out.xt_m = xt
        out.along_track_m = along
        out.course_err_deg = cerr

        if self._follow_entered_s is None:
            self._follow_entered_s = now_s

        # Break a bad CHASE latch if geometry blows out.
        break_xt = float(self.config.follow_align_xt_m) * float(
            self.config.follow_chase_break_xt_factor
        )
        if self._follow_chase_latched and (
            xt > break_xt or along > -0.5 * float(self.config.follow_behind_min_m)
        ):
            self._follow_chase_latched = False
            self._follow_aligned_since_s = None

        if self._follow_chase_latched:
            # Stay CHASE only while geometry remains chase-worthy.
            if not aligned:
                self._follow_chase_latched = False
                self._follow_aligned_since_s = None
                self._follow_phase = FollowPhase.JOIN
                out.follow_phase = FollowPhase.JOIN
                out.allow_visual_lock = False
                return out
            self._follow_phase = FollowPhase.CHASE
            out.follow_phase = FollowPhase.CHASE
            out.allow_visual_lock = True
            return out

        if not aligned:
            self._follow_aligned_since_s = None
            self._follow_phase = FollowPhase.JOIN
            out.follow_phase = FollowPhase.JOIN
            out.allow_visual_lock = False
            return out

        if self._follow_aligned_since_s is None:
            self._follow_aligned_since_s = now_s
        hold = float(self.config.follow_align_hold_s)
        if (now_s - float(self._follow_aligned_since_s)) >= hold:
            self._follow_chase_latched = True
            self._follow_phase = FollowPhase.CHASE
            out.follow_phase = FollowPhase.CHASE
            out.allow_visual_lock = True
        else:
            self._follow_phase = FollowPhase.ALIGN
            out.follow_phase = FollowPhase.ALIGN
            out.allow_visual_lock = False
        return out

    def _apply_realign(
        self,
        out: PlannerOutput,
        drone: DroneState,
        target: TargetState,
        *,
        now_s: float,
    ) -> PlannerOutput:
        """After bbox_lost: BRAKE then PATH; lock only after PATH + CHASE.

        BRAKE never opens visual lock even if a bbox streak is ready — wait
        until the FOLLOW path is seated (CHASE).
        """
        out.realign_active = bool(self._realign_active)
        if not self._realign_active:
            out.realign_turn_sign = 0.0
            out.realign_arrest = False
            out.realign_phase = RealignPhase.NONE
            return out

        if self._realign_turn_sign is None:
            xt_s = signed_cross_track_to_course_m(drone, target)
            self._realign_turn_sign = resolve_realign_turn_sign(
                ang_x_deg=self._realign_ang_x,
                ang_y_deg=self._realign_ang_y,
                xt_signed_m=xt_s,
                config=self.config,
            )
        out.realign_turn_sign = float(self._realign_turn_sign)
        out.realign_arrest = realign_dive_arresting(drone, self.config)

        # Always block visual while realigning; clear only on PATH + CHASE.
        out.allow_visual_lock = False

        if self._realign_phase is RealignPhase.NONE:
            self._realign_phase = RealignPhase.BRAKE

        # Gate still ahead and not diving → skip STOP/yaw BRAKE; PATH carrot.
        if (
            self._realign_phase is RealignPhase.BRAKE
            and not out.realign_arrest
            and self._gate_still_ahead(drone, target)
        ):
            self._realign_phase = RealignPhase.PATH
            self._realign_brake_since_s = None

        if self._realign_phase is RealignPhase.BRAKE:
            if self._realign_brake_since_s is None:
                self._realign_brake_since_s = now_s
            out.realign_phase = RealignPhase.BRAKE
            if realign_brake_ready(
                drone,
                target,
                self.config,
                now_s=now_s,
                brake_since_s=self._realign_brake_since_s,
            ):
                self._realign_phase = RealignPhase.PATH
                out.realign_phase = RealignPhase.PATH
            else:
                # Stay BRAKE: never hand off to visual mid-slowdown/turn.
                return out

        out.realign_phase = RealignPhase.PATH

        if out.behavior is BehaviorMode.FOLLOW:
            # FOLLOW already requires JOIN→ALIGN→CHASE; clear realign on CHASE.
            if self._follow_chase_latched:
                self._realign_active = False
                self._realign_aligned_since_s = None
                self._realign_phase = RealignPhase.NONE
                self._realign_brake_since_s = None
                out.realign_active = False
                out.realign_arrest = False
                out.realign_phase = RealignPhase.NONE
                out.allow_visual_lock = True
            return out

        # INTERCEPT: point at live gate LOS, then short hold.
        ok, err = intercept_realign_ok(drone, target, self.config)
        out.course_err_deg = float(err)
        if not ok:
            self._realign_aligned_since_s = None
            return out
        if self._realign_aligned_since_s is None:
            self._realign_aligned_since_s = now_s
        hold = float(self.config.realign_hold_s)
        if (now_s - float(self._realign_aligned_since_s)) >= hold:
            self._realign_active = False
            self._realign_aligned_since_s = None
            self._realign_phase = RealignPhase.NONE
            self._realign_brake_since_s = None
            out.realign_active = False
            out.realign_arrest = False
            out.realign_phase = RealignPhase.NONE
            out.allow_visual_lock = True
        return out

    def _gate_still_ahead(self, drone: DroneState, target: TargetState) -> bool:
        """True when lost-track but gate is still roughly in front — no 180 yaw.

        Requires both: not past the gate on course, and nose/track aimed near
        the live gate LOS. Overfly (along ≫ 0) or pointed away → BRAKE.
        """
        along = along_track_m(drone, target)
        if along > float(self.config.realign_ahead_along_m):
            return False
        los_err = los_heading_error_deg(drone, target.position)
        return los_err <= float(self.config.realign_ahead_los_deg)

    def _force_replan(self, drone: DroneState, target: TargetState) -> bool:
        """True when holding the previous path would be unsafe / obsolete."""
        if self._realign_active:
            return True
        if self.last_path is None:
            return True
        notes = self.last_path.notes or ""
        # Cold-start / past-engagement must track live gate / centerline.
        if (
            "failsafe_los_to_target" in notes
            or "past_engagement_los" in notes
            or "past_engagement_centerline" in notes
        ):
            return True
        if not target_course_reliable(target, float(self.config.min_course_speed_mps)):
            return True
        if self.last_final is not None and (
            final_retreats_from_target(drone, target, self.last_final)
            or self._past_engagement(drone, self.last_final)
        ):
            return True
        return False

    def _maybe_enrich_speed_course(self, measurement: RadarMeasurement) -> RadarMeasurement:
        if measurement.ground_speed_mps is not None and measurement.course_rad is not None:
            return measurement
        if self._last_raw is None:
            return measurement
        cfg = self.config
        try:
            speed, course = derive_speed_course_from_fixes(
                self._last_raw,
                measurement,
                home_lat_deg=cfg.home_lat_deg,
                home_lon_deg=cfg.home_lon_deg,
                home_alt_m=cfg.home_alt_m,
            )
        except ValueError:
            return measurement
        return replace(
            measurement,
            ground_speed_mps=measurement.ground_speed_mps
            if measurement.ground_speed_mps is not None
            else speed,
            course_rad=measurement.course_rad if measurement.course_rad is not None else course,
        )

    def _direct_to_target_path(
        self,
        drone: DroneState,
        target: TargetState,
        planning_t: float,
    ) -> Path:
        """Fail-safe: single WP at live target — no INTERCEPT/FOLLOW geometry."""
        path = self.path_planner.build_path(
            drone,
            target.position,
            behavior=BehaviorMode.INTERCEPT,
            t_400_s=planning_t,
            intermediate=None,
        )
        path.notes = (path.notes + ";" if path.notes else "") + "failsafe_los_to_target"
        return path

    def _aim_dirs(
        self,
        drone: DroneState,
        gate: NedPosition,
        ref: NedPosition | None,
        *,
        target: TargetState | None = None,
        behavior: BehaviorMode | None = None,
        path: Path | None = None,
    ) -> tuple[float, float]:
        max_off = float(self.config.carrot_max_offset_from_los_deg)
        if self._realign_active:
            max_off = min(
                max_off, float(self.config.realign_max_offset_from_los_deg)
            )
            # Prefer live gate LOS while recovering from visual loss.
            ref = None
        if behavior is BehaviorMode.FOLLOW and target is not None:
            join = None
            # Realign: never chase a stale join WP (behind → spin). Live
            # centerline / LOS only until PATH+CHASE reseats.
            if (
                not self._realign_active
                and path is not None
                and len(path.waypoints) >= 2
            ):
                join = path.waypoints[0].position
            return follow_aim_direction_ne(
                drone.position,
                gate,
                ref,
                target,
                course_aim_xt_m=float(self.config.follow_course_aim_xt_m),
                standoff_m=float(self.config.follow_standoff_m),
                min_ahead_m=float(self.config.follow_aim_min_ahead_m),
                capture_m=float(self.config.carrot_ref_capture_m),
                max_offset_from_los_deg=max_off,
                join_pos=join,
                xt_null_lookahead_m=max(
                    80.0, float(self.config.path_carrot_lookahead_m) * 0.3
                ),
            )
        return intercept_aim_direction_ne(
            drone.position,
            gate,
            ref,
            target,
            align_xt_m=float(self.config.intercept_align_xt_m),
            capture_m=float(self.config.carrot_ref_capture_m),
            max_offset_from_los_deg=max_off,
            lookahead_m=max(80.0, float(self.config.path_carrot_lookahead_m) * 0.3),
        )

    def _past_engagement(self, drone: DroneState, final: NedPosition | None) -> bool:
        """True when engagement ref is captured or behind the drone→gate half-plane."""
        if final is None:
            return False
        capture = float(self.config.carrot_ref_capture_m)
        return distance_m(drone.position, final) <= capture

    def _track_plan(
        self,
        drone: DroneState,
        target: TargetState,
        distance: float,
        *,
        now_s: float,
    ) -> PlannerOutput:
        t_400 = self.predictor.estimate_t_400(target, drone)
        if t_400 is not None:
            planning_t = float(t_400)
        else:
            planning_t = float(self.config.engagement_predict_horizon_s)

        course_ok = target_course_reliable(
            target, float(self.config.min_course_speed_mps)
        )
        # Cold-start: no course yet. If already in FOLLOW, keep FOLLOW via
        # live-gate LOS rather than silently opening INTERCEPT visual lock.
        if not course_ok:
            if self.last_behavior is BehaviorMode.FOLLOW:
                path = self._direct_to_target_path(drone, target, planning_t)
                path.notes = (
                    (path.notes + ";" if path.notes else "") + "follow_hold_los_no_course"
                )
                path.behavior = BehaviorMode.FOLLOW
                self.last_path = path
                self.last_final = target.position
                cn, ce = self._aim_dirs(
                    drone,
                    target.position,
                    None,
                    target=target,
                    behavior=BehaviorMode.FOLLOW,
                    path=path,
                )
                return PlannerOutput(
                    phase=MissionPhase.TRACK_PLAN,
                    path=path,
                    behavior=BehaviorMode.FOLLOW,
                    t_400_s=t_400,
                    distance_to_target_m=distance,
                    target=target,
                    carrot_dir_n=cn,
                    carrot_dir_e=ce,
                )
            self.last_final = None
            path = self._direct_to_target_path(drone, target, planning_t)
            self.last_path = path
            self.last_behavior = BehaviorMode.INTERCEPT
            cn, ce = self._aim_dirs(drone, target.position, None)
            return PlannerOutput(
                phase=MissionPhase.TRACK_PLAN,
                path=path,
                behavior=BehaviorMode.INTERCEPT,
                t_400_s=t_400,
                distance_to_target_m=distance,
                target=target,
                carrot_dir_n=cn,
                carrot_dir_e=ce,
            )

        raw_intercept = self.behaviors.propose_intercept_final(target, drone, planning_t)

        along_now = along_track_m(drone, target)
        # Behind the gate: never keep INTERCEPT / past-engagement punch path.
        if along_now < 0.0 and self.last_behavior is not BehaviorMode.FOLLOW:
            self.last_behavior = BehaviorMode.FOLLOW
            self.last_behavior_switch_s = now_s
            self._reset_follow_gate()
            self._follow_entered_s = now_s
            self._follow_phase = FollowPhase.JOIN

        # Past INTERCEPT engagement → stay on the course line (null XT), then
        # punch to gate. Only while still ahead / abreast — never when behind.
        if (
            self.last_behavior is not BehaviorMode.FOLLOW
            and along_now >= 0.0
            and (
                final_retreats_from_target(drone, target, raw_intercept)
                or self._past_engagement(drone, raw_intercept)
                or (
                    self.last_behavior is BehaviorMode.INTERCEPT
                    and self.last_final is not None
                    and (
                        final_retreats_from_target(drone, target, self.last_final)
                        or self._past_engagement(drone, self.last_final)
                    )
                )
            )
        ):
            centerline = intercept_centerline_final(drone, target, self.config)
            path = self.path_planner.build_path(
                drone,
                centerline,
                behavior=BehaviorMode.INTERCEPT,
                t_400_s=planning_t,
                intermediate=None,
            )
            path.notes = (
                (path.notes + ";" if path.notes else "") + "past_engagement_centerline"
            )
            self.last_final = centerline
            self.last_path = path
            self.last_behavior = BehaviorMode.INTERCEPT
            self._reset_follow_gate()
            cn, ce = self._aim_dirs(
                drone,
                target.position,
                centerline,
                target=target,
                behavior=BehaviorMode.INTERCEPT,
                path=path,
            )
            return PlannerOutput(
                phase=MissionPhase.TRACK_PLAN,
                path=path,
                behavior=BehaviorMode.INTERCEPT,
                t_400_s=t_400,
                distance_to_target_m=distance,
                target=target,
                carrot_dir_n=cn,
                carrot_dir_e=ce,
            )

        # Once FOLLOW is committed, stay FOLLOW. If the trail station ends up
        # "behind" the drone (overrun → retreat metric), or a poisoned blend
        # leaves the final ahead of the live gate while the raw trail is
        # behind, re-anchor to the live trail — do NOT flip to INTERCEPT
        # (that opened visual lock sideways in traj_diag_20260909_122116
        # while cerr≈40°).
        if self.last_behavior is BehaviorMode.FOLLOW:
            raw_follow = self.behaviors.propose_follow_final(target, drone, planning_t)
            blended = self.path_planner.blend_final(self.last_final, raw_follow)
            follow = self.behaviors.build_candidate(
                BehaviorMode.FOLLOW,
                target,
                drone,
                planning_t,
                path_planner=self.path_planner,
                blended_final=blended,
            )
            reanchor = float(follow.metrics.get("retreat", 0.0)) >= 0.5
            if not reanchor and blended is not None:
                # Blended ahead of live gate while raw trail is behind →
                # KF/EMA poison (traj_diag_20260915_135737).
                raw_along = point_along_track_m(raw_follow, target)
                blend_along = point_along_track_m(blended, target)
                if blend_along > 0.0 and raw_along <= 0.0:
                    reanchor = True
            if reanchor:
                follow = self.behaviors.build_candidate(
                    BehaviorMode.FOLLOW,
                    target,
                    drone,
                    planning_t,
                    path_planner=self.path_planner,
                    blended_final=raw_follow,
                )
                if follow.path.notes:
                    follow.path.notes += ";follow_reanchor"
                else:
                    follow.path.notes = "follow_reanchor"
            self.last_final = follow.path.final_position
            self.last_path = follow.path
            self.last_behavior = BehaviorMode.FOLLOW
            cn, ce = self._aim_dirs(
                drone,
                target.position,
                follow.path.final_position,
                target=target,
                behavior=BehaviorMode.FOLLOW,
                path=follow.path,
            )
            return PlannerOutput(
                phase=MissionPhase.TRACK_PLAN,
                path=follow.path,
                behavior=BehaviorMode.FOLLOW,
                t_400_s=t_400,
                distance_to_target_m=distance,
                target=target,
                carrot_dir_n=cn,
                carrot_dir_e=ce,
            )

        raw_follow = self.behaviors.propose_follow_final(target, drone, planning_t)
        if self.last_behavior is BehaviorMode.INTERCEPT:
            blended_intercept = self.path_planner.blend_final(self.last_final, raw_intercept)
            blended_follow = raw_follow
        elif self.last_behavior is BehaviorMode.FOLLOW:
            blended_intercept = raw_intercept
            blended_follow = self.path_planner.blend_final(self.last_final, raw_follow)
        else:
            blended_intercept = raw_intercept
            blended_follow = raw_follow

        intercept = self.behaviors.build_candidate(
            BehaviorMode.INTERCEPT,
            target,
            drone,
            planning_t,
            path_planner=self.path_planner,
            blended_final=blended_intercept,
        )
        follow = self.behaviors.build_candidate(
            BehaviorMode.FOLLOW,
            target,
            drone,
            planning_t,
            path_planner=self.path_planner,
            blended_final=blended_follow,
        )
        aspect = closing_aspect_deg(target, drone)
        chosen = self.behaviors.select(
            intercept,
            follow,
            previous=self.last_behavior,
            previous_switch_time_s=self.last_behavior_switch_s,
            now_s=now_s,
            course_reliable=True,
            aspect_deg=aspect,
        )

        if self.last_behavior is None or chosen.behavior is not self.last_behavior:
            self.last_behavior_switch_s = now_s
            if chosen.behavior is BehaviorMode.FOLLOW:
                # Fresh FOLLOW entry — restart align timer.
                self._follow_entered_s = now_s
                self._follow_aligned_since_s = None
                self._follow_chase_latched = False
                self._follow_phase = FollowPhase.JOIN
            elif chosen.behavior is BehaviorMode.INTERCEPT:
                self._reset_follow_gate()
        self.last_behavior = chosen.behavior
        self.last_final = chosen.path.final_position
        self.last_path = chosen.path
        cn, ce = self._aim_dirs(
            drone,
            target.position,
            chosen.path.final_position,
            target=target,
            behavior=chosen.behavior,
            path=chosen.path,
        )

        return PlannerOutput(
            phase=MissionPhase.TRACK_PLAN,
            path=chosen.path,
            behavior=chosen.behavior,
            t_400_s=t_400,
            distance_to_target_m=distance,
            target=target,
            carrot_dir_n=cn,
            carrot_dir_e=ce,
        )

    @staticmethod
    def range_drone_target(drone: DroneState, target_pos: NedPosition) -> float:
        return distance_m(drone.position, target_pos)
