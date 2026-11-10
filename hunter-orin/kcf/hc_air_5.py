#!/usr/bin/env python3
"""
Hard-coded teacher for air-to-air gates — speed / guidance channel split.

GPS-free: vision + attitude/gyro + baro vertical velocity (EKF vz only).
No horizontal vel_ned / GPS / metric gate width.

  Pitch  — closure governor → pitch_des (scale-free bbox rate).
  Thrust — vertical LOS pursuit (elev from ang_y + pitch → vz_des) scaled by
           adaptive speed estimate (GPS-free substitute for v_fwd scaling).
  Yaw    — horizontal aim (ang_x → 0).
  Roll   — wings level + crab cancellation (derotated LOS az drift).

Phases: lost → cruise → dive. pitch_des governor ramps when closure_filt
is strongly negative (gate receding) and convergence gates pass. Vertical
thrust: adaptive pursuit speed × tan(elev) + PI on vz; lost keeps decaying vz_des.
"""

from __future__ import annotations

import math
import time

import numpy as np

from fpv_gate_il.features import camera_pitch_from_obs
from fpv_gate_il.teacher.hc_air_acro import HCAirAcroConverter
from fpv_gate_il.teacher.hc_air_config import cfg_float, load_hc_air_config
from fpv_gate_il.teacher.los_geometry import G_MPS2, LosRateSample, LosRateTracker, wrap_pi

_OUTPUT_BODY_RATE = "body_rate"
_OUTPUT_ACRO_RC = "acro_rc"


class HCAir:
    """
    Hard-coded air teacher. Internal control always uses body rates (rad/s).

    Output mode (config ``output.mode`` or ``output_mode`` kwarg):

    * ``body_rate`` (default) — action is SET_ATTITUDE_TARGET body rates + thrust:

        [0] roll_rate_cmd    — rad/s
        [1] pitch_rate_cmd   — rad/s, negative = nose down
        [2] thrust_cmd       — collective (vertical)
        [3] yaw_rate_cmd     — rad/s

    * ``acro_rc`` — same inner loop; returned action is integer RC PWM (µs):

        [0] roll_pwm, [1] pitch_pwm, [2] throttle_pwm, [3] yaw_pwm

      Send via MAVLink RC_CHANNELS_OVERRIDE (not camera_bridge).
    """

    @classmethod
    def action_to_body_rates(cls, action: np.ndarray) -> tuple[float, float, float, float]:
        """Passthrough when ``output.mode=body_rate``. Not valid for acro PWM actions."""
        a = np.asarray(action, dtype=np.float64)
        return (
            float(a[0]),
            float(a[1]),
            float(a[3]),
            float(a[2]),
        )

    @classmethod
    def suggested_control_backend(
        cls,
        config_path: str | None = None,
        *,
        output_mode: str | None = None,
    ) -> str:
        mode = output_mode or cls._cfg(config_path).get("output", {}).get(
            "mode", _OUTPUT_BODY_RATE,
        )
        return "rc_override" if mode == _OUTPUT_ACRO_RC else "set_attitude_target"

    @classmethod
    def _cfg(cls, config_path: str | None = None) -> dict:
        return load_hc_air_config(config_path)

    @classmethod
    def action_channel_limits(
        cls,
        config_path: str | None = None,
        *,
        output_mode: str | None = None,
    ) -> tuple[float, float, float, float, float, float]:
        cfg = cls._cfg(config_path)
        mode = output_mode or cfg.get("output", {}).get("mode", _OUTPUT_BODY_RATE)
        if mode == _OUTPUT_ACRO_RC:
            acro = cfg["acro"]
            pwm_min = float(min(
                acro["rc"]["roll"]["min_pwm"],
                acro["rc"]["pitch"]["min_pwm"],
                acro["rc"]["throttle"]["min_pwm"],
                acro["rc"]["yaw"]["min_pwm"],
            ))
            pwm_max = float(max(
                acro["rc"]["roll"]["max_pwm"],
                acro["rc"]["pitch"]["max_pwm"],
                acro["rc"]["throttle"]["max_pwm"],
                acro["rc"]["yaw"]["max_pwm"],
            ))
            return (pwm_min, pwm_min, pwm_max, pwm_min, pwm_min, pwm_max)
        al = cfg["action_limits"]
        plat = cfg["platform"]
        return (
            float(al["roll_rad_s"]),
            float(al["pitch_down_rad_s"]),
            float(al["pitch_up_rad_s"]),
            float(al["yaw_rad_s"]),
            float(plat["thrust_cmd_min"]),
            float(plat["thrust_cmd_max"]),
        )

    def channel_limits(self) -> tuple[float, float, float, float, float, float]:
        return self.action_channel_limits(
            self._config_path,
            output_mode=self.output_mode,
        )

    @classmethod
    def teacher_tilt_comp(
        cls,
        roll_rad: float,
        pitch_rad: float,
        hover_thrust: float,
        config_path: str | None = None,
    ) -> float:
        cos_min = cfg_float(cls._cfg(config_path), "platform.tilt_cos_min", 0.5)
        cos_tilt = math.cos(pitch_rad) * math.cos(roll_rad)
        return (1.0 / max(cos_tilt, cos_min) - 1.0) * hover_thrust

    def _apply_config(self, cfg: dict) -> None:
        plat = cfg["platform"]
        cam = cfg["camera"]
        yaw = cfg["yaw"]
        pa = cfg["pitch_attitude"]
        pg = cfg["pitch_governor"]
        roll = cfg["roll"]
        vt = cfg["vertical_thrust"]
        va = cfg["vertical_adapt"]
        dive = cfg["dive_phase"]
        lost = cfg["lost_behavior"]
        dyn = cfg["dynamics"]
        sm = cfg["smoothing"]

        self.hover_thrust = float(plat["hover_thrust"])
        self.ff_open_loop_scale = float(plat["ff_open_loop_scale"])
        self.thrust_cmd_min = float(plat["thrust_cmd_min"])
        self.thrust_cmd_max = float(plat["thrust_cmd_max"])
        self.tilt_cos_min = float(plat["tilt_cos_min"])

        self.camera_pitch_default_deg = float(cam["pitch_default_deg"])
        self.camera_pitch_deg = self.camera_pitch_default_deg
        self.cam_axis_offset_deg = float(cam["axis_offset_deg"])
        self.cam_pitch_transfer = float(cam["pitch_transfer"])

        self.ang_x_deadband_deg = float(yaw["ang_x_deadband_deg"])
        self.kp_yaw = float(yaw["kp_rad_s_per_deg"])
        self.kd_yaw_body = float(yaw["kd_body_rad_s_per_dps"])
        self.max_yaw = float(yaw["max_cmd_rad_s"])

        self.pitch_des_cruise_deg = float(pa["cruise_des_deg"])
        self.pitch_des_min_deg = float(pa["des_min_deg"])
        self.pitch_des_max_deg = float(pa["des_max_deg"])
        self.pitch_assist_climb_err = float(pa["assist_climb_err_mps"])
        self.pitch_assist_sink_err = float(pa["assist_sink_err_mps"])
        self.k_pitch_assist = float(pa["assist_k_deg_per_mps"])
        self.pitch_assist_max_deg = float(pa["assist_max_deg"])
        self.pitch_des_min_relaxed_deg = float(pa["des_min_relaxed_deg"])
        self.pitch_des_fallback_deg = float(pa["fallback_des_deg"])
        self.kp_pitch_spd = float(pa["kp_rad_s_per_deg"])
        self.kd_pitch_body = float(pa["kd_body_rad_s_per_dps"])
        self.max_pitch = float(pa["max_cmd_rad_s"])

        self.closure_target = float(pg["closure_target_per_s"])
        self.closure_filt_alpha = float(pg["closure_filt_alpha"])
        self.k_pitch_ramp_up = float(pg["ramp_up_deg_s_per_unit"])
        self.k_pitch_ramp_down = float(pg["ramp_down_deg_s_per_unit"])
        self.pitch_des_accel_max = float(pg["des_accel_max_deg_s"])
        self.governor_warmup_s = float(pg["warmup_s"])
        self.governor_aim_max_deg = float(pg["aim_max_deg"])
        self.governor_chase_aim_max_deg = float(pg["chase_aim_max_deg"])
        self.governor_pitch_err_max_deg = float(pg["pitch_err_max_deg"])
        self.governor_vz_err_max = float(pg["vz_err_max_mps"])
        self.governor_chase_vert_err_max = float(pg["chase_vz_err_max_mps"])
        self.governor_recede_threshold = float(pg["recede_threshold_per_s"])
        self.governor_no_slowdown_size = float(pg["no_slowdown_size"])
        self.lock_reset_after_s = float(pg["lock_reset_after_s"])
        self.k_pitch_vert_brake = float(pg["vert_brake_deg_s"])
        self.lost_pitch_des_decay_after_s = float(pg["lost_des_decay_after_s"])
        self.k_pitch_lost_decay = float(pg["lost_des_decay_deg_s"])

        self.kp_roll_level = float(roll["kp_level_rad_s_per_deg"])
        self.kp_crab_los = float(roll["kp_crab_los_rad_s_per_dps"])
        self.los_az_filt_alpha = float(roll["los_az_filt_alpha"])
        self.los_az_lost_decay = float(roll["los_az_lost_decay"])
        self.max_roll = float(roll["max_cmd_rad_s"])
        self._los_tracker = LosRateTracker(
            alpha=self.los_az_filt_alpha,
            lost_decay=self.los_az_lost_decay,
        )
        guide = cfg.get("guidance", {})
        self.nav_n = float(guide.get("nav_constant", 4.0))
        self.bank_max_deg = float(guide.get("bank_max_deg", 22.0))
        self.dive_depression_deg = float(guide.get("dive_depression_deg", 25.0))
        self.alt_floor_m = float(guide.get("alt_floor_m", 8.0))
        self.alt_climb_m = float(guide.get("alt_climb_m", 4.0))
        self.alt_floor_dive_m = float(guide.get("alt_floor_dive_m", 2.0))
        self.alt_climb_dive_m = float(guide.get("alt_climb_dive_m", 1.0))
        self.k_alt_hold = float(guide.get("k_alt_hold", 0.4))
        self.alt_hold_vz_max = float(guide.get("alt_hold_vz_max_mps", 2.0))
        self.yaw_soft_deg = float(guide.get("yaw_soft_deg", 12.0))
        self.yaw_soft_scale = float(guide.get("yaw_soft_scale", 0.85))
        self.g_mps2 = G_MPS2
        # Bank per LOS rate (deg / (deg/s)). Same law in cruise and dive:
        # nulling the rate while closing is the collision course.
        self.k_bank = float(guide.get("k_bank_deg_per_dps", 6.0))
        self.bank_slew_deg_s = float(guide.get("bank_slew_deg_s", 12.0))
        self.bank_warmup_s = float(guide.get("match_warmup_s", 0.8))
        # Gyro attitude leaks toward the EKF this slowly. The EKF tilt drifts
        # under sustained acceleration when it has no velocity reference.
        self.att_ekf_tau_s = float(guide.get("att_ekf_tau_s", 60.0))
        self.vz_slew_mps2 = float(guide.get("vz_slew_mps2", 2.5))
        # Elevation rate to vertical speed at the lock-on size. Divided by the
        # bbox size ratio, since the same rate is a smaller velocity error once
        # the gate is closer. 4.2 ≈ 240 m × π/180.
        self.k_el_p = float(guide.get("k_el_p_mps_per_dps", 4.2))
        self.k_el_i = float(guide.get("k_el_i_mps_per_dps_s", 1.0))
        # Hold this depression. Steeper than it, command a positive elevation
        # rate so the altitude gap closes while the gate is still far.
        self.dep_target_deg = float(guide.get("dep_target_deg", 4.5))
        self.k_el_shallow = float(guide.get("k_el_shallow_dps_per_deg", 0.20))
        self.el_shallow_rate_max = float(guide.get("el_shallow_rate_max_dps", 1.2))
        # Extra descent only once the gate nears the bottom of the image.
        # Half of the 80° camera's vertical FOV is ~25°.
        self.ang_y_hold_deg = float(guide.get("ang_y_hold_deg", 12.0))
        self.ang_y_guard_deg = float(guide.get("ang_y_guard_deg", 18.0))
        self.k_frame_vz = float(guide.get("k_frame_vz_mps_per_deg", 0.25))
        self.frame_sink_max = float(guide.get("frame_sink_max_mps", 4.0))
        self.frame_climb_max = float(guide.get("frame_climb_max_mps", 0.6))

        self.kp_vz = float(vt["kp"])
        self.ki_vz = float(vt["ki"])
        self.k_pursuit_n = float(vt["k_pursuit"])
        self.v_pursuit_ref_mps = float(vt["v_pursuit_ref_mps"])
        self.vz_int_max = float(vt["vz_int_max"])
        self.vz_des_max = float(vt["vz_des_max_mps"])
        self.t_fb_max = float(vt["t_fb_max"])
        self.lost_vz_decay_tau_s = float(vt["lost_vz_decay_tau_s"])

        self.k_v_adapt_per_deg = float(va["k_per_deg"])
        self.v_adapt_max = float(va["v_max_mps"])
        self.v_adapt_decay_tau_s = float(va["decay_tau_s"])
        self.elev_lag_sign_deadband_deg = float(va["lag_sign_deadband_deg"])
        self.elev_drift_filt_alpha = float(va["elev_drift_filt_alpha"])
        self.elev_drift_step_max_deg = float(va["elev_drift_step_max_deg"])

        self.dive_size_ratio = float(dive["size_ratio"])
        self.dive_aim_max_deg = float(dive["aim_max_deg"])
        self.dive_ang_y_deg = float(dive.get("ang_y_deg", 20.0))
        self.dive_ang_y_margin_deg = float(dive.get("ang_y_margin_deg", 2.0))
        self.bbox_ref_window_s = float(dive.get("ref_window_s", 0.5))

        self.lost_yaw_scan = float(lost["yaw_scan_rad_s"])
        self.lost_pitch_target_deg = float(lost["pitch_target_deg"])
        self.lost_pitch_kp = float(lost["pitch_kp_rad_s_per_deg"])
        self.lost_yaw_kp_scale = float(lost["yaw_kp_scale"])
        self.lost_roll_cmd_scale = float(lost["roll_cmd_scale"])
        self.lost_pitch_cmd_scale = float(lost["pitch_cmd_scale"])

        self.bbox_rate_alpha = float(dyn["bbox_rate_alpha"])
        self.bbox_rate_lost_decay = float(dyn["bbox_rate_lost_decay"])
        self.default_dt_s = float(dyn["default_dt_s"])

        self.alpha_roll = float(sm["alpha_roll"])
        self.alpha_pitch = float(sm["alpha_pitch"])
        self.alpha_throttle = float(sm["alpha_throttle"])
        self.alpha_yaw = float(sm["alpha_yaw"])
        self.roll_rate_limit = float(sm["roll_rate_limit_rad_s"])
        self.pitch_rate_limit = float(sm["pitch_rate_limit_rad_s"])
        self.yaw_rate_limit = float(sm["yaw_rate_limit_rad_s"])
        self.throttle_rate_limit = float(sm["throttle_rate_limit_per_s"])

    def __init__(
        self,
        config_path: str | None = None,
        *,
        output_mode: str | None = None,
        **kwargs,
    ):
        if kwargs:
            raise TypeError(f"HCAir accepts no keyword overrides, got {sorted(kwargs)}")
        self._config_path = config_path
        cfg = self._cfg(config_path)
        self._apply_config(cfg)
        self.output_mode = str(
            output_mode or cfg.get("output", {}).get("mode", _OUTPUT_BODY_RATE)
        )
        if self.output_mode not in (_OUTPUT_BODY_RATE, _OUTPUT_ACRO_RC):
            raise ValueError(
                f"output_mode must be {_OUTPUT_BODY_RATE!r} or {_OUTPUT_ACRO_RC!r}, "
                f"got {self.output_mode!r}"
            )
        self._acro_converter: HCAirAcroConverter | None = None
        if self.output_mode == _OUTPUT_ACRO_RC:
            self._acro_converter = HCAirAcroConverter(cfg)

        self.prev_ang_x: float | None = None
        self.prev_ang_y: float | None = None
        self.prev_bbox_size = 0.0
        self._bbox_init_size = 0.0
        self._bbox_ref_sum = 0.0
        self._bbox_ref_n = 0
        self._bbox_ref_t = 0.0
        self._clear_ang_y_reference()
        self._bank_t = 0.0
        self.bbox_rate = 0.0
        self.d_ang_x = 0.0
        self.d_ang_y = 0.0
        self.current_phase = "lost"
        self._dive_committed = False
        self._pitch_des_gov = float(self.pitch_des_cruise_deg)
        self._closure_filt = 0.0
        self._lost_t = 0.0
        self._lock_t = 0.0
        self._vz_int = 0.0
        self._vz_des_lost_hold = 0.0
        self._governor_active = 0.0
        self._chase_receding = 0.0
        self._los_sample = LosRateSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)
        self._alt_ref: float | None = None
        self._alt_hold = 0.0
        self._roll_des_deg = 0.0
        self._vz_slew = 0.0
        self._vz_el_int = 0.0
        self._el_track_t = 0.0
        self._el_rate_des = 0.0
        self._guide_pitch_deg = 0.0
        self._a_close_mps2 = 0.0
        self._elev_prev: float | None = None
        self._v_adapt = 0.0
        self._d_elev_filt = 0.0
        self._att_ready = False
        self._att_rad = np.zeros(3, dtype=np.float64)

        self.prev_action = np.array([0.0, 0.0, self.hover_thrust, 0.0], dtype=np.float64)
        self.step_count = 0
        self.last_debug: dict = {}
        self.dt = self.default_dt_s
        self.elapsed_t = 0.0
        self._last_frame_id = None
        self._wall_t_prev = None
        self._last_output = None

    def try_load_acro_from_vehicle(self, master) -> str:
        """Upgrade ACRO rate→PWM mapping from FC params; yaml remains fallback."""
        if self._acro_converter is None:
            return "n/a"
        return self._acro_converter.try_load_from_vehicle(master)

    @staticmethod
    def _deadband(value: float, deadband: float) -> float:
        v = float(value)
        db = abs(float(deadband))
        if abs(v) <= db:
            return 0.0
        return v - math.copysign(db, v)

    @staticmethod
    def _parse_privileged(privileged) -> tuple[bool, float, float]:
        """Returns (ok, altitude_m, vz_up_mps). Only vertical EKF velocity is used."""
        if not privileged or not privileged.get("valid", True):
            return False, 0.0, 0.0
        alt = float(privileged.get("altitude", 0.0))
        vel = np.asarray(privileged.get("vel_ned", (0.0, 0.0, 0.0)), dtype=np.float64)
        vz_up = -float(vel[2])
        return True, alt, vz_up

    def _tilt_ff(self, roll_deg: float, pitch_deg: float) -> tuple[float, float]:
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        cos_tilt = max(math.cos(pitch_rad) * math.cos(roll_rad), self.tilt_cos_min)
        tilt_comp = (1.0 / cos_tilt - 1.0) * self.hover_thrust
        t_ff = self.hover_thrust / cos_tilt
        return float(t_ff), float(tilt_comp)

    def _gate_elevation_deg(
        self,
        pitch_deg: float,
        ang_y: float,
        camera_pitch_deg: float | None = None,
    ) -> float:
        del camera_pitch_deg
        return (
            float(self.cam_axis_offset_deg)
            + self.cam_pitch_transfer * float(pitch_deg)
            - float(ang_y)
        )

    def _elev_lag_sign(self, elev_deg: float) -> float:
        if abs(float(elev_deg)) <= self.elev_lag_sign_deadband_deg:
            return 0.0
        return math.copysign(1.0, float(elev_deg))

    def _update_v_adapt(
        self,
        elev_deg: float,
        bbox_valid: float,
        phase: str,
        dt: float,
    ) -> None:
        """Raise pursuit speed when pitch-derotated elevation drifts away (GPS-free)."""
        if bbox_valid < 0.5:
            decay = math.exp(-float(dt) / max(self.lost_vz_decay_tau_s, 1e-3))
            self._v_adapt *= decay
            self._elev_prev = None
            self._d_elev_filt = 0.0
            return
        if phase == "dive":
            self._elev_prev = float(elev_deg)
            return
        if self._elev_prev is None:
            self._elev_prev = float(elev_deg)
            return
        d_elev = float(elev_deg) - float(self._elev_prev)
        step_max = float(self.elev_drift_step_max_deg)
        d_elev = float(np.clip(d_elev, -step_max, step_max))
        alpha = float(self.elev_drift_filt_alpha)
        self._d_elev_filt = alpha * d_elev + (1.0 - alpha) * self._d_elev_filt
        lag = self._d_elev_filt * self._elev_lag_sign(elev_deg)
        if lag > 0.0:
            self._v_adapt += self.k_v_adapt_per_deg * lag
        else:
            decay = math.exp(-float(dt) / max(self.v_adapt_decay_tau_s, 1e-3))
            self._v_adapt *= decay
        self._v_adapt = float(np.clip(self._v_adapt, 0.0, self.v_adapt_max))
        self._elev_prev = float(elev_deg)

    def _estimate_vz_des(
        self,
        *,
        ang_y: float,
        pitch_deg: float,
        camera_pitch_deg: float,
    ) -> float:
        elev_deg = self._gate_elevation_deg(pitch_deg, ang_y, camera_pitch_deg)
        # tan() flips sign past ±90°, which would command a climb when the
        # gate is almost underneath the drone.
        elev_deg = float(np.clip(elev_deg, -80.0, 80.0))
        v_eff = float(self.v_pursuit_ref_mps) + float(self._v_adapt)
        vz_des = (
            float(self.k_pursuit_n)
            * v_eff
            * math.tan(math.radians(elev_deg))
        )
        return float(np.clip(vz_des, -self.vz_des_max, self.vz_des_max))

    def _governor_gates_ok(
        self,
        *,
        ang_x: float,
        pitch_deg: float,
        priv_ok: bool,
        receding: bool,
    ) -> bool:
        if not priv_ok:
            return False
        if self._lock_t < self.governor_warmup_s:
            return False
        aim_lim = (
            self.governor_chase_aim_max_deg if receding else self.governor_aim_max_deg
        )
        if abs(float(ang_x)) > aim_lim:
            return False
        pitch_err = float(self._pitch_des_gov) - float(pitch_deg)
        if abs(pitch_err) > self.governor_pitch_err_max_deg:
            return False
        return True

    def _update_pitch_des_governor(
        self,
        *,
        bbox_valid: float,
        bbox_size: float,
        phase: str,
        dt: float,
        ang_x: float,
        ang_y: float,
        pitch_deg: float,
        vz_up: float,
        priv_ok: bool,
        camera_pitch_deg: float,
    ) -> None:
        self._governor_active = 0.0
        self._chase_receding = 0.0
        if phase == "dive":
            return

        if bbox_valid < 0.5:
            self._lost_t += dt
            if self._lost_t >= self.lock_reset_after_s:
                self._lock_t = 0.0
                self._bank_t = 0.0
                self._clear_ang_y_reference()
            if self._lost_t >= self.lost_pitch_des_decay_after_s:
                self._pitch_des_gov = max(
                    self.pitch_des_cruise_deg,
                    float(self._pitch_des_gov) + self.k_pitch_lost_decay * dt,
                )
            return

        self._lost_t = 0.0
        self._lock_t += dt
        if bbox_size <= 0.0:
            return

        closure_raw = float(self.bbox_rate) / max(float(bbox_size), 1e-6)
        alpha = float(self.closure_filt_alpha)
        self._closure_filt = alpha * closure_raw + (1.0 - alpha) * self._closure_filt

        vz_des_est = self._estimate_vz_des(
            ang_y=ang_y,
            pitch_deg=pitch_deg,
            camera_pitch_deg=camera_pitch_deg,
        )
        receding = float(self._closure_filt) < self.governor_recede_threshold
        self._chase_receding = 1.0 if receding else 0.0
        vz_err_lim = (
            self.governor_chase_vert_err_max if receding else self.governor_vz_err_max
        )
        vert_ok = abs(float(vz_des_est) - float(vz_up)) <= vz_err_lim

        if (
            not vert_ok
            and not receding
            and float(self._pitch_des_gov) < self.pitch_des_cruise_deg
        ):
            self._pitch_des_gov = max(
                self.pitch_des_cruise_deg,
                float(self._pitch_des_gov) + self.k_pitch_vert_brake * dt,
            )

        gates_ok = self._governor_gates_ok(
            ang_x=ang_x,
            pitch_deg=pitch_deg,
            priv_ok=priv_ok,
            receding=receding,
        )

        err = self.closure_target - float(self._closure_filt)
        if err > 0.0 and gates_ok and vert_ok:
            dp = min(self.k_pitch_ramp_up * err, self.pitch_des_accel_max)
            self._pitch_des_gov = float(self._pitch_des_gov) - dp * dt
            self._governor_active = 1.0
        elif err < 0.0 and float(bbox_size) < self.governor_no_slowdown_size:
            self._pitch_des_gov = (
                float(self._pitch_des_gov) - self.k_pitch_ramp_down * err * dt
            )
            self._pitch_des_gov = max(
                float(self.pitch_des_cruise_deg), float(self._pitch_des_gov),
            )

        self._pitch_des_gov = float(np.clip(
            self._pitch_des_gov, self.pitch_des_min_deg, self.pitch_des_max_deg,
        ))

    def _pitch_att_cmd(
        self,
        *,
        pitch_deg: float,
        pitch_rate_dps: float,
        priv_ok: bool,
        vz_up: float = 0.0,
        vz_des_est: float | None = None,
        pitch_override_deg: float | None = None,
    ) -> tuple[float, float]:
        """Returns pitch_rate_cmd, pitch_des_deg."""
        if pitch_override_deg is not None:
            pitch_des = float(pitch_override_deg)
            p_min = float(self.pitch_des_min_deg)
            p_max = float(self.pitch_des_max_deg)
            pitch_des = float(np.clip(pitch_des, p_min, p_max))
        elif priv_ok:
            pitch_des = float(self._pitch_des_gov)
            p_min = float(self.pitch_des_min_deg)
            p_max = float(self.pitch_des_max_deg)
            if vz_des_est is not None:
                climb_excess = float(vz_up) - float(vz_des_est)
                sink_excess = float(vz_des_est) - float(vz_up)
                if climb_excess > self.pitch_assist_climb_err:
                    p_max = -min(
                        self.k_pitch_assist
                        * (climb_excess - self.pitch_assist_climb_err),
                        self.pitch_assist_max_deg,
                    )
                if sink_excess > self.pitch_assist_sink_err:
                    p_min = float(self.pitch_des_min_relaxed_deg)
            pitch_des = float(np.clip(pitch_des, p_min, p_max))
        else:
            pitch_des = float(self.pitch_des_fallback_deg)

        pitch_err = pitch_des - float(pitch_deg)
        pitch_cmd = (
            self.kp_pitch_spd * pitch_err
            - self.kd_pitch_body * float(pitch_rate_dps)
        )
        pitch_cmd = float(np.clip(pitch_cmd, -self.max_pitch, self.max_pitch))
        return pitch_cmd, pitch_des

    def _thrust_vertical(
        self,
        *,
        phase: str,
        bbox_valid: float,
        ang_y: float,
        roll_deg: float,
        pitch_deg: float,
        priv_ok: bool,
        vz_up: float,
        dt: float,
        camera_pitch_deg: float,
        alt_m: float = 0.0,
        depression_deg: float = 0.0,
        size_ratio: float = 1.0,
    ) -> tuple[float, float, float, float, float, float]:
        """
        Returns thrust, tilt_comp, t_ff, t_fb, vz_des, elev_deg.
        """
        t_ff, tilt_comp = self._tilt_ff(roll_deg, pitch_deg)
        elev_deg = self._gate_elevation_deg(pitch_deg, ang_y, camera_pitch_deg)
        t_fb = 0.0
        vz_des = 0.0

        if bbox_valid > 0.5 and priv_ok:
            vz_des = self._vertical_vz_des(
                self._estimate_vz_des(
                    ang_y=ang_y,
                    pitch_deg=pitch_deg,
                    camera_pitch_deg=camera_pitch_deg,
                ),
                alt_m=alt_m,
                priv_ok=True,
                depression_deg=depression_deg,
                phase=phase,
                ang_y=ang_y,
                bbox_valid=bbox_valid,
                size_ratio=size_ratio,
            )
            self._vz_des_lost_hold = float(vz_des)
            vz_err = float(vz_des) - float(vz_up)
            self._vz_int += self.ki_vz * vz_err * dt
            self._vz_int = float(np.clip(
                self._vz_int, -self.vz_int_max, self.vz_int_max,
            ))
            t_fb_unsat = self.kp_vz * vz_err + float(self._vz_int)
            t_fb = float(np.clip(t_fb_unsat, -self.t_fb_max, self.t_fb_max))
            if abs(t_fb - t_fb_unsat) > 1e-9:
                self._vz_int = float(t_fb - self.kp_vz * vz_err)
        elif priv_ok:
            decay = math.exp(-float(dt) / max(self.lost_vz_decay_tau_s, 1e-3))
            self._vz_des_lost_hold *= decay
            vz_des = self._vertical_vz_des(
                float(self._vz_des_lost_hold),
                alt_m=alt_m,
                priv_ok=True,
                depression_deg=0.0,
                phase="lost",
                ang_y=0.0,
                bbox_valid=0.0,
            )
            if abs(vz_des) > 1e-6:
                vz_err = float(vz_des) - float(vz_up)
                t_fb = float(np.clip(
                    self.kp_vz * vz_err + float(self._vz_int),
                    -self.t_fb_max,
                    self.t_fb_max,
                ))
        elif not priv_ok:
            t_ff *= float(self.ff_open_loop_scale)
            self._vz_int = 0.0

        thrust = float(np.clip(
            t_ff + t_fb,
            self.thrust_cmd_min,
            self.thrust_cmd_max,
        ))
        return thrust, tilt_comp, t_ff, t_fb, vz_des, elev_deg

    def _size_ratio(self, bbox_size: float) -> float:
        return float(bbox_size) / max(self._bbox_init_size, 1e-6)

    def _update_frame_dynamics(
        self,
        ang_x: float,
        ang_y: float,
        bbox_size: float,
        bbox_valid: float,
        inv_dt: float,
    ) -> None:
        if bbox_valid < 0.5:
            self.bbox_rate *= self.bbox_rate_lost_decay
            self.d_ang_x = 0.0
            self.d_ang_y = 0.0
            return
        if self.prev_ang_x is None:
            self.prev_ang_x = ang_x
            self.prev_ang_y = ang_y
            self.prev_bbox_size = bbox_size
            self.d_ang_x = 0.0
            self.d_ang_y = 0.0
            return
        self.d_ang_x = ang_x - self.prev_ang_x
        self.d_ang_y = ang_y - self.prev_ang_y
        raw = (bbox_size - self.prev_bbox_size) * inv_dt
        self.bbox_rate = (
            self.bbox_rate_alpha * raw + (1.0 - self.bbox_rate_alpha) * self.bbox_rate
        )
        self.prev_ang_x = ang_x
        self.prev_ang_y = ang_y
        self.prev_bbox_size = bbox_size

    def _select_phase(
        self, bbox_valid: float, bbox_size: float, ang_x: float, ang_y: float,
    ) -> str:
        if bbox_valid < 0.5:
            self._dive_committed = False
            return "lost"
        if self._dive_committed:
            return "dive"
        ratio = self._size_ratio(bbox_size)
        # ang_y stays near the frame-keep hold, so it is not an aim gate.
        # A gate that slides down past both the absolute edge and its lock
        # angle dives anyway: that is the vertical FOV limit. A gate that is
        # already low at lock (ground gate, upward camera) waits for growth.
        grown = (
            ratio + 1e-6 >= self.dive_size_ratio
            and abs(float(ang_x)) <= self.dive_aim_max_deg
        )
        if grown or self._ang_y_leaves_frame(ang_y):
            self._dive_committed = True
            return "dive"
        return "cruise"

    def _yaw_cmd(self, ang_x: float, yaw_rate_dps: float) -> tuple[float, float]:
        """Keep the gate in frame. Small errors yaw slowly so the nose does not
        outrun the velocity vector; errors near the FOV edge use full gain.
        """
        ax = self._deadband(ang_x, self.ang_x_deadband_deg)
        scale = self.yaw_soft_scale if abs(float(ang_x)) < self.yaw_soft_deg else 1.0
        yaw_cmd = scale * self.kp_yaw * ax - self.kd_yaw_body * float(yaw_rate_dps)
        yaw_cmd = float(np.clip(yaw_cmd, -self.max_yaw, self.max_yaw))
        return yaw_cmd, ax

    def _note_bbox_reference(self, bbox_size: float, bbox_valid: float, dt: float) -> None:
        """Average the lock-on size. One frame of a ~16 px box is too noisy."""
        if bbox_valid < 0.5 or bbox_size <= 0.0:
            return
        if self._bbox_ref_t >= self.bbox_ref_window_s and self._bbox_init_size > 0.0:
            return
        self._bbox_ref_sum += float(bbox_size)
        self._bbox_ref_n += 1
        self._bbox_ref_t += max(float(dt), 0.0)
        self._bbox_init_size = self._bbox_ref_sum / self._bbox_ref_n

    def _clear_ang_y_reference(self) -> None:
        self._ang_y_ref_sum = 0.0
        self._ang_y_ref_n = 0
        self._ang_y_ref_t = 0.0
        self._ang_y_ref = None

    def _note_ang_y_reference(self, ang_y: float, bbox_valid: float, dt: float) -> None:
        """Average ang_y over the lock window. The dive edge is relative to it."""
        if bbox_valid < 0.5:
            return
        if self._ang_y_ref_t >= self.bbox_ref_window_s and self._ang_y_ref is not None:
            return
        self._ang_y_ref_sum += float(ang_y)
        self._ang_y_ref_n += 1
        self._ang_y_ref_t += max(float(dt), 0.0)
        self._ang_y_ref = self._ang_y_ref_sum / self._ang_y_ref_n

    def _ang_y_leaves_frame(self, ang_y: float) -> bool:
        """True once the gate drops below the lock angle and the absolute edge."""
        if self._ang_y_ref is None or self._ang_y_ref_t < self.bbox_ref_window_s:
            return False
        limit = max(
            self.dive_ang_y_deg,
            self._ang_y_ref + self.dive_ang_y_margin_deg,
        )
        return float(ang_y) >= limit

    def _guidance_attitude(
        self,
        *,
        bbox_valid: float,
        size_ratio: float,
        dt: float,
    ) -> float:
        """Bank to cancel LOS azimuth rate. Pitch stays on the closure governor.

        Proportional only: a held bank is a lateral acceleration, not a held
        speed. Positive azimuth rate (bearing swinging east) banks right.
        The gain does not shrink as the box grows, so the line-up continues
        while the drone closes.
        """
        del size_ratio
        self._a_close_mps2 = 0.0
        self._guide_pitch_deg = float(self._pitch_des_gov)
        rate = float(self._los_sample.az_rate_dps)
        dt = max(float(dt), 0.0)
        # Independent of the pitch governor. That timer never starts when the
        # dive latches on the first frame, which held the wings level.
        if bbox_valid >= 0.5:
            self._bank_t += dt
        if bbox_valid < 0.5 or self._bank_t < self.bank_warmup_s:
            target = 0.0
        else:
            target = float(np.clip(
                self.k_bank * rate, -self.bank_max_deg, self.bank_max_deg,
            ))
        step = self.bank_slew_deg_s * dt
        delta = float(np.clip(target - self._roll_des_deg, -step, step))
        self._roll_des_deg = float(np.clip(
            self._roll_des_deg + delta, -self.bank_max_deg, self.bank_max_deg,
        ))
        return self._roll_des_deg

    def _altitude_limits(self, phase: str, bbox_valid: float) -> tuple[float, float]:
        """High floor unless the gate is locked and the dive has started."""
        if phase == "dive" and bbox_valid >= 0.5:
            return self.alt_floor_dive_m, self.alt_climb_dive_m
        return self.alt_floor_m, self.alt_climb_m

    def _frame_guard_vz(self, ang_y: float, bbox_valid: float) -> float | None:
        """Extra sink once the gate nears the bottom edge. None when it is not."""
        if bbox_valid < 0.5 or float(ang_y) <= self.ang_y_guard_deg:
            return None
        vz = -self.k_frame_vz * (float(ang_y) - self.ang_y_guard_deg)
        return float(np.clip(vz, -self.frame_sink_max, 0.0))

    def _elevation_rate_des(self) -> float:
        """Elevation rate that walks the depression to the shallow target.

        Positive when the gate is too far below the horizon: the gate should
        rise in the line of sight, which means descending faster than the
        collision course. Zero at the target, so the rate law then holds it.
        """
        signed_dep = -float(self._los_sample.el_deg)
        rate_des = self.k_el_shallow * (signed_dep - self.dep_target_deg)
        return float(np.clip(
            rate_des, -self.el_shallow_rate_max, self.el_shallow_rate_max,
        ))

    def _elevation_vz(self, size_ratio: float) -> float:
        """Vertical speed that drives the elevation rate to the shallow target.

        Negative elevation rate means the gate is dropping in the line of
        sight, so the command descends. Dividing by the size ratio turns the
        angular rate back into a velocity: the gate is closer when it is larger.
        The integral keeps that descent after the rate has been matched.
        """
        rate = float(self._los_sample.el_rate_dps)
        dt = max(float(self.dt), 0.0)
        # Independent of the pitch governor: that timer freezes once the dive
        # latches, which would otherwise disable this law for the whole dive.
        self._el_track_t += dt
        self._el_rate_des = self._elevation_rate_des()
        if self._el_track_t < self.bank_warmup_s:
            return 0.0
        err = rate - self._el_rate_des
        scale = 1.0 / max(float(size_ratio), 0.4)
        winding = (
            abs(self._vz_el_int) >= self.vz_des_max - 0.2
            and err * self._vz_el_int > 0.0
        )
        if not winding:
            self._vz_el_int += self.k_el_i * scale * err * dt
            self._vz_el_int = float(np.clip(
                self._vz_el_int, -self.vz_des_max, self.vz_des_max,
            ))
        return float(np.clip(
            self.k_el_p * scale * err + self._vz_el_int,
            -self.vz_des_max,
            self.vz_des_max,
        ))

    def _vertical_vz_des(
        self,
        pursuit_vz: float,
        *,
        alt_m: float,
        priv_ok: bool,
        depression_deg: float,
        phase: str,
        ang_y: float = 0.0,
        bbox_valid: float = 0.0,
        size_ratio: float = 1.0,
    ) -> float:
        """Match the elevation rate to the shallow-depression target.

        The frame guard only adds sink when the gate is about to leave the
        bottom of the image. Cruise and a lost lock keep the high altitude
        floor. A locked dive may follow the gate down to the dive floor.
        """
        del depression_deg, pursuit_vz
        if not priv_ok:
            self._alt_hold = 0.0
            return float(self._vz_slew)
        self._alt_hold = 0.0
        if bbox_valid < 0.5:
            self._el_rate_des = 0.0
            self._vz_el_int *= math.exp(-max(float(self.dt), 0.0) / 0.6)
            vz = 0.0
        else:
            vz = self._elevation_vz(size_ratio)
            guard = self._frame_guard_vz(ang_y, bbox_valid)
            if guard is not None:
                vz = min(vz, guard)
        raw = vz
        floor_m, climb_m = self._altitude_limits(phase, bbox_valid)
        if alt_m < floor_m:
            vz = max(vz, 0.0)
        if alt_m < climb_m:
            vz = max(vz, 2.0)
        if vz > raw:
            self._vz_el_int = min(self._vz_el_int, vz)
        vz = float(np.clip(vz, -self.vz_des_max, self.vz_des_max))
        step = self.vz_slew_mps2 * max(float(self.dt), 1e-3)
        self._vz_slew += float(np.clip(vz - self._vz_slew, -step, step))
        return float(self._vz_slew)

    def _roll_cmd(
        self,
        roll_deg: float,
        roll_des_deg: float,
        scale: float = 1.0,
    ) -> tuple[float, float]:
        err = float(roll_des_deg) - float(roll_deg)
        roll_cmd = float(np.clip(
            self.kp_roll_level * err * scale,
            -self.max_roll * scale,
            self.max_roll * scale,
        ))
        return roll_cmd, float(roll_des_deg)

    def _smooth(self, target: np.ndarray) -> np.ndarray:
        alphas = np.array([
            self.alpha_roll, self.alpha_pitch, self.alpha_throttle, self.alpha_yaw,
        ], dtype=np.float64)
        smoothed = alphas * target + (1.0 - alphas) * self.prev_action
        limits = np.array([
            self.roll_rate_limit,
            self.pitch_rate_limit,
            self.throttle_rate_limit,
            self.yaw_rate_limit,
        ], dtype=np.float64)
        delta = np.clip(smoothed - self.prev_action, -limits, limits)
        out = self.prev_action + delta
        out[0] = float(np.clip(out[0], -self.max_roll, self.max_roll))
        out[1] = float(np.clip(out[1], -self.max_pitch, self.max_pitch))
        out[2] = float(np.clip(out[2], self.thrust_cmd_min, self.thrust_cmd_max))
        out[3] = float(np.clip(out[3], -self.max_yaw, self.max_yaw))
        return out.astype(np.float32)

    def _propagate_attitude(
        self,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        roll_rate_dps: float,
        pitch_rate_dps: float,
        yaw_rate_dps: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """Body attitude from the gyros, seeded by the EKF at lock.

        The EKF has no lateral-velocity reference, so sustained acceleration
        shows up as a drifting tilt. Integrating the gyros avoids that. A slow
        leak toward the EKF bounds real-gyro bias on a long flight.
        """
        ekf = np.array([
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        ], dtype=np.float64)
        if not self._att_ready:
            self._att_rad = ekf.copy()
            self._att_ready = True
            return roll_deg, pitch_deg, yaw_deg
        phi, theta, psi = (float(v) for v in self._att_rad)
        p = math.radians(roll_rate_dps)
        q = math.radians(pitch_rate_dps)
        r = math.radians(yaw_rate_dps)
        dt = max(float(dt), 0.0)
        cth = math.cos(theta)
        if abs(cth) < 0.2:
            cth = math.copysign(0.2, cth if cth != 0.0 else 1.0)
        phi_dot = p + (q * math.sin(phi) + r * math.cos(phi)) * math.tan(theta)
        theta_dot = q * math.cos(phi) - r * math.sin(phi)
        psi_dot = (q * math.sin(phi) + r * math.cos(phi)) / cth
        phi += phi_dot * dt
        theta += theta_dot * dt
        psi += psi_dot * dt
        blend = dt / max(self.att_ekf_tau_s, 1e-3)
        phi += blend * wrap_pi(float(ekf[0]) - phi)
        theta += blend * wrap_pi(float(ekf[1]) - theta)
        psi += blend * wrap_pi(float(ekf[2]) - psi)
        self._att_rad = np.array([
            wrap_pi(phi),
            float(np.clip(theta, -math.pi / 2.0 + 0.05, math.pi / 2.0 - 0.05)),
            wrap_pi(psi),
        ], dtype=np.float64)
        return tuple(float(math.degrees(v)) for v in self._att_rad)

    def compute_action(self, obs, privileged=None, dt=None, frame_id=None):
        if frame_id is not None and self._last_output is not None:
            if frame_id == self._last_frame_id:
                return self._last_output
        self._last_frame_id = frame_id

        if dt is None:
            now = time.perf_counter()
            dt = self.default_dt_s if self._wall_t_prev is None else now - self._wall_t_prev
            self._wall_t_prev = now
        dt = float(np.clip(dt, 1e-3, 0.5))
        self.dt = dt
        self.elapsed_t += dt
        inv_dt = 1.0 / dt

        ang_x = float(obs[4])
        ang_y = float(obs[5])
        bbox_w = float(obs[2])
        bbox_h = float(obs[3])
        roll_deg = float(obs[6])
        pitch_deg = float(obs[7])
        yaw_deg = float(obs[8])
        roll_rate = float(obs[9])
        pitch_rate = float(obs[10])
        yaw_rate = float(obs[11])
        roll_deg, pitch_deg, yaw_deg = self._propagate_attitude(
            roll_deg, pitch_deg, yaw_deg,
            roll_rate, pitch_rate, yaw_rate, dt,
        )
        bbox_valid = float(obs[13])
        camera_pitch_deg = camera_pitch_from_obs(
            obs, default_deg=self.camera_pitch_default_deg,
        )

        priv_ok, alt, vz_up = self._parse_privileged(privileged)

        self.step_count += 1
        bbox_area = bbox_w * bbox_h
        bbox_size = math.sqrt(max(bbox_area, 1e-10))
        self._update_frame_dynamics(ang_x, ang_y, bbox_size, bbox_valid, inv_dt)

        # Inertial LOS rate. Image ang_x plus body yaw rate is not this: roll
        # couples a low target into ang_x, and body yaw rate is not world yaw rate.
        self._los_sample = self._los_tracker.update(
            ang_x_deg=ang_x,
            ang_y_deg=ang_y,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            camera_pitch_deg=camera_pitch_deg,
            dt=dt,
            valid=bbox_valid > 0.5,
        )
        los_az_dps = float(self._los_sample.az_rate_raw_dps)
        los_el_dps = float(self._los_sample.el_rate_raw_dps)
        los_az_filt = float(self._los_sample.az_rate_dps)

        self._note_bbox_reference(bbox_size, bbox_valid, dt)
        self._note_ang_y_reference(ang_y, bbox_valid, dt)
        phase = self._select_phase(bbox_valid, bbox_size, ang_x, ang_y)
        self.current_phase = phase
        size_ratio = self._size_ratio(bbox_size) if bbox_valid > 0.5 else 0.0
        elev_for_lead = (
            self._gate_elevation_deg(pitch_deg, ang_y, camera_pitch_deg)
            if bbox_valid > 0.5
            else 0.0
        )
        self._update_v_adapt(elev_for_lead, bbox_valid, phase, dt)
        self._update_pitch_des_governor(
            bbox_valid=bbox_valid,
            bbox_size=bbox_size,
            phase=phase,
            dt=dt,
            ang_x=ang_x,
            ang_y=ang_y,
            pitch_deg=pitch_deg,
            vz_up=vz_up,
            priv_ok=priv_ok,
            camera_pitch_deg=camera_pitch_deg,
        )

        if priv_ok and bbox_valid > 0.5 and self._alt_ref is None and alt > 5.0:
            self._alt_ref = float(alt)
        depression_deg = 0.0
        if bbox_valid > 0.5:
            depression_deg = max(0.0, -float(self._los_sample.el_deg))
        roll_des = self._guidance_attitude(
            bbox_valid=bbox_valid,
            size_ratio=size_ratio,
            dt=dt,
        )
        pitch_override = None

        pitch_des = elev_deg = 0.0
        t_ff = t_fb = vz_des = 0.0
        roll_crab = 0.0

        if bbox_valid < 0.5:
            mem_x = 0.0 if self.prev_ang_x is None else float(self.prev_ang_x)
            yaw_cmd = float(np.clip(
                self.kp_yaw * self.lost_yaw_kp_scale * mem_x,
                -self.lost_yaw_scan,
                self.lost_yaw_scan,
            ))
            roll_cmd, roll_crab = self._roll_cmd(
                roll_deg, 0.0, scale=self.lost_roll_cmd_scale,
            )
            if priv_ok:
                vz_des_est = self._estimate_vz_des(
                    ang_y=0.0,
                    pitch_deg=pitch_deg,
                    camera_pitch_deg=camera_pitch_deg,
                )
                pitch_cmd, pitch_des = self._pitch_att_cmd(
                    pitch_deg=pitch_deg,
                    pitch_rate_dps=pitch_rate,
                    priv_ok=True,
                    vz_up=vz_up,
                    vz_des_est=vz_des_est,
                )
            else:
                pitch_cmd = float(np.clip(
                    self.lost_pitch_kp * (self.lost_pitch_target_deg - pitch_deg),
                    -self.max_pitch * self.lost_pitch_cmd_scale,
                    self.max_pitch * self.lost_pitch_cmd_scale,
                ))
                pitch_des = self.lost_pitch_target_deg
            throttle_cmd, tilt, t_ff, t_fb, vz_des, elev_deg = self._thrust_vertical(
                phase=phase,
                bbox_valid=0.0,
                ang_y=0.0,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                priv_ok=priv_ok,
                vz_up=vz_up,
                dt=dt,
                camera_pitch_deg=camera_pitch_deg,
                alt_m=alt,
                depression_deg=0.0,
            )
            ax_used = mem_x
            ang_y_err = 0.0
        else:
            roll_cmd, roll_crab = self._roll_cmd(roll_deg, roll_des)
            vz_des_est = (
                self._estimate_vz_des(
                    ang_y=ang_y,
                    pitch_deg=pitch_deg,
                    camera_pitch_deg=camera_pitch_deg,
                )
                if priv_ok
                else None
            )
            pitch_cmd, pitch_des = self._pitch_att_cmd(
                pitch_deg=pitch_deg,
                pitch_rate_dps=pitch_rate,
                priv_ok=priv_ok,
                vz_up=vz_up,
                vz_des_est=vz_des_est,
                pitch_override_deg=pitch_override,
            )
            yaw_cmd, ax_used = self._yaw_cmd(ang_x, yaw_rate)
            throttle_cmd, tilt, t_ff, t_fb, vz_des, elev_deg = self._thrust_vertical(
                phase=phase,
                bbox_valid=bbox_valid,
                ang_y=ang_y,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                priv_ok=priv_ok,
                vz_up=vz_up,
                dt=dt,
                camera_pitch_deg=camera_pitch_deg,
                alt_m=alt,
                depression_deg=depression_deg,
                size_ratio=size_ratio,
            )
            ang_y_err = float(ang_y)

        target = np.array([roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd], dtype=np.float64)
        body_action = self._smooth(target)
        self.prev_action = body_action.astype(np.float64)

        if self._acro_converter is not None:
            action = self._acro_converter.body_rates_to_rc_pwm(body_action)
        else:
            action = body_action.astype(np.float32)

        self.last_debug = {
            "step": self.step_count,
            "phase": phase,
            "bbox_area": float(bbox_area),
            "bbox_valid": float(bbox_valid),
            "ang_x": float(ang_x),
            "ang_y": float(ang_y),
            "desired_ang_y": float(elev_deg),
            "ang_y_err": float(ang_y_err),
            "camera_pitch_deg": float(camera_pitch_deg),
            "d_ang_x": float(self.d_ang_x),
            "d_ang_y": float(self.d_ang_y),
            "bbox_rate": float(self.bbox_rate),
            "bbox_size_ratio": float(size_ratio),
            "pitch_gain_scale": 1.0,
            "chase_blend": float(self._governor_active),
            "chase_receding": float(self._chase_receding),
            "roll_lat_cmd": float(roll_crab),
            "roll_level_cmd": float(roll_cmd),
            "roll_kp_scale": 1.0,
            "roll_phase_gain": 1.0,
            "roll_rate_limit_applied": float(self.roll_rate_limit),
            "ang_x_ctrl": float(ax_used),
            "ang_x_spike_filtered": 0.0,
            "lateral_throttle_factor": 0.0,
            "target_bank_deg": 0.0,
            "los_rate_az_rad": float(math.radians(los_az_dps)),
            "los_rate_el_rad": float(math.radians(los_el_dps)),
            "los_rate_az_filt_rad": float(math.radians(los_az_filt)),
            "los_rate_az_dps": float(los_az_dps),
            "los_rate_el_dps": float(los_el_dps),
            "los_rate_az_filt_dps": float(los_az_filt),
            "los_az_world_deg": float(self._los_sample.az_deg),
            "los_el_world_deg": float(self._los_sample.el_deg),
            "los_el_rate_filt_dps": float(self._los_sample.el_rate_dps),
            "roll_pn_cmd": 0.0,
            "roll_vlat_bias": float(roll_crab),
            "roll_des_deg": float(self._roll_des_deg),
            "gyro_roll_deg": float(math.degrees(self._att_rad[0])),
            "gyro_pitch_deg": float(math.degrees(self._att_rad[1])),
            "gyro_yaw_deg": float(math.degrees(self._att_rad[2])),
            "guide_pitch_deg": float(self._guide_pitch_deg),
            "a_close_mps2": float(self._a_close_mps2),
            "depression_deg": float(depression_deg),
            "alt_hold": float(self._alt_hold),
            "alt_ref_m": float(self._alt_ref) if self._alt_ref is not None else float("nan"),
            "track_mode": 1.0 if phase == "dive" else 0.0,
            "roll_ang_trim": 0.0,
            "est_V_lat": float(los_az_filt),
            "est_V_forward": float(self.v_pursuit_ref_mps),
            "est_V_c": float(self._pitch_des_gov),
            "est_Z": float(elev_deg),
            "est_tau": float(pitch_des),
            "est_healthy": float(priv_ok),
            "est_range_ratio": float(self._closure_filt),
            "roll_level_scale": 1.0,
            "bank_clamped": 0.0,
            "roll_cmd_raw": float(roll_cmd),
            "pitch_cmd_raw": float(pitch_cmd),
            "throttle_cmd_raw": float(throttle_cmd),
            "yaw_cmd_raw": float(yaw_cmd),
            "parallel_severity": float(self._vz_int),
            "parallel_active": 0.0,
            "parallel_match_t": 0.0,
            "forward_match_scale": 0.0,
            "track_elapsed_s": 0.0,
            "track_armed": 0.0,
            "track_complete": 0.0,
            "drone_fwd_mps": 0.0,
            "align_elapsed_s": 0.0,
            "align_armed": 0.0,
            "align_complete": 0.0,
            "align_good_streak": 0.0,
            "align_ang_y_ref": float(pitch_des),
            "align_behind_ok": 0.0,
            "gate_drift_dps": float(vz_des),
            "alt_hold_blend": float(t_fb),
            "dive_committed": float(self._dive_committed),
            "forward_dive_scale": 1.0 if phase == "dive" else 0.0,
            "forward_throttle_scale": 0.0,
            "tilt_comp": float(tilt),
            "alt_hold_thrust": float(t_ff + t_fb),
            "alt_ref": float("nan"),
            "alt_meas": float(alt) if priv_ok else float("nan"),
            "vz_up": float(vz_up) if priv_ok else float("nan"),
            "vz_des": float(vz_des),
            "vz_el_int": float(self._vz_el_int),
            "el_rate_des_dps": float(self._el_rate_des),
            "v_adapt_mps": float(self._v_adapt),
            "t_ff": float(t_ff),
            "t_fb": float(t_fb),
            "t_close": 0.0,
            "dive_thrust_boost": 0.0,
            "body_roll_cmd": float(body_action[0]),
            "body_pitch_cmd": float(body_action[1]),
            "body_throttle_cmd": float(body_action[2]),
            "body_yaw_cmd": float(body_action[3]),
            "roll_cmd": float(action[0]),
            "pitch_cmd": float(action[1]),
            "throttle_cmd": float(action[2]),
            "yaw_cmd": float(action[3]),
            "output_mode": self.output_mode,
        }
        if self._acro_converter is not None:
            self.last_debug.update(self._acro_converter.last_mapping_debug())
        self._last_output = action.astype(np.float32)
        return self._last_output

    def reset(self):
        self.prev_action = np.array([0.0, 0.0, self.hover_thrust, 0.0], dtype=np.float64)
        self.step_count = 0
        self.last_debug = {}
        self.prev_ang_x = None
        self.prev_ang_y = None
        self.prev_bbox_size = 0.0
        self._bbox_init_size = 0.0
        self._bbox_ref_sum = 0.0
        self._bbox_ref_n = 0
        self._bbox_ref_t = 0.0
        self._clear_ang_y_reference()
        self._bank_t = 0.0
        self.bbox_rate = 0.0
        self.d_ang_x = 0.0
        self.d_ang_y = 0.0
        self.current_phase = "lost"
        self._dive_committed = False
        self._pitch_des_gov = float(self.pitch_des_cruise_deg)
        self._closure_filt = 0.0
        self._lost_t = 0.0
        self._lock_t = 0.0
        self._vz_int = 0.0
        self._vz_des_lost_hold = 0.0
        self._governor_active = 0.0
        self._chase_receding = 0.0
        self._los_sample = LosRateSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)
        self._los_tracker.reset()
        self._alt_ref = None
        self._alt_hold = 0.0
        self._roll_des_deg = 0.0
        self._vz_slew = 0.0
        self._vz_el_int = 0.0
        self._el_track_t = 0.0
        self._el_rate_des = 0.0
        self._guide_pitch_deg = 0.0
        self._a_close_mps2 = 0.0
        self._elev_prev = None
        self._v_adapt = 0.0
        self._d_elev_filt = 0.0
        self._att_ready = False
        self._att_rad = np.zeros(3, dtype=np.float64)
        self.dt = self.default_dt_s
        self.elapsed_t = 0.0
        self._last_frame_id = None
        self._wall_t_prev = None
        self._last_output = None
        if self._acro_converter is not None:
            self._acro_converter.reset()
