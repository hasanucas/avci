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
from fpv_gate_il.teacher.hc_air_config import cfg_float, load_hc_air_config


class HCAir:
    """
    Action vector (4,) for SET_ATTITUDE_TARGET body-rate + thrust control:

        [0] roll_rate_cmd    — rad/s
        [1] pitch_rate_cmd   — rad/s, negative = nose down
        [2] thrust_cmd       — collective (vertical)
        [3] yaw_rate_cmd     — rad/s
    """

    @classmethod
    def action_to_body_rates(cls, action: np.ndarray) -> tuple[float, float, float, float]:
        a = np.asarray(action, dtype=np.float64)
        return (
            float(a[0]),
            float(a[1]),
            float(a[3]),
            float(a[2]),
        )

    @classmethod
    def _cfg(cls, config_path: str | None = None) -> dict:
        return load_hc_air_config(config_path)

    @classmethod
    def action_channel_limits(
        cls, config_path: str | None = None,
    ) -> tuple[float, float, float, float, float, float]:
        cfg = cls._cfg(config_path)
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

    def __init__(self, config_path: str | None = None, **kwargs):
        if kwargs:
            raise TypeError(f"HCAir accepts no keyword overrides, got {sorted(kwargs)}")
        self._config_path = config_path
        self._apply_config(self._cfg(config_path))

        self.prev_ang_x: float | None = None
        self.prev_ang_y: float | None = None
        self.prev_bbox_size = 0.0
        self._bbox_init_size = 0.0
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
        self._los_az_filt_dps = 0.0
        self._elev_prev: float | None = None
        self._v_adapt = 0.0
        self._d_elev_filt = 0.0

        self.prev_action = np.array([0.0, 0.0, self.hover_thrust, 0.0], dtype=np.float64)
        self.step_count = 0
        self.last_debug: dict = {}
        self.dt = self.default_dt_s
        self.elapsed_t = 0.0
        self._last_frame_id = None
        self._wall_t_prev = None
        self._last_output = None

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

    def _filter_los_az(self, los_az_dps: float, bbox_valid: float) -> float:
        if bbox_valid < 0.5:
            self._los_az_filt_dps *= self.los_az_lost_decay
            return float(self._los_az_filt_dps)
        alpha = float(self.los_az_filt_alpha)
        self._los_az_filt_dps = (
            alpha * float(los_az_dps) + (1.0 - alpha) * self._los_az_filt_dps
        )
        return float(self._los_az_filt_dps)

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
    ) -> tuple[float, float]:
        """Returns pitch_rate_cmd, pitch_des_deg."""
        if priv_ok:
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
                    # Less nose-down / slight nose-up to free vertical thrust when
                    # climb demand is unmet (gate above, thrust often already saturated).
                    p_min = float(self.pitch_des_min_relaxed_deg)
                    boost = min(
                        self.k_pitch_assist
                        * (sink_excess - self.pitch_assist_sink_err),
                        self.pitch_assist_max_deg,
                    )
                    pitch_des = float(pitch_des) + float(boost)
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
    ) -> tuple[float, float, float, float, float, float]:
        """
        Returns thrust, tilt_comp, t_ff, t_fb, vz_des, elev_deg.
        """
        t_ff, tilt_comp = self._tilt_ff(roll_deg, pitch_deg)
        elev_deg = self._gate_elevation_deg(pitch_deg, ang_y, camera_pitch_deg)
        t_fb = 0.0
        vz_des = 0.0

        if bbox_valid > 0.5 and priv_ok:
            vz_des = self._estimate_vz_des(
                ang_y=ang_y,
                pitch_deg=pitch_deg,
                camera_pitch_deg=camera_pitch_deg,
            )
            self._vz_des_lost_hold = float(vz_des)
            vz_err = float(vz_des) - float(vz_up)
            integrate = phase != "dive"
            if integrate:
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
            vz_des = float(self._vz_des_lost_hold)
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
            if self._bbox_init_size <= 0.0:
                self._bbox_init_size = bbox_size
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
        if self._bbox_init_size <= 0.0:
            self._bbox_init_size = bbox_size

    def _select_phase(
        self, bbox_valid: float, bbox_size: float, ang_x: float, ang_y: float,
    ) -> str:
        if bbox_valid < 0.5:
            self._dive_committed = False
            return "lost"
        if self._dive_committed:
            return "dive"
        ratio = self._size_ratio(bbox_size)
        aimed = (
            abs(float(ang_x)) <= self.dive_aim_max_deg
            and abs(float(ang_y)) <= self.dive_aim_max_deg
        )
        if ratio + 1e-6 >= self.dive_size_ratio and aimed:
            self._dive_committed = True
            return "dive"
        return "cruise"

    def _yaw_cmd(self, ang_x: float, yaw_rate_dps: float) -> tuple[float, float]:
        ax = self._deadband(ang_x, self.ang_x_deadband_deg)
        yaw_cmd = self.kp_yaw * ax - self.kd_yaw_body * float(yaw_rate_dps)
        yaw_cmd = float(np.clip(yaw_cmd, -self.max_yaw, self.max_yaw))
        return yaw_cmd, ax

    def _roll_cmd(
        self,
        roll_deg: float,
        bbox_valid: float,
        los_az_filt_dps: float,
        scale: float = 1.0,
    ) -> tuple[float, float]:
        level = -self.kp_roll_level * float(roll_deg)
        crab = 0.0
        if bbox_valid > 0.5:
            crab = self.kp_crab_los * float(los_az_filt_dps)
        roll_cmd = float(np.clip(
            (level + crab) * scale,
            -self.max_roll * scale,
            self.max_roll * scale,
        ))
        return roll_cmd, crab

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
        pitch_rate = float(obs[10])
        yaw_rate = float(obs[11])
        bbox_valid = float(obs[13])
        camera_pitch_deg = camera_pitch_from_obs(
            obs, default_deg=self.camera_pitch_default_deg,
        )

        priv_ok, alt, vz_up = self._parse_privileged(privileged)

        self.step_count += 1
        bbox_area = bbox_w * bbox_h
        bbox_size = math.sqrt(max(bbox_area, 1e-10))
        self._update_frame_dynamics(ang_x, ang_y, bbox_size, bbox_valid, inv_dt)

        los_az_dps = 0.0
        if bbox_valid > 0.5:
            los_az_dps = self.d_ang_x * inv_dt + float(yaw_rate)
        los_az_filt = self._filter_los_az(los_az_dps, bbox_valid)

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
                roll_deg, bbox_valid, los_az_filt, scale=self.lost_roll_cmd_scale,
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
            )
            ax_used = mem_x
            ang_y_err = 0.0
        else:
            roll_cmd, roll_crab = self._roll_cmd(
                roll_deg, bbox_valid, los_az_filt,
            )
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
            )
            ang_y_err = float(ang_y)

        target = np.array([roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd], dtype=np.float64)
        action = self._smooth(target)
        self.prev_action = action.astype(np.float64)

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
            "los_rate_el_rad": 0.0,
            "los_rate_az_filt_rad": float(math.radians(los_az_filt)),
            "los_rate_az_dps": float(los_az_dps),
            "los_rate_el_dps": 0.0,
            "los_rate_az_filt_dps": float(los_az_filt),
            "roll_pn_cmd": 0.0,
            "roll_vlat_bias": float(roll_crab),
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
            "v_adapt_mps": float(self._v_adapt),
            "t_ff": float(t_ff),
            "t_fb": float(t_fb),
            "t_close": 0.0,
            "dive_thrust_boost": 0.0,
            "roll_cmd": float(action[0]),
            "pitch_cmd": float(action[1]),
            "throttle_cmd": float(action[2]),
            "yaw_cmd": float(action[3]),
        }
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
        self._los_az_filt_dps = 0.0
        self._elev_prev = None
        self._v_adapt = 0.0
        self._d_elev_filt = 0.0
        self.dt = self.default_dt_s
        self.elapsed_t = 0.0
        self._last_frame_id = None
        self._wall_t_prev = None
        self._last_output = None
