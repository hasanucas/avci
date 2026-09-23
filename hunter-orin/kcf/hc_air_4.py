#!/usr/bin/env python3
# =============================================================================
# HUNTER ADAPTASYONU — hc_air_4.py  (31 Agu 2026)
# =============================================================================
# KAYNAK : angle_tar.gz  src/fpv_gate_il/teacher/hc_air.py  (1848 satir)
# BAZ     : repo kcf/hc_air_1.py (1485 satir)
# FARK    : +416 / -53 satir, 46 hunk. Revizyon degil, YENI SURUM.
#           Yeni alt sistemler: ground_floor (privileged AGL bandi),
#           _pass_commit, _soft_sat_los_dps, _shed_bank_for_climb,
#           _term_boost_allowed, _terminal_gate_above.
#           Roll ciddi yumusatildi (alpha 0.45->0.32, slew 90->45,
#           term slew 140->55, max_bank_term 48->40, LOS soft-sat 10 dps).
#           thrust_cmd_max 0.30 -> 0.75.
#
# HUNTER'A OZEL 4 DEGISIKLIK (hepsi dosya icinde ACIKLAMALI):
#   1. Config cozumu: parents[3] yolu yerine kcf/fpv_gate_4.yaml aranir,
#      bulunamazsa GURULTULU coker. ESKI fpv_gate.yaml'a asla dusmez.
#      (27 Agu devir notundaki "config tuzagi".)
#   2. Terminal esigi: GORECELI size_ratio yolu geri kondu ve varsayilan
#      yapildi. Upstream'in ref_width_m/ref_height_m/ref_range_m mutlak
#      yolu "hedef 0.5x0.5 m" varsayimi tasiyordu — HUNTER ilkesine aykiri.
#      Mutlak yol opt-in ve acilista gorunur uyari basiyor.
#   3. PITCH TOHUM YAMASI (25 Agu) yeniden uygulandi. Bu dosya kusuru
#      yapisal olarak tasiyordu: reset() pitch'i cruise'a tohumluyor,
#      burun-asagi angajmanda ilk kare +18...+25 derece sicrama.
#      _pitch_initialized bayragi, olculen pitch'ten tohumlama. 12 satir.
#   4. ground_floor VARSAYILAN KAPALI (fpv_gate_4.yaml enabled: false).
#      Gerekce: controller_guided icindeki FloorMonitor (20 m, ALT_HOLD'a
#      atar) ile ayni anda calisirsa iki taban sigortasi carpisir, ve
#      teacher'inki priv['altitude'] = fc.alt_rel = BARO okuyor — baro
#      kalibrasyonu HALA acik is (BARO1_GND_PRESS ~1000 m kaymis).
#      Tek degisken disiplini: once FloorMonitor tek otorite kalsin.
#
# UYARI: --att-pitch-max kelepcesi pitch tohumunu sender'da EZER.
#        Bu teacher'la o bayragi KULLANMA.
# =============================================================================
"""
Hard-coded teacher for air-to-air gates — attitude-hold setpoints (GPS-free).

Vision + IMU attitude/gyro + baro vertical velocity (EKF vz only).
No horizontal vel_ned / GPS / metric gate width.

Action vector (4,) for SET_ATTITUDE_TARGET quaternion hold:

    [0] roll_des_deg
    [1] pitch_des_deg   — negative = nose down
    [2] thrust_cmd      — collective (vertical)
    [3] yaw_des_deg     — absolute heading

  Pitch  — closure governor → pitch_des; near-gate off-aim bleeds pitch
           toward lat_slow_des (extend t_go) and holds while still off-aim.
           Altitude protect bleeds pitch toward cruise when sinking hard or
           thrust is saturated. Overfly (near + gate below reticle) bleeds
           nose-down + sink boost.
  Thrust — cruise: horizon reticle on gate TOP → vz_des + PI (same altitude,
           slightly above gate center). lat_slow: climb clamped (or mild climb
           when elev high). Terminal (near): climb capped when level/below;
           gate-above keeps climb + thr (est8); sink/overfly free.
           Ground floor (privileged AGL): hold band clips sink + climb-arrest
           and bleeds pitch toward hold_des (above cruise) + fast thr slew
           when off-pass; aimed pass-commit dives to crash_floor only so the
           aperture can be flown through (est50 overfly fix). Far sink cap
           until terminal size (lock size_ratio=1 must not disable it).
  Yaw    — measured yaw + offset from ang_x.
  Roll   — bank from range-scaled LOS az crab + ang_x bank (early trim).
           Soft-sat LOS, climb bank-shed when on-aim, term boost only off-aim,
           soft roll slew (GPS-free). Overfly bank-shed skipped when |ang_x| large.

Phases: lost → cruise (terminal band when bbox_size ≥ FOV-derived size; no dive latch).
"""

from __future__ import annotations

import math
import time
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from fpv_gate_il.features import (
    DEFAULT_CAMERA_HFOV_DEG,
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_WIDTH,
    _camera_vfov_deg,
    camera_pitch_from_obs,
)
from fpv_gate_il.teacher.camera_geometry import normalized_bbox_size_at_range

# =============================================================================
# HUNTER config cozumu — upstream'den DEGISTIRILDI (31 Agu 2026)
# =============================================================================
# Upstream varsayilani  parents[3]/config/fpv_gate.yaml  idi. HUNTER'in duz
# kcf/ yapisinda o yol TUTMAZ ve dosya bulunamayinca davranis surume gore
# degisiyordu — 27 Agu devir notundaki "config tuzagi" tam olarak buydu:
# hc_air_2'ye --teacher-config verilmeyince SESSIZCE yanlis yaml yukleniyordu
# (terminal=0, ground_floor kapali, bank 48) ve kimse fark etmiyordu.
#
# Burada kural: DOGRU dosya fpv_gate_4.yaml'dir, --teacher-config ile ACIKCA
# verilir. Verilmezse kcf/ altinda SADECE fpv_gate_4.yaml aranir; ESKI
# fpv_gate.yaml'a asla dusulmez. Bulunamazsa GURULTULU coker.
# Her yuklemede hangi dosyanin okundugu ekrana basilir.
# =============================================================================
_HERE = Path(__file__).resolve()
_KCF_DIR = _HERE.parent                       # kcf/

_CANDIDATES = [
    _KCF_DIR / "fpv_gate_4.yaml",                     # kcf/fpv_gate_4.yaml (ONERILEN)
    _KCF_DIR / "config" / "fpv_gate_4.yaml",
    _KCF_DIR.parent / "config" / "fpv_gate_4.yaml",
]


def _find_config() -> Path:
    import os
    env = os.environ.get("HC_AIR_4_YAML")
    if env:
        p = Path(env)
        if p.exists():
            return p
    for c in _CANDIDATES:
        if c.exists():
            return c
    aranan = "\n  ".join(str(c) for c in _CANDIDATES)
    raise FileNotFoundError(
        "hc_air_4 config'i (fpv_gate_4.yaml) bulunamadi. --teacher-config ile "
        "acikca ver, ya da su konumlardan birine koy:\n  " + aranan +
        "\n  (ESKI fpv_gate.yaml BILEREK aday degil — hc_air_1 icindir.)"
    )


def _deep_get(cfg: dict[str, Any], dotted: str, default: Any = None) -> Any:
    node: Any = cfg
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


@lru_cache(maxsize=8)
def load_hc_air_config(path: str | None = None) -> dict[str, Any]:
    """Load HCAir params from config/fpv_gate.yaml (``hc_air`` section).

    If ``path`` points to a file without an ``hc_air`` key, the whole document
    is treated as the teacher config (portable single-section YAML).
    """
    cfg_path = Path(path) if path is not None else _find_config()
    with cfg_path.open(encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    print(f"[HC_AIR_4] config yuklendi: {cfg_path}")
    if isinstance(raw, dict) and "hc_air" in raw:
        return raw["hc_air"]
    return raw


def cfg_float(cfg: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = _deep_get(cfg, key, default)
    return float(value)


class HCAir:
    """Attitude-hold teacher: action = [roll_deg, pitch_deg, thrust, yaw_deg]."""

    @classmethod
    def action_to_attitude_deg(cls, action: np.ndarray) -> tuple[float, float, float, float]:
        a = np.asarray(action, dtype=np.float64)
        return float(a[0]), float(a[1]), float(a[2]), float(a[3])

    @classmethod
    def action_to_body_rates(cls, action: np.ndarray) -> tuple[float, float, float, float]:
        """Legacy alias — rates no longer used; returns attitude channels."""
        roll, pitch, thrust, yaw = cls.action_to_attitude_deg(action)
        return roll, pitch, yaw, thrust

    @classmethod
    def _cfg(cls, config_path: str | None = None) -> dict:
        return load_hc_air_config(config_path)

    @classmethod
    def action_channel_limits(
        cls, config_path: str | None = None,
    ) -> tuple[float, float, float, float, float, float]:
        """
        Returns (max_bank, pitch_down_mag, pitch_up_mag, max_yaw_offset,
                 thrust_min, thrust_max) for env clipping helpers.
        """
        cfg = cls._cfg(config_path)
        al = cfg["action_limits"]
        plat = cfg["platform"]
        return (
            float(al["max_bank_deg"]),
            abs(float(al["pitch_des_min_deg"])),
            abs(float(al["pitch_des_max_deg"])),
            float(al["max_yaw_offset_deg"]),
            float(plat["thrust_cmd_min"]),
            float(plat["thrust_cmd_max"]),
        )

    @classmethod
    def attitude_limits(
        cls, config_path: str | None = None,
    ) -> tuple[float, float, float, float, float]:
        """max_bank, pitch_min, pitch_max, thrust_min, thrust_max."""
        cfg = cls._cfg(config_path)
        al = cfg["action_limits"]
        plat = cfg["platform"]
        return (
            float(al["max_bank_deg"]),
            float(al["pitch_des_min_deg"]),
            float(al["pitch_des_max_deg"]),
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
        ap = cfg["altitude_protect"]
        gf = cfg.get("ground_floor", {})
        pg = cfg["pitch_governor"]
        roll = cfg["roll"]
        vt = cfg["vertical_thrust"]
        va = cfg["vertical_adapt"]
        term = cfg.get("terminal", {})
        lost = cfg["lost_behavior"]
        dyn = cfg["dynamics"]
        sm = cfg["smoothing"]
        al = cfg["action_limits"]
        of = cfg.get("overfly", {})

        self.hover_thrust = float(plat["hover_thrust"])
        self.ff_open_loop_scale = float(plat["ff_open_loop_scale"])
        self.thrust_cmd_min = float(plat["thrust_cmd_min"])
        self.thrust_cmd_max = float(plat["thrust_cmd_max"])
        self.tilt_cos_min = float(plat["tilt_cos_min"])

        self.camera_pitch_default_deg = float(cam["pitch_default_deg"])
        self.camera_pitch_deg = self.camera_pitch_default_deg
        self.cam_axis_offset_deg = float(cam["axis_offset_deg"])
        self.cam_pitch_transfer = float(cam["pitch_transfer"])
        self.use_gate_top_reticle = bool(cam.get("use_gate_top_reticle", True))
        self.reticle_elev_bias_deg = float(cam.get("reticle_elev_bias_deg", 0.0))
        aspect = float(DEFAULT_CAMERA_WIDTH) / max(float(DEFAULT_CAMERA_HEIGHT), 1.0)
        self.camera_vfov_deg = _camera_vfov_deg(DEFAULT_CAMERA_HFOV_DEG, aspect)

        self.ang_x_deadband_deg = float(yaw["ang_x_deadband_deg"])
        self.kp_yaw_offset = float(yaw["kp_offset_deg_per_deg"])
        self.max_yaw_offset = float(yaw["max_offset_deg"])

        self.pitch_des_cruise_deg = float(pa["cruise_des_deg"])
        self.pitch_des_min_deg = float(pa["des_min_deg"])
        self.pitch_des_max_deg = float(pa["des_max_deg"])
        self.pitch_assist_climb_err = float(pa["assist_climb_err_mps"])
        self.pitch_assist_sink_err = float(pa["assist_sink_err_mps"])
        self.k_pitch_assist = float(pa["assist_k_deg_per_mps"])
        self.pitch_assist_max_deg = float(pa["assist_max_deg"])
        self.pitch_des_min_relaxed_deg = float(pa["des_min_relaxed_deg"])
        self.pitch_des_fallback_deg = float(pa["fallback_des_deg"])

        self.alt_protect_sink_mps = float(ap["sink_mps"])
        self.alt_protect_thrust_sat_frac = float(ap["thrust_sat_frac"])
        self.alt_protect_thrust_sat_vz_margin = float(ap["thrust_sat_vz_margin_mps"])
        self.k_pitch_alt_protect = float(ap["bleed_deg_s"])
        self.alt_protect_sat_floor_cruise = bool(ap["sat_pitch_floor_cruise"])

        self.ground_floor_enabled = bool(gf.get("enabled", False))
        self.ground_floor_m = float(gf.get("floor_m", 4.0))
        self.ground_floor_soft_start_m = float(gf.get("soft_start_m", 8.0))
        self.ground_floor_max_sink_near_mps = float(gf.get("max_sink_near_mps", 0.0))
        self.ground_floor_climb_arrest_mps = float(gf.get("climb_arrest_mps", 0.0))
        self.ground_floor_pitch_bleed_deg_s = float(gf.get("pitch_bleed_deg_s", 0.0))
        self.ground_floor_pitch_hold_des_deg = float(
            gf.get("pitch_hold_des_deg", 0.0)
        )
        self.ground_floor_override_overfly = bool(gf.get("override_overfly", True))
        self.ground_floor_bypass_term_thr = bool(
            gf.get("bypass_terminal_thrust_cap", True)
        )
        self.ground_floor_pass_size_ratio = float(gf.get("pass_size_ratio", 1.5))
        self.ground_floor_pass_aim_deg = float(gf.get("pass_aim_deg", 4.0))
        self.ground_floor_crash_m = float(gf.get("crash_floor_m", 1.2))
        self.ground_floor_crash_soft_m = float(gf.get("crash_soft_start_m", 2.5))

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
        self.lat_slow_size_ratio = float(pg["lat_slow_size_ratio"])
        self.lat_slow_aim_deg = float(pg["lat_slow_aim_deg"])
        self.lat_slow_des_deg = float(pg["lat_slow_des_deg"])
        self.k_pitch_lat_slow = float(pg["lat_slow_deg_s"])
        self.lat_slow_climb_elev_deg = float(pg.get("lat_slow_climb_elev_deg", 2.0))
        self.lat_slow_climb_cap_mps = float(pg.get("lat_slow_climb_cap_mps", 0.0))
        self.lost_pitch_des_decay_after_s = float(pg["lost_des_decay_after_s"])
        self.k_pitch_lost_decay = float(pg["lost_des_decay_deg_s"])

        self.kp_crab_bank = float(roll["kp_crab_bank_deg_per_dps"])
        self.los_az_filt_alpha = float(roll["los_az_filt_alpha"])
        self.los_az_lost_decay = float(roll["los_az_lost_decay"])
        self.crab_los_deadband_dps = float(roll.get("crab_los_deadband_dps", 0.0))
        self.crab_far_ref_size_ratio = float(roll.get("crab_far_ref_size_ratio", 1.0))
        self.crab_far_size_floor = float(roll.get("crab_far_size_floor", 0.25))
        self.crab_far_scale_max = float(roll.get("crab_far_scale_max", 3.0))
        self.crab_near_size_ratio = float(roll.get("crab_near_size_ratio", 1.0))
        self.crab_near_scale = float(roll.get("crab_near_scale", 1.0))
        self.crab_near_deadband_dps = float(
            roll.get("crab_near_deadband_dps", self.crab_los_deadband_dps)
        )
        self.crab_term_scale = float(roll.get("crab_term_scale", 1.0))
        self.crab_term_require_off_aim = bool(
            roll.get("crab_term_require_off_aim", False)
        )
        self.crab_term_aim_deg = float(roll.get("crab_term_aim_deg", 2.0))
        self.los_crab_soft_sat_dps = float(roll.get("los_crab_soft_sat_dps", 0.0))
        self.kp_ang_x_bank = float(roll["kp_ang_x_bank_deg_per_deg"])
        self.ang_x_bank_size_ratio = float(roll["ang_x_bank_size_ratio"])
        self.ang_x_term_scale = float(roll.get("ang_x_term_scale", 1.0))
        self.max_bank_deg = float(roll["max_bank_deg"])
        self.max_bank_term_deg = float(
            roll.get("max_bank_term_deg", self.max_bank_deg)
        )
        self.overfly_bank_shed_frac = float(roll.get("overfly_bank_shed_frac", 0.0))
        self.overfly_bank_shed_climb_mps = float(
            roll.get("overfly_bank_shed_climb_mps", 0.5)
        )
        self.overfly_bank_shed_ax_min_deg = float(
            roll.get("overfly_bank_shed_ax_min_deg", 0.0)
        )
        self.climb_bank_shed_frac = float(roll.get("climb_bank_shed_frac", 0.0))
        self.climb_bank_shed_vz_des_mps = float(
            roll.get("climb_bank_shed_vz_des_mps", 1.5)
        )
        self.climb_bank_shed_ax_min_deg = float(
            roll.get("climb_bank_shed_ax_min_deg", 2.5)
        )
        self.yaw_term_scale = float(roll.get("yaw_term_scale", 1.0))

        self.overfly_size_ratio = float(of.get("size_ratio", 1.2))
        self.overfly_elev_deg = float(of.get("elev_deg", -1.5))
        self.overfly_sink_boost = float(of.get("sink_boost", 1.6))
        self.overfly_min_sink_mps = float(of.get("min_sink_mps", 2.5))
        self.overfly_pitch_bleed_deg_s = float(of.get("pitch_bleed_deg_s", 8.0))
        self.overfly_pitch_des_deg = float(of.get("pitch_des_deg", -12.0))
        self.overfly_thrust_cmd_min = float(
            of.get("thrust_cmd_min", self.thrust_cmd_min)
        )
        self.overfly_zero_tilt_ff = bool(of.get("zero_tilt_ff", True))
        self.overfly_reset_vz_int = bool(of.get("reset_vz_int_on_enter", True))

        self.kp_vz = float(vt["kp"])
        self.ki_vz = float(vt["ki"])
        self.k_pursuit_n = float(vt["k_pursuit"])
        self.v_pursuit_ref_mps = float(vt["v_pursuit_ref_mps"])
        self.vz_int_max = float(vt["vz_int_max"])
        self.vz_des_max = float(vt["vz_des_max_mps"])
        self.far_sink_cap_mps = float(vt.get("far_sink_cap_mps", 2.0))
        self.t_fb_max = float(vt["t_fb_max"])
        self.pitch_thrust_gain = float(vt["pitch_thrust_gain"])
        self.sink_ff_fade_elev_deg = float(vt["sink_ff_fade_elev_deg"])
        self.lost_vz_decay_tau_s = float(vt["lost_vz_decay_tau_s"])

        self.k_v_adapt_per_deg = float(va["k_per_deg"])
        self.v_adapt_max = float(va["v_max_mps"])
        self.v_adapt_decay_tau_s = float(va["decay_tau_s"])
        self.elev_lag_sign_deadband_deg = float(va["lag_sign_deadband_deg"])
        self.elev_drift_filt_alpha = float(va["elev_drift_filt_alpha"])
        self.elev_drift_step_max_deg = float(va["elev_drift_step_max_deg"])

        # ---------------------------------------------------------------
        # TERMINAL ESIGI — HUNTER ilkesi geri kilitlendi (31 Agu 2026)
        #
        # Upstream sadece MUTLAK yolu birakmisti: ref_width_m/ref_height_m/
        # ref_range_m -> normalized_bbox_size_at_range -> terminal_size, ve
        # _is_terminal(bbox_size) bunu mutlak esik olarak kullaniyordu.
        # Bu, "hedef 0.5 x 0.5 m" varsayimi tasir. HUNTER'da hedef boyutu
        # ASLA varsayilmaz (bbox = yakinlik vekili).
        #
        # Geri konan GORECELI yol, hc_air_1'in sahada uctugu kural:
        #     size_ratio = bbox_size / kilit anindaki bbox  >=  size_ratio
        # Kilit anindaki bbox olcumdur, varsayim degil.
        #
        # size_ratio > 0 ise GORECELI yol kazanir (HUNTER varsayilani: 2.0).
        # Mutlak yol sadece size_ratio <= 0 VE ref_* > 0 iken acilir; o
        # durumda acilista GORUNUR uyari basilir.
        # ---------------------------------------------------------------
        self.terminal_size_ratio = float(term.get("size_ratio", 0.0))
        self.terminal_ref_width_m = float(term.get("ref_width_m", 0.0))
        self.terminal_ref_height_m = float(term.get("ref_height_m", 0.0))
        self.terminal_ref_range_m = float(term.get("ref_range_m", 0.0))
        self.terminal_size = 0.0
        if self.terminal_size_ratio > 0.0:
            self.terminal_mode = "size_ratio"
        else:
            self.terminal_size = normalized_bbox_size_at_range(
                self.terminal_ref_width_m,
                self.terminal_ref_height_m,
                self.terminal_ref_range_m,
                image_width=float(DEFAULT_CAMERA_WIDTH),
                image_height=float(DEFAULT_CAMERA_HEIGHT),
                hfov_deg=float(DEFAULT_CAMERA_HFOV_DEG),
            )
            if self.terminal_size > 1e-9:
                self.terminal_mode = "abs_size"
                print(
                    "[HC_AIR_4] !!! UYARI: terminal esigi MUTLAK bbox modunda "
                    f"(size={self.terminal_size:.5f}) — "
                    f"{self.terminal_ref_width_m}x{self.terminal_ref_height_m} m "
                    f"hedef @ {self.terminal_ref_range_m} m VARSAYIMI. "
                    "HUNTER ilkesine aykiri; terminal.size_ratio kullan."
                )
            else:
                self.terminal_mode = "off"
                print(
                    "[HC_AIR_4] terminal bandi KAPALI "
                    "(size_ratio=0 ve ref_* verilmedi)."
                )
        self.terminal_climb_cap_mps = float(term.get("climb_cap_mps", 1.5))
        self.terminal_climb_cap_above_elev_deg = float(
            term.get("climb_cap_above_elev_deg", 1.0)
        )
        self.terminal_climb_cap_above_mps = float(
            term.get("climb_cap_above_mps", 6.0)
        )
        self.terminal_bypass_thr_above_elev_deg = float(
            term.get("bypass_thrust_cap_above_elev_deg", 1.0)
        )
        self.terminal_pitch_des_deg = float(term.get("pitch_des_deg", -10.0))
        self.terminal_pitch_bleed_deg_s = float(term.get("pitch_bleed_deg_s", 10.0))
        self.terminal_thrust_cap = float(term.get("thrust_cap", 0.85))

        self.lost_yaw_scan_offset = float(lost["yaw_scan_offset_deg"])
        self.lost_pitch_target_deg = float(lost["pitch_target_deg"])
        self.lost_yaw_kp_scale = float(lost["yaw_kp_scale"])
        self.lost_roll_cmd_scale = float(lost["roll_cmd_scale"])
        self.lost_climb_arrest_mps = float(lost.get("climb_arrest_mps", 0.8))
        self.lost_climb_sink_mps = float(lost.get("climb_sink_mps", 4.0))
        self.lost_climb_pitch_des_deg = float(
            lost.get("climb_pitch_des_deg", lost["pitch_target_deg"])
        )
        self.lost_climb_thrust_cap = float(lost.get("climb_thrust_cap", 0.85))

        self.bbox_rate_alpha = float(dyn["bbox_rate_alpha"])
        self.bbox_rate_lost_decay = float(dyn["bbox_rate_lost_decay"])
        self.default_dt_s = float(dyn["default_dt_s"])

        self.alpha_roll = float(sm["alpha_roll"])
        self.alpha_pitch = float(sm["alpha_pitch"])
        self.alpha_throttle = float(sm["alpha_throttle"])
        self.alpha_yaw = float(sm["alpha_yaw"])
        self.roll_slew_deg_s = float(sm["roll_slew_deg_s"])
        self.roll_slew_term_deg_s = float(
            sm.get("roll_slew_term_deg_s", sm["roll_slew_deg_s"])
        )
        self.pitch_slew_deg_s = float(sm["pitch_slew_deg_s"])
        self.yaw_slew_deg_s = float(sm["yaw_slew_deg_s"])
        self.throttle_rate_limit = float(sm["throttle_rate_limit_per_s"])
        self.throttle_rate_limit_dive = float(sm["throttle_rate_limit_dive_per_s"])

        self.limit_bank = float(al["max_bank_deg"])
        self.limit_pitch_min = float(al["pitch_des_min_deg"])
        self.limit_pitch_max = float(al["pitch_des_max_deg"])

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
        self._pitch_des_gov = float(self.pitch_des_cruise_deg)
        self._closure_filt = 0.0
        self._lost_t = 0.0
        self._lock_t = 0.0
        self._vz_int = 0.0
        self._vz_des_lost_hold = 0.0
        self._governor_active = 0.0
        self._chase_receding = 0.0
        self._lat_slow_active = 0.0
        self._alt_protect_active = 0.0
        self._ground_floor_active = 0.0
        self._ground_floor_pass = 0.0
        self._los_az_filt_dps = 0.0
        self._elev_prev: float | None = None
        self._v_adapt = 0.0
        self._d_elev_filt = 0.0
        self._yaw_initialized = False
        self._pitch_initialized = False
        self._overfly_active = 0.0
        self._overfly_prev = 0.0
        self._lost_climb_active = 0.0
        self._term_lat_active = 0.0
        self._terminal_active = 0.0
        self._terminal_climb_exempt = 0.0

        self.prev_action = np.array(
            [0.0, self.pitch_des_cruise_deg, self.hover_thrust, 0.0],
            dtype=np.float64,
        )
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
    def _wrap_deg(deg: float) -> float:
        x = (float(deg) + 180.0) % 360.0 - 180.0
        return x

    @staticmethod
    def _shortest_yaw_delta(target_deg: float, from_deg: float) -> float:
        return HCAir._wrap_deg(float(target_deg) - float(from_deg))

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

    def _ang_y_aim_deg(self, ang_y: float, bbox_h: float) -> float:
        """Vertical aim angle: gate TOP if enabled, else bbox center."""
        ay = float(ang_y)
        if not self.use_gate_top_reticle:
            return ay
        # Positive ang_y = below image center; top edge is above center → smaller ay.
        half_h = 0.5 * max(float(bbox_h), 0.0) * float(self.camera_vfov_deg)
        return ay - half_h

    def _pitch_for_reticle(self, pitch_deg: float) -> float:
        """
        Pitch used for horizon reticle. Floor at cruise so nose-down for speed
        does not move the altitude reticle (avoids pitch→sink death spiral).
        """
        return max(float(pitch_deg), float(self.pitch_des_cruise_deg))

    def _reticle_elev_deg(
        self,
        pitch_deg: float,
        ang_y_aim: float,
        camera_pitch_deg: float | None = None,
    ) -> float:
        """
        Elevation of gate-aim LOS above horizon (deg, +up).
        Zero ⇒ same altitude (aim point on horizontal ray).

        ang_y positive = below image center (bridge). optical = pitch - cam.
        elev = optical - ang_y_aim + bias. bias shifts null (est3 false climb).
        """
        cam = float(
            self.camera_pitch_default_deg
            if camera_pitch_deg is None
            else camera_pitch_deg
        )
        return (
            self._pitch_for_reticle(pitch_deg)
            - cam
            - float(ang_y_aim)
            + float(self.reticle_elev_bias_deg)
        )

    def _gate_elevation_deg(
        self,
        pitch_deg: float,
        ang_y: float,
        camera_pitch_deg: float | None = None,
    ) -> float:
        """Legacy camera-model elev (partial pitch transfer). Used for v_adapt."""
        del camera_pitch_deg
        return (
            float(self.cam_axis_offset_deg)
            + self.cam_pitch_transfer * float(pitch_deg)
            - float(ang_y)
        )

    def _los_path_elev_deg(
        self,
        pitch_deg: float,
        ang_y: float,
        camera_pitch_deg: float | None = None,
    ) -> float:
        """Alias: full inertial elev with raw pitch (debug)."""
        cam = float(
            self.camera_pitch_default_deg
            if camera_pitch_deg is None
            else camera_pitch_deg
        )
        return float(pitch_deg) - cam - float(ang_y)

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
        if bbox_valid < 0.5:
            decay = math.exp(-float(dt) / max(self.lost_vz_decay_tau_s, 1e-3))
            self._v_adapt *= decay
            self._elev_prev = None
            self._d_elev_filt = 0.0
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
        bbox_h: float = 0.0,
        phase: str = "cruise",
        size_ratio: float = 0.0,
        bbox_size: float = 0.0,
        alt_m: float | None = None,
        ang_x: float = 0.0,
        bbox_valid: float = 1.0,
    ) -> float:
        """
        Desired climb rate (up +). Cruise/terminal: null horizon reticle on gate top.
        Near + gate below reticle: overfly sink boost (est10).
        Terminal: climb capped (pass without balloon).
        Ground floor: privileged AGL sink clip (after overfly); pass-commit
        exempts hold so the aperture can be flown through.
        """
        ang_y_aim = self._ang_y_aim_deg(ang_y, bbox_h)
        elev = self._reticle_elev_deg(pitch_deg, ang_y_aim, camera_pitch_deg)
        v_eff = float(self.v_pursuit_ref_mps) + float(self._v_adapt)
        vz_des = (
            float(self.k_pursuit_n)
            * v_eff
            * math.tan(math.radians(elev))
        )
        vz_des = self._apply_overfly_sink(
            vz_des, elev, size_ratio, alt_m=alt_m,
            ang_x=ang_x, bbox_valid=bbox_valid,
        )
        # Never dive hard while far (before terminal size). size_ratio starts
        # at 1.0 on lock — do not use that as the far/near gate (est31 crash).
        if (not self._is_terminal(bbox_size)) and vz_des < 0.0:
            vz_des = max(vz_des, -float(self.far_sink_cap_mps))
        vz_des = self._apply_terminal_climb_cap(
            vz_des, bbox_size, elev_deg=elev,
        )
        vz_des = self._apply_ground_floor(
            vz_des, alt_m,
            size_ratio=size_ratio, ang_x=ang_x, bbox_valid=bbox_valid,
        )
        return float(np.clip(vz_des, -self.vz_des_max, self.vz_des_max))

    def _is_terminal(self, bbox_size: float) -> bool:
        """Terminal band. GORECELI yol oncelikli (HUNTER ilkesi)."""
        if self.terminal_size_ratio > 0.0:
            # _size_ratio = bbox_size / kilit anindaki bbox — olcum/olcum.
            return self._size_ratio(bbox_size) >= self.terminal_size_ratio
        thresh = float(self.terminal_size)
        if thresh <= 1e-9:
            return False
        return float(bbox_size) >= thresh

    def _terminal_gate_above(self, elev_deg: float) -> bool:
        """Gate above horizon reticle — still need climb (est8)."""
        return float(elev_deg) > float(self.terminal_climb_cap_above_elev_deg)

    def _apply_terminal_climb_cap(
        self,
        vz_des: float,
        bbox_size: float,
        *,
        elev_deg: float = 0.0,
    ) -> float:
        """Near gate: limit climb when level/below; allow climb when gate above."""
        if not self._is_terminal(bbox_size):
            return float(vz_des)
        self._terminal_active = 1.0
        if float(vz_des) <= 0.0:
            return float(vz_des)
        if self._terminal_gate_above(elev_deg):
            self._terminal_climb_exempt = 1.0
            cap_above = float(self.terminal_climb_cap_above_mps)
            if cap_above <= 0.0:
                return float(vz_des)
            return min(float(vz_des), cap_above)
        return min(float(vz_des), float(self.terminal_climb_cap_mps))

    def _bypass_terminal_thrust(self, elev_deg: float = 0.0) -> bool:
        if self._ground_floor_bypass_terminal_thrust():
            return True
        if self._terminal_climb_exempt > 0.5:
            return True
        return float(elev_deg) > float(self.terminal_bypass_thr_above_elev_deg)

    def _clamp_vz_des_lat_slow(
        self, vz_des: float, elev_deg: float = 0.0,
    ) -> float:
        """est18: no climb while lat_slow; est4: mild climb if gate well above."""
        if self._lat_slow_active <= 0.5:
            return float(vz_des)
        if float(elev_deg) > self.lat_slow_climb_elev_deg:
            return min(float(self.lat_slow_climb_cap_mps), float(vz_des))
        return min(0.0, float(vz_des))

    def _pass_commit(
        self,
        size_ratio: float,
        ang_x: float,
        bbox_valid: float,
    ) -> bool:
        """Near + on-aim: allow diving through the gate (hold floor off)."""
        if float(bbox_valid) < 0.5:
            return False
        if float(size_ratio) < float(self.ground_floor_pass_size_ratio):
            return False
        return abs(float(ang_x)) <= float(self.ground_floor_pass_aim_deg)

    def _ground_floor_in_band(self, alt_m: float | None) -> bool:
        if not self.ground_floor_enabled or alt_m is None:
            return False
        return float(alt_m) < float(self.ground_floor_soft_start_m)

    def _ground_floor_hold_active(
        self,
        alt_m: float | None,
        *,
        size_ratio: float = 0.0,
        ang_x: float = 0.0,
        bbox_valid: float = 0.0,
    ) -> bool:
        """Climb/pitch hold — not during aimed pass (est50 overfly fix)."""
        if not self._ground_floor_in_band(alt_m):
            return False
        if self._pass_commit(size_ratio, ang_x, bbox_valid):
            return False
        return True

    def _ground_floor_blocks_overfly(
        self,
        alt_m: float | None,
        *,
        size_ratio: float = 0.0,
        ang_x: float = 0.0,
        bbox_valid: float = 0.0,
    ) -> bool:
        if not self.ground_floor_override_overfly:
            return False
        # During pass, allow overfly sink to help thread the aperture.
        if self._pass_commit(size_ratio, ang_x, bbox_valid):
            return False
        return self._ground_floor_in_band(alt_m)

    def _ground_floor_bypass_terminal_thrust(self) -> bool:
        return (
            self.ground_floor_bypass_term_thr
            and self._ground_floor_active > 0.5
        )

    def _clip_agl_band(
        self,
        vz_des: float,
        alt_m: float,
        *,
        floor: float,
        soft: float,
        max_sink: float,
        climb: float,
    ) -> float:
        """Blend sink clip + climb arrest between soft and floor."""
        if soft <= floor:
            soft = floor + 1e-3
        vz = float(vz_des)
        if alt_m >= soft:
            return vz
        self._ground_floor_active = 1.0
        if alt_m <= floor:
            out = max(vz, -max_sink)
            out = max(out, climb)
            return float(out)
        scale = (alt_m - floor) / (soft - floor)
        vz_hard = max(vz, -max_sink)
        out = scale * vz + (1.0 - scale) * vz_hard
        out = max(out, climb * (1.0 - scale))
        return float(out)

    def _apply_ground_floor(
        self,
        vz_des: float,
        alt_m: float | None,
        *,
        size_ratio: float = 0.0,
        ang_x: float = 0.0,
        bbox_valid: float = 0.0,
    ) -> float:
        """AGL hold, or crash-only floor during aimed pass commit."""
        if not self.ground_floor_enabled or alt_m is None:
            return float(vz_des)
        alt = float(alt_m)
        passing = self._pass_commit(size_ratio, ang_x, bbox_valid)
        if passing:
            self._ground_floor_pass = 1.0
            return self._clip_agl_band(
                float(vz_des),
                alt,
                floor=float(self.ground_floor_crash_m),
                soft=float(self.ground_floor_crash_soft_m),
                max_sink=0.0,
                climb=0.0,
            )
        return self._clip_agl_band(
            float(vz_des),
            alt,
            floor=float(self.ground_floor_m),
            soft=float(self.ground_floor_soft_start_m),
            max_sink=max(0.0, float(self.ground_floor_max_sink_near_mps)),
            climb=max(0.0, float(self.ground_floor_climb_arrest_mps)),
        )

    def _overfly_condition(
        self,
        elev_deg: float,
        size_ratio: float,
        alt_m: float | None = None,
        ang_x: float = 0.0,
        bbox_valid: float = 1.0,
    ) -> bool:
        if self._ground_floor_blocks_overfly(
            alt_m, size_ratio=size_ratio, ang_x=ang_x, bbox_valid=bbox_valid,
        ):
            return False
        return (
            float(size_ratio) >= self.overfly_size_ratio
            and float(elev_deg) < self.overfly_elev_deg
        )

    def _apply_overfly_sink(
        self,
        vz_des: float,
        elev_deg: float,
        size_ratio: float,
        alt_m: float | None = None,
        ang_x: float = 0.0,
        bbox_valid: float = 1.0,
    ) -> float:
        """Near + gate below reticle → amplify / floor sink (est10 overfly)."""
        if not self._overfly_condition(
            elev_deg, size_ratio, alt_m=alt_m,
            ang_x=ang_x, bbox_valid=bbox_valid,
        ):
            return float(vz_des)
        out = float(vz_des)
        if out < 0.0:
            out *= float(self.overfly_sink_boost)
        depth = min(1.0, (self.overfly_elev_deg - float(elev_deg)) / 5.0)
        out = min(out, -float(self.overfly_min_sink_mps) * depth)
        return out

    def _crab_range_scale(self, size_ratio: float) -> float:
        """Amplify crab when gate is far (small size_ratio)."""
        sr = max(float(size_ratio), float(self.crab_far_size_floor))
        ref = max(float(self.crab_far_ref_size_ratio), 1e-3)
        if sr >= ref:
            return 1.0
        return float(min(self.crab_far_scale_max, ref / sr))

    def _pitch_thrust_fade(self, elev_cam_deg: float) -> float:
        """1 at/above 0 elev_cam; 0 at/below sink_ff_fade_elev_deg."""
        fade_lo = float(self.sink_ff_fade_elev_deg)
        if fade_lo >= 0.0:
            return 1.0 if elev_cam_deg >= 0.0 else 0.0
        if elev_cam_deg >= 0.0:
            return 1.0
        if elev_cam_deg <= fade_lo:
            return 0.0
        return float(elev_cam_deg / fade_lo)

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

    def _thrust_saturated(self) -> bool:
        thr = float(self.prev_action[2])
        return thr >= self.alt_protect_thrust_sat_frac * self.thrust_cmd_max

    def _altitude_protect_needed(
        self,
        *,
        priv_ok: bool,
        vz_up: float,
        vz_des: float = 0.0,
    ) -> bool:
        """Protect only when sinking harder than commanded (or thr-sat lag)."""
        if not priv_ok:
            return False
        vz_u = float(vz_up)
        vz_d = float(vz_des)
        if vz_u < vz_d - self.alt_protect_sink_mps:
            return True
        if (
            self._thrust_saturated()
            and vz_u < vz_d - self.alt_protect_thrust_sat_vz_margin
        ):
            return True
        return False

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
        bbox_h: float = 0.0,
        alt_m: float | None = None,
    ) -> None:
        self._governor_active = 0.0
        self._chase_receding = 0.0
        self._lat_slow_active = 0.0
        self._alt_protect_active = 0.0
        self._ground_floor_active = 0.0
        self._ground_floor_pass = 0.0
        self._overfly_active = 0.0
        self._terminal_active = 0.0
        self._terminal_climb_exempt = 0.0
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

        size_ratio = self._size_ratio(bbox_size)
        vz_des_est = self._estimate_vz_des(
            ang_y=ang_y,
            pitch_deg=pitch_deg,
            camera_pitch_deg=camera_pitch_deg,
            bbox_h=bbox_h,
            phase=phase,
            size_ratio=size_ratio,
            bbox_size=bbox_size,
            alt_m=alt_m,
            ang_x=ang_x,
            bbox_valid=bbox_valid,
        )
        ang_y_aim = self._ang_y_aim_deg(ang_y, bbox_h)
        elev_ret = self._reticle_elev_deg(pitch_deg, ang_y_aim, camera_pitch_deg)
        overfly = self._overfly_condition(
            elev_ret, size_ratio, alt_m=alt_m,
            ang_x=ang_x, bbox_valid=bbox_valid,
        )
        if overfly:
            self._overfly_active = 1.0

        receding = float(self._closure_filt) < self.governor_recede_threshold
        self._chase_receding = 1.0 if receding else 0.0
        vz_err_lim = (
            self.governor_chase_vert_err_max if receding else self.governor_vz_err_max
        )
        vert_ok = abs(float(vz_des_est) - float(vz_up)) <= vz_err_lim

        hold = self._ground_floor_hold_active(
            alt_m, size_ratio=size_ratio, ang_x=ang_x, bbox_valid=bbox_valid,
        )

        if (
            not overfly
            and not vert_ok
            and not receding
            and float(self._pitch_des_gov) < self.pitch_des_cruise_deg
        ):
            # Bleed nose-up toward cruise (min clamps at cruise when more nose-down).
            self._pitch_des_gov = min(
                self.pitch_des_cruise_deg,
                float(self._pitch_des_gov) + self.k_pitch_vert_brake * dt,
            )

        if overfly:
            if float(self._pitch_des_gov) > self.overfly_pitch_des_deg:
                self._pitch_des_gov = max(
                    self.overfly_pitch_des_deg,
                    float(self._pitch_des_gov) - self.overfly_pitch_bleed_deg_s * dt,
                )
        elif (not hold) and self._is_terminal(bbox_size):
            self._terminal_active = 1.0
            # Mild nose-down for pass; skip if lat_slow is bleeding nose-up.
            if (
                abs(float(ang_x)) <= self.lat_slow_aim_deg
                and float(self._pitch_des_gov) > self.terminal_pitch_des_deg
            ):
                self._pitch_des_gov = max(
                    self.terminal_pitch_des_deg,
                    float(self._pitch_des_gov) - self.terminal_pitch_bleed_deg_s * dt,
                )

        # Hold band only (not pass-commit): bleed pitch toward hold_des
        # (above cruise) so a ballistic sink can be arrested (est31).
        if hold and self.ground_floor_pitch_bleed_deg_s > 1e-6:
            hold_des = float(self.ground_floor_pitch_hold_des_deg)
            if float(self._pitch_des_gov) < hold_des:
                self._pitch_des_gov = min(
                    hold_des,
                    float(self._pitch_des_gov)
                    + float(self.ground_floor_pitch_bleed_deg_s) * dt,
                )
            self._ground_floor_active = 1.0

        # est15: lat_slow bled nose-up while overfly needed sink — skip if overfly.
        # est18: keep active while off-aim (not only while bleeding) so climb
        # clamp stays on after pitch parks at lat_slow_des.
        lat_slow = (
            (not overfly)
            and (not hold)
            and size_ratio >= self.lat_slow_size_ratio
            and abs(float(ang_x)) > self.lat_slow_aim_deg
        )
        if lat_slow:
            self._lat_slow_active = 1.0
            if float(self._pitch_des_gov) < self.lat_slow_des_deg:
                self._pitch_des_gov = min(
                    self.lat_slow_des_deg,
                    float(self._pitch_des_gov) + self.k_pitch_lat_slow * dt,
                )

        alt_protect = self._altitude_protect_needed(
            priv_ok=priv_ok, vz_up=vz_up, vz_des=vz_des_est,
        )
        # Don't nose-up-protect while intentionally sinking through an overfly.
        if overfly and float(vz_des_est) < -0.5:
            alt_protect = False
        if alt_protect:
            if float(self._pitch_des_gov) < self.pitch_des_cruise_deg:
                self._pitch_des_gov = min(
                    self.pitch_des_cruise_deg,
                    float(self._pitch_des_gov) + self.k_pitch_alt_protect * dt,
                )
            self._alt_protect_active = 1.0

        gates_ok = self._governor_gates_ok(
            ang_x=ang_x,
            pitch_deg=pitch_deg,
            priv_ok=priv_ok,
            receding=receding,
        )

        gnd_band = hold
        err = self.closure_target - float(self._closure_filt)
        if (
            err > 0.0
            and gates_ok
            and vert_ok
            and not lat_slow
            and not alt_protect
            and not gnd_band
        ):
            dp = min(self.k_pitch_ramp_up * err, self.pitch_des_accel_max)
            self._pitch_des_gov = float(self._pitch_des_gov) - dp * dt
            self._governor_active = 1.0
        elif (
            err < 0.0
            and float(bbox_size) < self.governor_no_slowdown_size
            and not lat_slow
            and not gnd_band
        ):
            self._pitch_des_gov = (
                float(self._pitch_des_gov) - self.k_pitch_ramp_down * err * dt
            )
            self._pitch_des_gov = min(
                float(self.pitch_des_cruise_deg),
                float(self._pitch_des_gov),
            )

        self._pitch_des_gov = float(np.clip(
            self._pitch_des_gov, self.pitch_des_min_deg, self.pitch_des_max_deg,
        ))

    def _pitch_des_cmd(
        self,
        *,
        priv_ok: bool,
        vz_up: float = 0.0,
        vz_des_est: float | None = None,
    ) -> float:
        """Attitude-hold pitch setpoint (deg)."""
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
                    p_min = float(self.pitch_des_min_relaxed_deg)
            if (
                self._alt_protect_active > 0.5
                and self.alt_protect_sat_floor_cruise
                and self._thrust_saturated()
            ):
                p_min = max(p_min, float(self.pitch_des_cruise_deg))
            elif self._alt_protect_active > 0.5:
                p_min = max(p_min, float(self.pitch_des_min_relaxed_deg))
            return float(np.clip(pitch_des, p_min, p_max))
        return float(self.pitch_des_fallback_deg)

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
        bbox_h: float = 0.0,
        roll_cmd_deg: float | None = None,
        pitch_cmd_deg: float | None = None,
        size_ratio: float = 0.0,
        bbox_size: float = 0.0,
        alt_m: float | None = None,
        ang_x: float = 0.0,
    ) -> tuple[float, float, float, float, float, float]:
        # Tilt FF: commanded attitude so collective rises with dive.
        # Overfly/sink: bank tilt-FF fights thr cut (est16 thr=+0.30) — wings-level FF.
        roll_ff = float(roll_deg if roll_cmd_deg is None else roll_cmd_deg)
        pitch_ff = float(pitch_deg if pitch_cmd_deg is None else pitch_cmd_deg)
        ang_y_aim = self._ang_y_aim_deg(ang_y, bbox_h)
        elev_ret = self._reticle_elev_deg(pitch_deg, ang_y_aim, camera_pitch_deg)
        overfly_now = (
            bbox_valid > 0.5
            and self._overfly_condition(
                elev_ret, size_ratio, alt_m=alt_m,
                ang_x=ang_x, bbox_valid=bbox_valid,
            )
        )
        if overfly_now:
            if (
                self.overfly_reset_vz_int
                and self._overfly_prev < 0.5
            ):
                self._vz_int = 0.0
            self._overfly_active = 1.0
        self._overfly_prev = 1.0 if overfly_now else 0.0

        roll_for_tilt = (
            0.0
            if (overfly_now and self.overfly_zero_tilt_ff)
            else roll_ff
        )
        t_ff, tilt_comp = self._tilt_ff(roll_for_tilt, pitch_ff)
        pitch_extra = max(0.0, self.pitch_des_cruise_deg - pitch_ff)
        fade = self._pitch_thrust_fade(elev_ret)
        if overfly_now:
            fade = 0.0  # don't add nose-down collective while sinking through
        t_ff += self.pitch_thrust_gain * pitch_extra * fade
        elev_deg = elev_ret
        t_fb = 0.0
        vz_des = 0.0

        if bbox_valid > 0.5 and priv_ok:
            vz_des = self._estimate_vz_des(
                ang_y=ang_y,
                pitch_deg=pitch_deg,
                camera_pitch_deg=camera_pitch_deg,
                bbox_h=bbox_h,
                phase=phase,
                size_ratio=size_ratio,
                bbox_size=bbox_size,
                alt_m=alt_m,
                ang_x=ang_x,
                bbox_valid=bbox_valid,
            )
            vz_des = self._clamp_vz_des_lat_slow(vz_des, elev_deg=elev_ret)
            vz_des = self._apply_ground_floor(
                vz_des, alt_m,
                size_ratio=size_ratio, ang_x=ang_x, bbox_valid=bbox_valid,
            )
            if overfly_now:
                self._overfly_active = 1.0
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
            # est13: lost sonrası thr hover'a dönüp tırmanış sürdü — climb arrest.
            if float(vz_up) > self.lost_climb_arrest_mps:
                self._vz_des_lost_hold = min(
                    float(self._vz_des_lost_hold),
                    -float(self.lost_climb_sink_mps),
                )
            # Lost: no pass commit → full hold floor / climb arrest.
            vz_des = self._apply_ground_floor(
                float(self._vz_des_lost_hold), alt_m,
                size_ratio=0.0, ang_x=0.0, bbox_valid=0.0,
            )
            self._vz_des_lost_hold = float(vz_des)
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

        thrust_min = float(self.thrust_cmd_min)
        if (
            self._overfly_active > 0.5
            and float(vz_up) > self.overfly_bank_shed_climb_mps
        ):
            thrust_min = min(thrust_min, float(self.overfly_thrust_cmd_min))
        thrust = float(np.clip(
            t_ff + t_fb,
            thrust_min,
            self.thrust_cmd_max,
        ))
        if self._is_terminal(bbox_size) and bbox_valid > 0.5:
            self._terminal_active = 1.0
            if not self._bypass_terminal_thrust(elev_ret):
                thrust = min(float(thrust), float(self.terminal_thrust_cap))
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
        self,
        bbox_valid: float,
        bbox_size: float,
        ang_x: float,
        *,
        reticle_elev_deg: float = 0.0,
    ) -> str:
        """lost | cruise. Dive latch removed — terminal band handles near-gate."""
        del bbox_size, ang_x, reticle_elev_deg  # kept for call-site compat
        if bbox_valid < 0.5:
            return "lost"
        return "cruise"

    def _yaw_attitude_cmd(
        self,
        ang_x: float,
        yaw_meas_deg: float,
        scale: float = 1.0,
    ) -> tuple[float, float, float]:
        ax = self._deadband(ang_x, self.ang_x_deadband_deg)
        offset = float(np.clip(
            self.kp_yaw_offset * ax * scale,
            -self.max_yaw_offset * scale,
            self.max_yaw_offset * scale,
        ))
        yaw_des = self._wrap_deg(float(yaw_meas_deg) + offset)
        return yaw_des, ax, offset

    def _soft_sat_los_dps(self, los_dps: float) -> float:
        """Soft-limit LOS rate for crab (GPS-free). 0 ref disables."""
        ref = float(self.los_crab_soft_sat_dps)
        if ref <= 1e-6:
            return float(los_dps)
        return float(ref * math.tanh(float(los_dps) / ref))

    def _term_boost_allowed(self, ang_x: float) -> bool:
        if not self.crab_term_require_off_aim:
            return True
        return abs(float(ang_x)) > float(self.crab_term_aim_deg)

    def _is_term_lateral(self, bbox_size: float) -> bool:
        return self._is_terminal(bbox_size)

    def _bank_limit_deg(self, bbox_size: float) -> float:
        if self._is_term_lateral(bbox_size):
            return float(max(self.max_bank_deg, self.max_bank_term_deg))
        return float(self.max_bank_deg)

    def _roll_attitude_cmd(
        self,
        bbox_valid: float,
        los_az_filt_dps: float,
        scale: float = 1.0,
        *,
        ang_x: float = 0.0,
        size_ratio: float = 0.0,
        bbox_size: float = 0.0,
    ) -> tuple[float, float, float]:
        """Returns roll_des_deg, crab_bank_deg, ang_x_bank_deg."""
        crab = 0.0
        trim = 0.0
        bank_lim = self._bank_limit_deg(bbox_size)
        if bbox_valid > 0.5:
            range_scale = self._crab_range_scale(size_ratio)
            near = float(size_ratio) >= self.crab_near_size_ratio
            term = self._is_term_lateral(bbox_size)
            db = (
                self.crab_near_deadband_dps if near else self.crab_los_deadband_dps
            )
            los_rate = self._deadband(float(los_az_filt_dps), db)
            los_rate = self._soft_sat_los_dps(los_rate)
            crab = self.kp_crab_bank * los_rate * range_scale
            if near:
                crab *= float(self.crab_near_scale)
            term_boost = term and self._term_boost_allowed(ang_x)
            if term:
                self._term_lat_active = 1.0
            if term_boost:
                crab *= float(self.crab_term_scale)
            if float(size_ratio) >= self.ang_x_bank_size_ratio:
                ax = self._deadband(ang_x, self.ang_x_deadband_deg)
                trim = self.kp_ang_x_bank * ax
                if term_boost:
                    trim *= float(self.ang_x_term_scale)
        roll_des = float(np.clip(
            (crab + trim) * scale,
            -bank_lim * scale,
            bank_lim * scale,
        ))
        return roll_des, crab, trim

    def _shed_bank_for_overfly_climb(
        self,
        roll_des: float,
        *,
        vz_up: float,
        vz_des: float,
        ang_x: float = 0.0,
    ) -> float:
        """Reduce bank when climbing against a sink command (est13 thr-floor climb)."""
        if self.overfly_bank_shed_frac <= 1e-6:
            return float(roll_des)
        if self._overfly_active < 0.5:
            return float(roll_des)
        # Crossing gate: keep full bank when laterally off (est1).
        if abs(float(ang_x)) >= self.overfly_bank_shed_ax_min_deg > 0.0:
            return float(roll_des)
        if float(vz_up) < self.overfly_bank_shed_climb_mps:
            return float(roll_des)
        if float(vz_des) > -0.5:
            return float(roll_des)
        keep = max(0.0, 1.0 - float(self.overfly_bank_shed_frac))
        return float(roll_des) * keep

    def _shed_bank_for_climb(
        self,
        roll_des: float,
        *,
        vz_des: float,
        ang_x: float = 0.0,
    ) -> float:
        """
        High-gate climb: soften bank when on-aim so crab doesn't fight climb
        (est4 ±48 flip). Sink/dive (vz_des low) untouched — ground gates OK.
        Off-aim keeps full bank for lateral catch-up. GPS-free (vz_des only).
        """
        if self.climb_bank_shed_frac <= 1e-6:
            return float(roll_des)
        if float(vz_des) < float(self.climb_bank_shed_vz_des_mps):
            return float(roll_des)
        if abs(float(ang_x)) >= float(self.climb_bank_shed_ax_min_deg):
            return float(roll_des)
        keep = max(0.0, 1.0 - float(self.climb_bank_shed_frac))
        return float(roll_des) * keep

    def _smooth(self, target: np.ndarray, dt: float) -> np.ndarray:
        alphas = np.array([
            self.alpha_roll, self.alpha_pitch, self.alpha_throttle, self.alpha_yaw,
        ], dtype=np.float64)
        # Unwrap yaw target toward previous yaw for EMA.
        yaw_tgt = float(target[3])
        yaw_prev = float(self.prev_action[3])
        yaw_unwrapped = yaw_prev + self._shortest_yaw_delta(yaw_tgt, yaw_prev)
        tgt = target.copy()
        tgt[3] = yaw_unwrapped

        smoothed = alphas * tgt + (1.0 - alphas) * self.prev_action
        thr_rate = float(self.throttle_rate_limit)
        if (
            float(tgt[1]) < self.pitch_des_cruise_deg - 0.5
            or self._alt_protect_active > 0.5
            or self._ground_floor_active > 0.5
            or self._overfly_active > 0.5
            or self._lost_climb_active > 0.5
            or self._terminal_active > 0.5
        ):
            thr_rate = max(thr_rate, float(self.throttle_rate_limit_dive))
        # Term uses its own slew (may be lower than mid — no longer force faster).
        if self._term_lat_active > 0.5:
            roll_slew = float(self.roll_slew_term_deg_s)
        else:
            roll_slew = float(self.roll_slew_deg_s)
        slews = np.array([
            roll_slew,
            self.pitch_slew_deg_s,
            thr_rate,
            self.yaw_slew_deg_s,
        ], dtype=np.float64) * float(dt)
        delta = np.clip(smoothed - self.prev_action, -slews, slews)
        out = self.prev_action + delta
        out[0] = float(np.clip(out[0], -self.limit_bank, self.limit_bank))
        out[1] = float(np.clip(out[1], self.limit_pitch_min, self.limit_pitch_max))
        thr_lo = float(self.thrust_cmd_min)
        if self._overfly_active > 0.5:
            thr_lo = min(thr_lo, float(self.overfly_thrust_cmd_min))
        out[2] = float(np.clip(out[2], thr_lo, self.thrust_cmd_max))
        if self._lost_climb_active > 0.5:
            out[2] = min(float(out[2]), float(self.lost_climb_thrust_cap))
        if (
            self._terminal_active > 0.5
            and not self._bypass_terminal_thrust()
        ):
            out[2] = min(float(out[2]), float(self.terminal_thrust_cap))
        out[3] = self._wrap_deg(out[3])
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
        yaw_meas = float(obs[8])
        yaw_rate = float(obs[11])
        bbox_valid = float(obs[13])
        camera_pitch_deg = camera_pitch_from_obs(
            obs, default_deg=self.camera_pitch_default_deg,
        )

        if not self._yaw_initialized:
            self.prev_action[3] = yaw_meas
            self._yaw_initialized = True

        # -------------------------------------------------------------------
        # PITCH TOHUM YAMASI — 25 Agu 2026 (hc_air_1 + hc_air_2'den tasindi)
        #
        # reset() pitch'i cruise'a tohumluyor. Burun-asagi angajmanda ilk
        # karede +18...+25 derece "kendini duzlestirme" sicramasi cikiyor,
        # hedef kadraj dibine dusuyor, track kayboluyor. Saha logu
        # 102910/103703/081522, 8/8 giriste olculdu, deterministik.
        #
        # Cozum yukaridaki _yaw_initialized deseninin pitch kopyasi: ilk
        # karede _pitch_des_gov ve prev_action[1] OLCULEN pitch'ten baslar
        # (des bandina kirpilarak), cruise'a kendi slew'iyle yurur.
        # SIFIR deger degisikligi — sadece baslangic noktasi.
        #
        # UYARI: --att-pitch-max kelepcesi bu tohumu sender'da EZER ve
        # sicrama geri gelir. Kelepceyi bu teacher'la KULLANMA.
        # -------------------------------------------------------------------
        if not self._pitch_initialized:
            seed = float(np.clip(
                pitch_deg, self.limit_pitch_min, self.limit_pitch_max,
            ))
            self._pitch_des_gov = seed
            self.prev_action[1] = seed
            self._pitch_initialized = True

        priv_ok, alt, vz_up = self._parse_privileged(privileged)
        alt_m = float(alt) if priv_ok else None

        self.step_count += 1
        bbox_area = bbox_w * bbox_h
        bbox_size = math.sqrt(max(bbox_area, 1e-10))
        self._update_frame_dynamics(ang_x, ang_y, bbox_size, bbox_valid, inv_dt)

        los_az_dps = 0.0
        if bbox_valid > 0.5:
            los_az_dps = self.d_ang_x * inv_dt + float(yaw_rate)
        los_az_filt = self._filter_los_az(los_az_dps, bbox_valid)

        ang_y_aim = self._ang_y_aim_deg(ang_y, bbox_h) if bbox_valid > 0.5 else 0.0
        reticle_elev = (
            self._reticle_elev_deg(pitch_deg, ang_y_aim, camera_pitch_deg)
            if bbox_valid > 0.5
            else 0.0
        )
        phase = self._select_phase(
            bbox_valid, bbox_size, ang_x, reticle_elev_deg=reticle_elev,
        )
        self.current_phase = phase
        size_ratio = self._size_ratio(bbox_size) if bbox_valid > 0.5 else 0.0
        self._term_lat_active = 0.0
        self._lost_climb_active = 0.0
        self._terminal_active = 0.0
        self._terminal_climb_exempt = 0.0
        elev_for_lead = reticle_elev
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
            bbox_h=bbox_h,
            alt_m=alt_m,
        )

        elev_deg = 0.0
        t_ff = t_fb = vz_des = 0.0
        roll_crab = 0.0
        roll_trim = 0.0
        yaw_offset = 0.0

        if bbox_valid < 0.5:
            mem_x = 0.0 if self.prev_ang_x is None else float(self.prev_ang_x)
            yaw_des, ax_used, yaw_offset = self._yaw_attitude_cmd(
                mem_x, yaw_meas, scale=self.lost_yaw_kp_scale,
            )
            yaw_des = self._wrap_deg(
                yaw_meas + float(np.clip(
                    yaw_offset, -self.lost_yaw_scan_offset, self.lost_yaw_scan_offset,
                ))
            )
            roll_des, roll_crab, roll_trim = self._roll_attitude_cmd(
                bbox_valid, los_az_filt, scale=self.lost_roll_cmd_scale,
                ang_x=mem_x, size_ratio=0.0, bbox_size=0.0,
            )
            lost_climbing = priv_ok and float(vz_up) > self.lost_climb_arrest_mps
            if lost_climbing:
                self._lost_climb_active = 1.0
            if priv_ok:
                vz_des_est = self._estimate_vz_des(
                    ang_y=0.0,
                    pitch_deg=pitch_deg,
                    camera_pitch_deg=camera_pitch_deg,
                    bbox_h=0.0,
                    phase=phase,
                    size_ratio=0.0,
                    bbox_size=0.0,
                    alt_m=alt_m,
                    ang_x=0.0,
                    bbox_valid=0.0,
                )
                pitch_des = self._pitch_des_cmd(
                    priv_ok=True, vz_up=vz_up, vz_des_est=vz_des_est,
                )
                if lost_climbing:
                    pitch_des = min(
                        float(pitch_des), float(self.lost_climb_pitch_des_deg),
                    )
            else:
                pitch_des = float(self.lost_pitch_target_deg)
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
                bbox_h=0.0,
                roll_cmd_deg=roll_des,
                pitch_cmd_deg=pitch_des,
                size_ratio=0.0,
                bbox_size=0.0,
                alt_m=alt_m,
                ang_x=0.0,
            )
            if lost_climbing:
                throttle_cmd = min(
                    float(throttle_cmd), float(self.lost_climb_thrust_cap),
                )
            ang_y_err = 0.0
        else:
            if self._overfly_condition(
                reticle_elev, size_ratio, alt_m=alt_m,
                ang_x=ang_x, bbox_valid=bbox_valid,
            ):
                self._overfly_active = 1.0
            roll_des, roll_crab, roll_trim = self._roll_attitude_cmd(
                bbox_valid, los_az_filt,
                ang_x=ang_x, size_ratio=size_ratio, bbox_size=bbox_size,
            )
            vz_des_est = (
                self._estimate_vz_des(
                    ang_y=ang_y,
                    pitch_deg=pitch_deg,
                    camera_pitch_deg=camera_pitch_deg,
                    bbox_h=bbox_h,
                    phase=phase,
                    size_ratio=size_ratio,
                    bbox_size=bbox_size,
                    alt_m=alt_m,
                    ang_x=ang_x,
                    bbox_valid=bbox_valid,
                )
                if priv_ok
                else None
            )
            if priv_ok and vz_des_est is not None:
                vz_des_est = self._clamp_vz_des_lat_slow(
                    float(vz_des_est), elev_deg=reticle_elev,
                )
                vz_des_est = self._apply_ground_floor(
                    float(vz_des_est), alt_m,
                    size_ratio=size_ratio, ang_x=ang_x, bbox_valid=bbox_valid,
                )
                roll_des = self._shed_bank_for_climb(
                    roll_des,
                    vz_des=float(vz_des_est),
                    ang_x=ang_x,
                )
                roll_des = self._shed_bank_for_overfly_climb(
                    roll_des,
                    vz_up=vz_up,
                    vz_des=float(vz_des_est),
                    ang_x=ang_x,
                )
            pitch_des = self._pitch_des_cmd(
                priv_ok=priv_ok, vz_up=vz_up, vz_des_est=vz_des_est,
            )
            yaw_scale = (
                float(self.yaw_term_scale)
                if self._is_term_lateral(bbox_size)
                else 1.0
            )
            yaw_des, ax_used, yaw_offset = self._yaw_attitude_cmd(
                ang_x, yaw_meas, scale=yaw_scale,
            )
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
                bbox_h=bbox_h,
                roll_cmd_deg=roll_des,
                pitch_cmd_deg=pitch_des,
                size_ratio=size_ratio,
                bbox_size=bbox_size,
                alt_m=alt_m,
                ang_x=ang_x,
            )
            ang_y_err = float(ang_y_aim) - (
                self._pitch_for_reticle(pitch_deg) - float(camera_pitch_deg)
            )

        target = np.array(
            [roll_des, pitch_des, throttle_cmd, yaw_des], dtype=np.float64,
        )
        action = self._smooth(target, dt)
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
            "bbox_size": float(bbox_size),
            "bbox_size_ratio": float(size_ratio),
            "terminal_size": float(self.terminal_size),
            "pitch_gain_scale": 1.0,
            "chase_blend": float(self._governor_active),
            "chase_receding": float(self._chase_receding),
            "lat_slow_active": float(self._lat_slow_active),
            "alt_protect_active": float(self._alt_protect_active),
            "ground_floor_active": float(self._ground_floor_active),
            "ground_floor_pass": float(self._ground_floor_pass),
            "overfly_active": float(self._overfly_active),
            "term_lat_active": float(self._term_lat_active),
            "lost_climb_active": float(self._lost_climb_active),
            "terminal_active": float(self._terminal_active),
            "terminal_climb_exempt": float(self._terminal_climb_exempt),
            "roll_lat_cmd": float(roll_crab),
            "roll_level_cmd": float(roll_des),
            "roll_kp_scale": 1.0,
            "roll_phase_gain": 1.0,
            "roll_rate_limit_applied": float(
                self.roll_slew_term_deg_s
                if self._term_lat_active > 0.5
                else self.roll_slew_deg_s
            ),
            "ang_x_ctrl": float(ax_used),
            "ang_x_spike_filtered": 0.0,
            "lateral_throttle_factor": 0.0,
            "target_bank_deg": float(roll_des),
            "los_rate_az_rad": float(math.radians(los_az_dps)),
            "los_rate_el_rad": 0.0,
            "los_rate_az_filt_rad": float(math.radians(los_az_filt)),
            "los_rate_az_dps": float(los_az_dps),
            "los_rate_el_dps": 0.0,
            "los_rate_az_filt_dps": float(los_az_filt),
            "roll_pn_cmd": 0.0,
            "roll_vlat_bias": float(roll_crab),
            "roll_ang_trim": float(roll_trim),
            "est_V_lat": float(los_az_filt),
            "est_V_forward": float(self.v_pursuit_ref_mps),
            "est_V_c": float(self._pitch_des_gov),
            "est_Z": float(elev_deg),
            "est_tau": float(pitch_des),
            "est_healthy": float(priv_ok),
            "est_range_ratio": float(self._closure_filt),
            "roll_level_scale": 1.0,
            "bank_clamped": float(
                1.0
                if abs(roll_des) >= self._bank_limit_deg(bbox_size) - 1e-3
                else 0.0
            ),
            "roll_cmd_raw": float(roll_des),
            "pitch_cmd_raw": float(pitch_des),
            "throttle_cmd_raw": float(throttle_cmd),
            "yaw_cmd_raw": float(yaw_des),
            "yaw_offset_deg": float(yaw_offset),
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
            "dive_committed": 0.0,
            "forward_dive_scale": float(self._terminal_active),
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
            "control_mode": "attitude_hold",
        }
        self._last_output = action.astype(np.float32)
        return self._last_output

    def reset(self):
        self.prev_action = np.array(
            [0.0, self.pitch_des_cruise_deg, self.hover_thrust, 0.0],
            dtype=np.float64,
        )
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
        self._pitch_des_gov = float(self.pitch_des_cruise_deg)
        self._closure_filt = 0.0
        self._lost_t = 0.0
        self._lock_t = 0.0
        self._vz_int = 0.0
        self._vz_des_lost_hold = 0.0
        self._governor_active = 0.0
        self._chase_receding = 0.0
        self._lat_slow_active = 0.0
        self._alt_protect_active = 0.0
        self._ground_floor_active = 0.0
        self._ground_floor_pass = 0.0
        self._overfly_active = 0.0
        self._overfly_prev = 0.0
        self._lost_climb_active = 0.0
        self._term_lat_active = 0.0
        self._terminal_active = 0.0
        self._terminal_climb_exempt = 0.0
        self._los_az_filt_dps = 0.0
        self._elev_prev = None
        self._v_adapt = 0.0
        self._d_elev_filt = 0.0
        self._yaw_initialized = False
        self._pitch_initialized = False
        self.dt = self.default_dt_s
        self.elapsed_t = 0.0
        self._last_frame_id = None
        self._wall_t_prev = None
        self._last_output = None
