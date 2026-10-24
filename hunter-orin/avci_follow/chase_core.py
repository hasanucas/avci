#!/usr/bin/env python3
"""
chase_core — the one TRACK_PLAN brain shared by every front-end.

Both front-ends drive the chaser through this module, so the laptop and the
Orin fly byte-identical logic and one offline test covers both:

  follow_gps.py  (laptop)  two telemetry radios -> TrackController -> radio
  chase_orin.py  (Orin)    one FC link (MAVProxy) -> TrackController -> USB

A tick mirrors the sim's `trajectory_approach_until_lock` loop body
(scripts/trajectory_approach_session.py):

  * gate failure (target stale / no fix / too far / too close vertically):
    hold the chaser where it is — the sim's --stale-hold behaviour — instead
    of letting it keep flying to the last carrot (up to 250 m ahead).
  * new target fix (exact arrival stamp, never a duplicate) -> pipeline.update
  * EVERY tick -> apply_planner_output(last_out) so the carrot keeps sliding.

Planner inputs match the sim: drone heading = ATTITUDE yaw (the behaviour
selector uses it as the "nose" while hovering); course over ground is only a
fallback when no fresh attitude is available.

The vendored planner (fpv_gate_il.trajectory) is untouched; everything here is
session glue.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from fpv_gate_il.gate_gps import GateGpsSample
from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.coords import latlonalt_to_local
from fpv_gate_il.trajectory.mav_bridge import (
    CarrotDirSlew,
    CatchupSpeedLatch,
    PositionSetpointHold,
    SpeedCommandSlew,
    apply_planner_output,
    traj_hold_alt_m,
)
from fpv_gate_il.trajectory.pipeline import TrajectoryPipeline
from fpv_gate_il.trajectory.radar import RadarMeasurement
from fpv_gate_il.trajectory.types import DroneState, NedVelocity

EARTH_R = 6371000.0


# ---------------------------------------------------------------- geometry
def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def has_fix(s) -> bool:
    """3D fix and a real coordinate (reject 0,0 'null island')."""
    if s.get("lat") is None or s.get("fix", 0) < 3:
        return False
    return not (abs(s["lat"]) < 1e-6 and abs(s["lon"]) < 1e-6)


def chaser_home_amsl(s) -> Optional[float]:
    """Chaser home AMSL = alt_amsl - alt_rel. Constant through a flight even
    as the baro drifts (both fields drift together), so it is the stable
    reference for cross-vehicle altitude math."""
    if s.get("alt_amsl") is None or s.get("alt_rel") is None:
        return None
    return float(s["alt_amsl"]) - float(s["alt_rel"])


# ---------------------------------------------------------------- shared CLI
def add_planner_args(ap) -> None:
    """Planner / altitude / safety flags. Shared so the laptop and the Orin
    front-ends always have identical flags and defaults."""
    # --- trajectory planner (peak speed fix + adaptive schedule) ---
    ap.add_argument("--max-speed", type=float, default=20.0,
                    help="uzaktaki tepe hiz (m/s). WP_SPD tavani + DO_CHANGE_SPEED "
                         "buna gore kurulur (eski 'hep 10' hatasi burada cozulur).")
    ap.add_argument("--deadline-range", type=float, default=100.0,
                    help="bu mesafede hiz close-approach'a iner (m). Varsayilan 100.")
    ap.add_argument("--close-approach", type=float, default=10.0,
                    help="deadline icindeki yaklasma hizi (m/s)")
    ap.add_argument("--decel-start-range", type=float, default=200.0,
                    help="bu mesafeden itibaren tepe hizdan yavaslamaya basla (m). "
                         "Varsayilan 200: ~200 m'ye kadar tam hiz, 200->100 arasi iner.")
    ap.add_argument("--accel", type=float, default=4.0,
                    help="WP_ACC / planlayici ivme siniri (m/s^2)")
    ap.add_argument("--carrot", type=float, default=250.0,
                    help="carrot on-bakis mesafesi (m)")
    ap.add_argument("--time-margin", type=float, default=2.5,
                    help="intercept zaman marji (s)")

    # --- altitude ---
    ap.add_argument("--alt-offset", type=float, default=-3.0,
                    help="hedefin irtifasina gore ofset (m). POZITIF = ustunde, "
                         "NEGATIF = altinda. Her tik yeniden hesaplanir (latch yok). "
                         "Varsayilan -3: unutulsa bile 3 m altta bir dikey marj.")
    ap.add_argument("--alt-match", dest="alt_match", action="store_true", default=True,
                    help="temiz irtifa modu (VARSAYILAN ACIK): dogrudan hedef_irtifa "
                         "+ offset'e ucus, dosyanin balon temizlik/tavan mantigi devrede degil.")
    ap.add_argument("--no-alt-match", dest="alt_match", action="store_false",
                    help="temiz modu KAPAT; dosyanin orijinal balon temizlik/tavan "
                         "mantigina don (alcak hedefin ustunden gec, 50 m'de sinirla).")
    ap.add_argument("--sep-bias", type=float, default=0.0,
                    help="iki drone GERCEKTEN ayni seviyedeyken okunan dikey ayrim. "
                         "Once --dry-run ile olc, buraya gir.")
    ap.add_argument("--min-alt", type=float, default=10.0,
                    help="komut irtifasi alt siniri (m, chaser home ustu)")

    # --- safety ---
    ap.add_argument("--min-sep", type=float, default=0.0,
                    help="OLCULEN dikey ayrim |sep| bunun altina inerse takipci yerinde "
                         "tutulur (m). 0 = KAPALI. Intercept dogal olarak yaklastigi icin "
                         "varsayilan kapali; ayni irtifada gecis icin 0 birak.")
    ap.add_argument("--max-dist", type=float, default=1000.0,
                    help="guvenlik kelepcesi (m): otesinde takipci yerinde tutulur")
    ap.add_argument("--max-age", type=float, default=3.0,
                    help="hedef konumu bayatlama siniri (s): otesinde takipci yerinde tutulur")

    # --- posvel (kept as flag) ---
    ap.add_argument("--posvel", action="store_true",
                    help="komuta hedefin hizini da kat (ileri besleme). Carrot "
                         "planlayicida nadiren gerekir; sabit hedefte gereksiz.")


def build_traj_config(a, home_lat: float, home_lon: float) -> TrajectoryConfig:
    """TrajectoryConfig from CLI args. NED origin = chaser startup lat/lon."""
    cfg = TrajectoryConfig(
        home_lat_deg=float(home_lat),
        home_lon_deg=float(home_lon),
        home_alt_m=0.0,  # coords ignore this; command alt comes from traj_hold_alt_m(gate)
        max_speed_mps=float(a.max_speed),
        deadline_range_m=float(a.deadline_range),
        close_approach_speed_mps=float(a.close_approach),
        speed_decel_start_range_m=float(a.decel_start_range),
        max_acceleration_mps2=float(a.accel),
        path_carrot_lookahead_m=float(a.carrot),
        time_margin_s=float(a.time_margin),
    )
    if a.alt_match:
        cfg.traj_alt_match_above_m = -1.0e9  # traj_hold_alt_m returns gate alt unchanged
    return cfg


def warn_offset_vs_minsep(a) -> None:
    if a.min_sep and a.min_sep > 0 and abs(a.alt_offset) < a.min_sep:
        print(f"[!] UYARI: |--alt-offset|={abs(a.alt_offset):.0f} m < --min-sep={a.min_sep:.0f} m. "
              f"Dikey ayrim sigortasi amaclanan yakin geciste tetiklenir. "
              f"Ayni irtifada intercept istiyorsan --min-sep 0 ver, ya da "
              f"|--alt-offset|'i buyut.", flush=True)


def print_plan_banner(a) -> None:
    where = "ustunde" if a.alt_offset > 0 else ("altinda" if a.alt_offset < 0 else "ayni irtifada")
    print(f"\n  plan   : takipci hedefi kovalar; karsidan gelirsen ONUNU KESER (intercept), "
          f"caprazda ARKADAN takip eder (follow) - kisitsiz.")
    print(f"  hiz    : uzakta {a.max_speed:.0f} m/s, {a.deadline_range:.0f} m icinde "
          f"{a.close_approach:.0f} m/s'e iner (adaptif). WP_ACC={a.accel:.0f}.")
    print(f"  irtifa : hedefin {abs(a.alt_offset):.0f} m {where}"
          + (" (alt-match: dogrudan)" if a.alt_match else " (dosya temizlik/tavan mantigi)")
          + f"; alt sinir {a.min_alt:.0f} m")
    print(f"  komut  : {'pozisyon + hedef hizi (posvel)' if a.posvel else 'pozisyon + yaw'}")
    print(f"  bekleme: hedef bayat/fix yok/kelepce -> takipci YERINDE TUTULUR (son carrot'a gitmez)")
    if a.min_sep and a.min_sep > 0:
        print(f"  sigorta: olculen |dikey ayrim| < {a.min_sep:.0f} m ise takipci yerinde tutulur")
    else:
        print(f"  sigorta: --min-sep 0 (KAPALI). Carpisma sorumlulugu pilotta - "
              f"RC'de moda hazir dur, dikey ofset ver.")
    if a.sep_bias:
        print(f"  duzeltme: ayrim okumasindan {a.sep_bias:+.1f} m cikariliyor")


# ---------------------------------------------------------------- the tick
@dataclass
class Tick:
    """Everything one control tick decided — for printing and logging."""
    why: Optional[str] = None          # gate failed -> reason (no planner command)
    held: bool = False                 # a hold-in-place setpoint was sent
    d: Optional[float] = None          # horizontal distance to target (m)
    sep: Optional[float] = None        # measured vertical separation - sep_bias (m)
    updated: bool = False              # pipeline.update ran on a new target fix
    compute_ms: Optional[float] = None
    send_ms: Optional[float] = None
    behavior: str = "-"
    follow_phase: str = "-"
    v_cmd: Optional[float] = None      # last commanded groundspeed (m/s)
    cmd_alt: Optional[float] = None    # commanded altitude (m, chaser home)
    target_speed: Optional[float] = None
    range_m: Optional[float] = None    # planner's range to target
    allow_visual_lock: Optional[bool] = None
    realign_phase: str = "-"
    heading_src: str = "-"             # "yaw" (sim-faithful) or "course" (fallback)


class _NullCmdLat:
    def arm_speed(self, *a, **k):
        pass

    def note_gs(self, *a, **k):
        pass


class TrackController:
    """TRACK_PLAN phase: the trajectory pipeline driving the chaser.

    Snapshot contract (both front-ends provide it): lat, lon, alt_amsl, alt_rel,
    vx, vy, vz (NED m/s), fix, nsat, mode, age (s since newest position),
    hz, pos_mono (monotonic arrival time of newest position; exact per fix),
    yaw (rad) + att_age (s) when attitude is streamed.
    """

    def __init__(self, cfg: TrajectoryConfig, env, *, alt_offset: float, sep_bias: float,
                 min_alt: float, min_sep: float, max_dist: float, max_age: float,
                 posvel: bool, dry_run: bool, cmd_lat=None, att_max_age_s: float = 1.0):
        self.cfg = cfg
        self.env = env
        self.alt_offset = float(alt_offset)
        self.sep_bias = float(sep_bias)
        self.min_alt = float(min_alt)
        self.min_sep = float(min_sep or 0.0)
        self.max_dist = float(max_dist)
        self.max_age = float(max_age)
        self.posvel = bool(posvel)
        self.dry_run = bool(dry_run)
        self.cmd_lat = cmd_lat if cmd_lat is not None else _NullCmdLat()
        self.att_max_age_s = float(att_max_age_s)

        self.pipeline = TrajectoryPipeline(cfg)
        self.pipeline.soft_reset_guidance(realign=False)
        self.speed_slew = SpeedCommandSlew.from_config(cfg)
        self.pos_hold = PositionSetpointHold.from_config(cfg)
        self.dir_slew = CarrotDirSlew.from_config(cfg)
        self.catchup = CatchupSpeedLatch()

        self.last_out = None
        self.last_speed: Optional[float] = None
        self._last_pos_key = None
        self._speed_seeded = False
        self._last_armed_speed: Optional[float] = None
        self.sent = 0
        self.holds = 0
        self.course_fallbacks = 0

    # ------------------------------------------------ stage-4 hook (not wired yet)
    def begin_realign(self, last_ang_x=None, last_ang_y=None) -> None:
        """VISUAL -> TRAJ after bbox loss, exactly as the sim session does it."""
        self.pipeline.soft_reset_guidance(realign=True, last_ang_x=last_ang_x,
                                          last_ang_y=last_ang_y)

    # ------------------------------------------------ helpers
    def gate_alt(self, ts, chome: float) -> float:
        """Target altitude in the chaser-home frame, plus the operator offset."""
        return (float(ts["alt_amsl"]) - self.sep_bias - chome) + self.alt_offset

    def drone_state(self, cs, now: float):
        c_ned = latlonalt_to_local(cs["lat"], cs["lon"], cs["alt_rel"],
                                   home_lat_deg=self.cfg.home_lat_deg,
                                   home_lon_deg=self.cfg.home_lon_deg,
                                   home_alt_m=self.cfg.home_alt_m)
        yaw, att_age = cs.get("yaw"), cs.get("att_age")
        if yaw is not None and att_age is not None and att_age <= self.att_max_age_s:
            heading, src = float(yaw), "yaw"
        else:
            heading, src = math.atan2(cs["vy"], cs["vx"]), "course"
            self.course_fallbacks += 1
        drone = DroneState(
            position=c_ned,
            velocity=NedVelocity(float(cs["vx"]), float(cs["vy"]), float(cs["vz"])),
            timestamp_s=now,
            speed_mps=math.hypot(cs["vx"], cs["vy"]),
            heading_rad=heading,
        )
        return drone, src

    def _hold(self, cs) -> bool:
        """Sim --stale-hold: position target at the chaser's current position."""
        if self.dry_run or not has_fix(cs) or cs.get("alt_rel") is None:
            return False
        self.env.refresh_from_snapshot(cs)
        if self.posvel:
            self.env.set_feedforward_velocity(0.0, 0.0, 0.0)  # hold = zero velocity
        self.env.send_position_target_global(
            lat_deg=float(cs["lat"]), lon_deg=float(cs["lon"]),
            alt_rel_m=float(cs["alt_rel"]))
        self.holds += 1
        return True

    # ------------------------------------------------ one control tick
    def tick(self, ts, cs, now: float) -> Tick:
        k = Tick()

        # --- gates: chaser first (a hold needs its own position) ---
        if cs.get("lat") is None:
            k.why = "takipci konumu henuz gelmedi"
            return k
        if not has_fix(cs):
            k.why = f"TAKIPCI GPS yok (fix={cs.get('fix')}, sat={cs.get('nsat')})"
            return k  # cannot hold without our own GPS; FC failsafes handle this
        chome = chaser_home_amsl(cs)
        if ts.get("lat") is None:
            k.why = "hedef konumu henuz gelmedi"
        elif not has_fix(ts):
            k.why = f"HEDEF GPS yok (fix={ts.get('fix')}, sat={ts.get('nsat')})"
        elif ts["age"] > self.max_age:
            k.why = f"hedef konumu {ts['age']:.1f} s bayat"
        elif chome is None:
            k.why = "takipci home AMSL cozulemedi"
        else:
            k.d = haversine_m(cs["lat"], cs["lon"], ts["lat"], ts["lon"])
            k.sep = (float(ts["alt_amsl"]) - float(cs["alt_amsl"])) - self.sep_bias
            if k.d > self.max_dist:
                k.why = f"mesafe {k.d:.0f} m > kelepce {self.max_dist:.0f} m"
            elif self.min_sep > 0 and abs(k.sep) < self.min_sep:
                k.why = (f"!! DIKEY AYRIM {k.sep:+.1f} m ({abs(k.sep):.1f} < {self.min_sep:.0f})"
                         f" - YERINDE TUT, KUMANDAYA HAZIR OL")
        if k.why:
            k.held = self._hold(cs)
            return k

        # --- own state in the fixed chaser-home NED frame ---
        drone, k.heading_src = self.drone_state(cs, now)

        # --- feed the KF only on a genuinely new target fix (exact stamp) ---
        pos_key = ts.get("pos_mono")
        if pos_key is None:  # front-end without exact stamps: coarse fallback
            pos_key = round(now - float(ts["age"]), 3)
        if self._last_pos_key is None or pos_key != self._last_pos_key:
            self._last_pos_key = pos_key
            spd = math.hypot(ts["vx"], ts["vy"])
            meas = RadarMeasurement(
                timestamp_s=now,  # sim: gate_sample_to_radar(gate, timestamp_s=now)
                latitude_deg=float(ts["lat"]),
                longitude_deg=float(ts["lon"]),
                altitude_up_m=float(self.gate_alt(ts, chome)),
                ground_speed_mps=spd if spd > 0.05 else None,
                course_rad=(math.atan2(ts["vy"], ts["vx"]) if spd > 0.05 else None),
            )
            t0 = time.perf_counter()
            self.last_out = self.pipeline.update(meas, drone)
            k.compute_ms = (time.perf_counter() - t0) * 1e3
            k.updated = True

        # --- gate sample every tick; min-alt floor ---
        out = self.last_out
        gate_alt = self.gate_alt(ts, chome)
        rng = out.distance_to_target_m if out is not None else None
        preview = traj_hold_alt_m(gate_alt, self.cfg, range_m=rng)
        if preview < self.min_alt:
            gate_alt += (self.min_alt - preview)  # bump so commanded alt >= floor
        gate = GateGpsSample(lat=float(ts["lat"]), lon=float(ts["lon"]),
                             alt=float(gate_alt), recv_mono=float(ts.get("pos_mono") or now))

        # --- drive the chaser ---
        if out is not None and not self.dry_run:
            self.env.refresh_from_snapshot(cs)
            if self.posvel:
                # TARGET velocity (follow_relay parity). Bug fix: was the chaser's own.
                self.env.set_feedforward_velocity(ts["vx"], ts["vy"], ts["vz"])
            if not self._speed_seeded:
                self.speed_slew.reset(float(drone.speed_mps))
                self._speed_seeded = True
            t0 = time.perf_counter()
            self.last_speed = apply_planner_output(
                self.env, out, gate=gate, drone=drone, config=self.cfg,
                last_speed_cmd=self.last_speed, speed_slew=self.speed_slew,
                pos_hold=self.pos_hold, dir_slew=self.dir_slew,
                catchup=self.catchup, now_s=now,
            )
            k.send_ms = (time.perf_counter() - t0) * 1e3
            self.sent += 1

        # --- command->response speed-reaction sampling (includes FC + accel) ---
        chaser_gs = math.hypot(cs["vx"], cs["vy"])
        if self.last_speed is not None:
            if (self._last_armed_speed is None
                    or abs(self.last_speed - self._last_armed_speed) >= 1.5):
                self.cmd_lat.arm_speed(chaser_gs, self.last_speed, now)
                self._last_armed_speed = self.last_speed
        self.cmd_lat.note_gs(chaser_gs, now)

        # --- report ---
        if out is not None:
            k.behavior = out.behavior.value if out.behavior else "-"
            k.follow_phase = out.follow_phase.value if out.follow_phase else "-"
            k.range_m = out.distance_to_target_m
            k.allow_visual_lock = bool(out.allow_visual_lock)
            k.realign_phase = (out.realign_phase.value
                               if (out.realign_active and out.realign_phase) else "-")
        k.v_cmd = self.last_speed
        k.cmd_alt = traj_hold_alt_m(gate_alt, self.cfg, range_m=rng)
        k.target_speed = math.hypot(ts["vx"], ts["vy"])
        return k


# ---------------------------------------------------------------- state log
_STATE_HEADER = (
    "t_wall,t_mono,phase,why,held,updated,behavior,follow_phase,allow_lock,realign,"
    "v_cmd,cmd_alt,range_m,d_m,sep_m,heading_src,"
    "c_lat,c_lon,c_alt_rel,c_alt_amsl,c_vx,c_vy,c_vz,c_roll,c_pitch,c_yaw,c_att_age,c_hz,"
    "t_lat,t_lon,t_alt_amsl,t_vx,t_vy,t_vz,t_age,t_hz,compute_ms,send_ms"
)


class StateLog:
    """One row per tick with both vehicles' full state (position, velocity,
    chaser attitude) plus the planner decision. Wall clock is the machine's
    own, so on the Orin it lines up with the onboard video recording: every
    GPS-only flight doubles as ground truth for the vision shadow stage
    (where the target SHOULD appear in the camera)."""

    def __init__(self, path, meta: dict | None = None, enabled: bool = True):
        from link_log import _CsvFile  # local import: link_log stays optional
        self.enabled = bool(enabled)
        self.csv = _CsvFile(path, dict(meta or {}), _STATE_HEADER) if self.enabled else None
        self._last_flush = time.monotonic()

    def write(self, *, t_wall, t_mono, phase, k: Tick, ts, cs) -> None:
        if not self.enabled:
            return

        def f(x, n=3):
            return "" if x is None else round(float(x), n)

        def ll(x):
            return "" if x is None else f"{float(x):.7f}"

        self.csv.write_row([
            f"{t_wall:.3f}", f"{t_mono:.3f}", phase, (k.why or "").replace(",", ";"),
            int(k.held), int(k.updated), k.behavior, k.follow_phase,
            "" if k.allow_visual_lock is None else int(k.allow_visual_lock), k.realign_phase,
            f(k.v_cmd, 2), f(k.cmd_alt, 2), f(k.range_m, 1), f(k.d, 1), f(k.sep, 2),
            k.heading_src,
            ll(cs.get("lat")), ll(cs.get("lon")), f(cs.get("alt_rel"), 2), f(cs.get("alt_amsl"), 2),
            f(cs.get("vx"), 2), f(cs.get("vy"), 2), f(cs.get("vz"), 2),
            f(cs.get("roll"), 4), f(cs.get("pitch"), 4), f(cs.get("yaw"), 4),
            f(cs.get("att_age"), 3), f(cs.get("hz"), 2),
            ll(ts.get("lat")), ll(ts.get("lon")), f(ts.get("alt_amsl"), 2),
            f(ts.get("vx"), 2), f(ts.get("vy"), 2), f(ts.get("vz"), 2),
            f(ts.get("age"), 3), f(ts.get("hz"), 2),
            f(k.compute_ms, 3), f(k.send_ms, 3),
        ])
        if t_mono - self._last_flush > 2.0:
            self._last_flush = t_mono
            self.csv.flush()

    def close(self) -> None:
        if self.enabled and self.csv is not None:
            self.csv.close()
