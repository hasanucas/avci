#!/usr/bin/env python3
"""
chase_orin.py — the chase brain running ON the chaser's Orin (Asama 1).

    HEDEF FC --SiK--> laptop (relay_target.py) --SiK--> TAKIPCI FC --USB-->
        MAVProxy --UDP--> THIS SCRIPT --UDP--> MAVProxy --USB--> TAKIPCI FC

Same brain as the laptop's follow_gps.py (chase_core.TrackController: the
vendored trajectory planner, unrestricted INTERCEPT/FOLLOW, adaptive speed,
altitude offset), but the chaser is commanded over its own cable: no radio
latency or loss on the command path. The target's position arrives through
the chaser FC (forwarded by the laptop relay), told apart by system id.

Phase machine — a port of the sim session (trajectory_approach_session.py):

    TRACK_PLAN --(bbox lock)--> VISUAL --(bbox lost)--> REALIGN --> TRACK_PLAN

Asama 1 wires TRACK_PLAN only. VISUAL (Asama 3) and REALIGN (Asama 4) have
their places in the loop but are gated off (VISUAL_ENABLED / REALIGN_ENABLED);
the vision shadow observer (Asama 2) plugs into the per-tick hook.

Run on the Orin (MAVProxy must already be running, see README):
  python3 chase_orin.py --dry-run                       # read-only check
  python3 chase_orin.py --sep-bias <m> --set-mode       # chase

Abort: RC mode switch (the script sees the chaser leave GUIDED, stops sending
and never touches the mode again). Ctrl-C over SSH -> chaser LOITER. If SSH or
the SIYI link drops, the RC switch is the only abort — keep a hand on it.
"""
from __future__ import annotations

import argparse
import sys
import time
import traceback
from enum import Enum

from chase_core import (
    StateLog,
    TrackController,
    add_planner_args,
    build_traj_config,
    has_fix,
    haversine_m,
    print_plan_banner,
    warn_offset_vs_minsep,
)
from chaser_env import ChaserEnv
from link_log import LinkLogger
from mav_hub import MavHub


class Phase(Enum):
    WAIT = "wait"
    TRACK_PLAN = "track_plan"
    VISUAL = "visual"      # Asama 3: bbox lock -> HCAir handoff (not wired)
    REALIGN = "realign"    # Asama 4: bbox lost -> pipeline realign (not wired)


# Stage gates. Each flips only together with its own offline tests.
VISUAL_ENABLED = False
REALIGN_ENABLED = False


class NullObserver:
    """Asama 2 hook (vision shadow mode). Called every tick with the full
    state; it may log and measure but must never command the vehicle."""

    def on_tick(self, *, t_mono, t_wall, phase, tick, ts, cs) -> None:
        pass

    def lock_ready(self) -> bool:
        return False

    def last_angles(self):
        return (None, None)

    def close(self) -> None:
        pass


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # --- link to the chaser FC (via MAVProxy) ---
    ap.add_argument("--fc", default="udpout:127.0.0.1:14551",
                    help="takipci FC baglantisi. MAVProxy '--out=udpin:...:14551' ise "
                         "varsayilan dogru. MAVProxy yoksa: /dev/ttyACM0 (+ --fc-baud).")
    ap.add_argument("--fc-baud", type=int, default=115200, help="seri baglanti icin baud")
    ap.add_argument("--chaser-sysid", type=int, default=1, help="takipci FC sysid")
    ap.add_argument("--target-sysid", type=int, default=2,
                    help="hedef FC sysid (hedefte SYSID_THISMAV bu olmali)")
    ap.add_argument("--src-sysid", type=int, default=251,
                    help="bu scriptin kendi sysid'i (laptop relay 250 kullanir)")
    ap.add_argument("--pos-hz", type=float, default=10.0,
                    help="takipci konum hizi, USB uzerinden (Hz)")
    ap.add_argument("--att-hz", type=float, default=10.0,
                    help="takipci ATTITUDE hizi, USB uzerinden (Hz). Yaw = hover'da burun yonu.")
    ap.add_argument("--rate", type=float, default=10.0,
                    help="en dusuk dongu hizi (Hz). Dongu ayrica her yeni hedef fix'inde uyanir.")
    ap.add_argument("--wait-s", type=float, default=20.0,
                    help="baslangicta iki araci bekleme suresi (s)")
    ap.add_argument("--print-hz", type=float, default=2.0,
                    help="durum satiri hizi (Hz); SSH'i bogmamak icin dusuk")

    # --- planner / altitude / safety / posvel: identical to follow_gps ---
    add_planner_args(ap)

    # --- logging ---
    ap.add_argument("--no-log", action="store_true", help="loglamayi kapat (varsayilan ACIK)")
    ap.add_argument("--log-dir", default="logs", help="log klasoru")
    ap.add_argument("--rtt-ping-s", type=float, default=1.0,
                    help="komut->tepki RTT ping araligi (s), USB uzerinden. 0 = kapali.")

    # --- run control ---
    ap.add_argument("--set-mode", action="store_true", help="modu script degistirsin (GUIDED)")
    ap.add_argument("--dry-run", action="store_true", help="FC'ye komut gonderme")
    return ap


class Orchestrator:
    def __init__(self, a, hub, *, observer=None, clock=time):
        self.a = a
        self.hub = hub
        self.observer = observer or NullObserver()
        self.clock = clock
        self.phase = Phase.WAIT
        self.link_log = None
        self.state_log = None
        self.ctl = None
        self.env = None
        self.commanding = False   # True once the chaser is GUIDED under our control

    # ------------------------------------------------ helpers
    def _on_msg(self, name, msg, t_mono):
        self.link_log.note_message(name, msg, t_mono)
        if name == "TAKIPCI" and msg.get_type() == "COMMAND_ACK":
            tgt = getattr(msg, "target_system", 0) or 0
            if tgt in (0, self.a.src_sysid):  # ours, not MAVProxy's or the laptop's
                self.link_log.cmd_lat.note_ack(msg, t_mono)

    def _print_vehicle(self, tag, s):
        if s.get("lat") is None:
            print(f"  {tag}: konum yok")
            return
        print(f"  {tag}: {s['lat']:.7f}, {s['lon']:.7f}  amsl={s['alt_amsl']:.1f} "
              f"rel={s['alt_rel']:.1f}  fix={s['fix']}/{s['nsat']}  mod={s['mode']}  "
              f"gelen={s['hz']:.1f} Hz")

    # ------------------------------------------------ main flow
    def run(self) -> int:
        code, pilot_exit = 1, False
        try:
            code, pilot_exit = self._run()
        except KeyboardInterrupt:
            print("\n[i] Durduruldu.")
            code = 0
        except Exception:
            traceback.print_exc()
            print("[X] Beklenmeyen hata - durdu.")
            code = 1
        finally:
            self._finish(pilot_exit=pilot_exit)
        return code

    def _run(self):
        a, hub, C = self.a, self.hub, self.clock
        warn_offset_vs_minsep(a)
        self.link_log = LinkLogger(
            a.log_dir,
            meta={"script": "chase_orin", "fc": a.fc, "max_speed_mps": a.max_speed,
                  "deadline_range_m": a.deadline_range, "alt_offset_m": a.alt_offset,
                  "alt_match": int(a.alt_match), "posvel": int(a.posvel),
                  "rate_hz": a.rate, "pos_hz": a.pos_hz, "att_hz": a.att_hz},
            enabled=not a.no_log)
        hub.on_msg = self._on_msg
        hub.start()
        hub.request_streams(pos_hz=a.pos_hz, att_hz=a.att_hz)

        print(f"\n[i] {a.fc} uzerinden takipci (sysid {a.chaser_sysid}) ve "
              f"hedef (sysid {a.target_sysid}) bekleniyor...")
        c_ok, t_ok = hub.wait_ready(a.wait_s)
        others = dict(hub.others)
        if others:
            print(f"  [i] baglantida baska sistemler de var (sysid:paket): {others} "
                  f"- 255 MAVProxy, 250 laptop relay olagan")
        if not c_ok:
            print("[X] Takipci heartbeat'i yok. MAVProxy calisiyor mu, --fc portu dogru mu?")
            return 2, False
        if not t_ok:
            print("[X] Hedef konumu gelmedi. Kontrol et: (1) laptop'ta relay_target.py calisiyor "
                  "mu, (2) hedefte SYSID_THISMAV=2 mi, (3) takipci radyo linki saglam mi, "
                  "(4) takipci FC'de MAVLink yonlendirmesi kapali olmasin (SERIALx_OPTIONS).")
            return 2, False

        ts, cs = hub.target.snapshot(), hub.chaser.snapshot()
        self._print_vehicle("HEDEF  ", ts)
        self._print_vehicle("TAKIPCI", cs)
        if has_fix(ts) and has_fix(cs):
            print(f"  aralarindaki mesafe: "
                  f"{haversine_m(cs['lat'], cs['lon'], ts['lat'], ts['lon']):.0f} m")
            print(f"  olculen dikey ayrim: {ts['alt_amsl'] - cs['alt_amsl']:+.1f} m "
                  f"(+ = takipci altta)")
        if not has_fix(cs):
            print("[X] Takipci 3D fix olmadan yorunge cercevesi kurulamaz.")
            return 2, False

        cfg = build_traj_config(a, cs["lat"], cs["lon"])
        self.env = ChaserEnv(hub.chaser_handle(), verbose_speed=False,
                             cmd_latency=self.link_log.cmd_lat)
        self.env.enable_posvel(a.posvel)
        self.ctl = TrackController(
            cfg, self.env, alt_offset=a.alt_offset, sep_bias=a.sep_bias,
            min_alt=a.min_alt, min_sep=a.min_sep, max_dist=a.max_dist,
            max_age=a.max_age, posvel=a.posvel, dry_run=a.dry_run,
            cmd_lat=self.link_log.cmd_lat,
            att_max_age_s=max(1.0, 3.0 / a.att_hz) if a.att_hz > 0 else 1.0)
        self.state_log = StateLog(self.link_log.log_dir / f"chase_state_{self.link_log.stamp}.csv",
                                  meta={"script": "chase_orin"}, enabled=not a.no_log)

        print_plan_banner(a)
        print(f"  beyin  : Orin (komutlar USB'den, radyodan degil). Faz: TRACK_PLAN. "
              f"VISUAL/REALIGN: kapali (Asama 3/4).")
        if a.dry_run:
            print("\n--dry-run: FC'ye komut gonderilmiyor.\n")
        else:
            print("\nOnay icin 'evet' yaz:")
            if input("> ").strip().lower() != "evet":
                print("[i] Iptal.")
                return 0, False
            if a.set_mode:
                if not hub.set_chaser_mode("GUIDED"):
                    print("[X] Takipci GUIDED moduna gecmedi.")
                    return 2, False
            else:
                print("[i] Takipciyi kumandadan GUIDED moduna al. Bekleniyor...")
                while hub.chaser.snapshot()["mode"] != "GUIDED":
                    C.sleep(0.3)
                print("[i] Takipci GUIDED modunda.")
            self.commanding = True
            self.env.configure_guided_speed_limits(speed_mps=float(a.max_speed),
                                                   accel_mps2=float(a.accel))
            self.env.verify_wp_spd(float(a.max_speed))

        print("Ctrl-C -> takipci LOITER. Kumanda her an ezer.\n")
        # --- the sim session's outer loop ---
        need_realign = False
        last_ang = (None, None)
        while True:
            if need_realign:
                self.ctl.begin_realign(*last_ang)      # Asama 4
            reason = self._track_plan_until_lock()
            if reason == "pilot":
                return 0, True
            if reason != "lock" or not VISUAL_ENABLED:
                return 0, False
            vis_reason = self._visual_phase()          # Asama 3
            if vis_reason != "bbox_lost" or not REALIGN_ENABLED:
                return 0, False
            need_realign = True
            last_ang = self.observer.last_angles()

    def _track_plan_until_lock(self) -> str:
        """TRACK_PLAN until the pilot takes over (or, from Asama 3, a lock)."""
        a, hub, C, ctl = self.a, self.hub, self.clock, self.ctl
        self.phase = Phase.TRACK_PLAN
        period = 1.0 / max(1e-3, a.rate)
        print_dt = 1.0 / max(1e-3, a.print_hz)
        last_print = -1e9
        last_stream_req = last_ping = C.monotonic()
        last_beh = None
        while True:
            # wake on each new target fix, at the latest after one period
            hub.target.new_pos.wait(timeout=period)
            hub.target.new_pos.clear()
            now, wall = C.monotonic(), C.time()
            ts, cs = hub.target.snapshot(), hub.chaser.snapshot()

            if not a.dry_run and cs["mode"] != "GUIDED":
                print(f"\n[!] Takipci modu {cs['mode']} oldu. Kontrol kumandada, cikiliyor.")
                return "pilot"
            if now - last_stream_req > 5.0:
                try:
                    hub.request_streams(pos_hz=a.pos_hz, att_hz=a.att_hz)
                except Exception:
                    pass
                last_stream_req = now
            if not a.dry_run and a.rtt_ping_s > 0 and now - last_ping >= a.rtt_ping_s:
                try:
                    self.env.ping_rtt(now)
                except Exception:
                    pass
                last_ping = now

            k = ctl.tick(ts, cs, now)
            self.observer.on_tick(t_mono=now, t_wall=wall, phase=self.phase, tick=k, ts=ts, cs=cs)
            self.state_log.write(t_wall=wall, t_mono=now, phase=self.phase.value, k=k, ts=ts, cs=cs)
            self.link_log.note_relay(
                t_mono=now, updated=k.updated,
                target_age_ms=ts["age"] * 1e3, chaser_age_ms=cs["age"] * 1e3,
                compute_ms=k.compute_ms, send_ms=k.send_ms,
                behavior=("hold" if k.held else "wait") if k.why else k.behavior,
                follow_phase=k.why[:16] if k.why else k.follow_phase,
                v_cmd_mps=k.v_cmd, dist_m=k.d, sep_m=k.sep, gate_alt_m=k.cmd_alt,
                target_hz=ts["hz"], chaser_hz=cs["hz"])

            changed = (k.behavior != last_beh)
            last_beh = k.behavior
            if changed or now - last_print >= print_dt:
                last_print = now
                if k.why:
                    print(f"  [BEKLE] {k.why}" + ("  -> takipci yerinde tutuluyor" if k.held else ""))
                else:
                    v = k.v_cmd if k.v_cmd is not None else 0.0
                    print(f"  d={k.d:6.1f} m  ayrim={k.sep:+5.1f} m  beh={k.behavior:9s} "
                          f"fp={k.follow_phase:5s}  v_cmd={v:4.1f}  komut_alt={k.cmd_alt:5.1f} m  "
                          f"hedef_hiz={k.target_speed:4.1f}  hedef_yas={ts['age']:.2f}s "
                          f"({ts['hz']:.1f}Hz)  yon={k.heading_src}  n={ctl.sent}")

            # Asama 3 handoff point (gated): stable lock AND the planner allows it.
            if VISUAL_ENABLED and self.observer.lock_ready() and k.allow_visual_lock:
                return "lock"

    def _visual_phase(self) -> str:  # Asama 3
        raise NotImplementedError("VISUAL fazi Asama 3'te baglanacak")

    def _finish(self, *, pilot_exit: bool) -> None:
        """Runs exactly once. The mode is touched ONLY if we were commanding
        and the pilot did not take over: then GUIDED -> LOITER. A pilot's
        mode choice is never overridden (their RC abort is the safety)."""
        hub = self.hub
        if self.ctl is not None:
            print(f"[i] gonderim={self.ctl.sent} yerinde-tut={self.ctl.holds}")
        try:
            if self.commanding:
                mode = hub.chaser.snapshot()["mode"]
                if pilot_exit or mode != "GUIDED":
                    print(f"[i] Kontrol pilotta (mod={mode}); mod degistirilmiyor.")
                else:
                    print("[i] Takipci LOITER'a aliniyor...")
                    hub.set_chaser_mode("LOITER")
        finally:
            self.observer.close()
            if self.state_log is not None:
                self.state_log.close()
            if self.link_log is not None:
                self.link_log.close()
            hub.stop()


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    try:
        hub = MavHub.connect(a.fc, baud=a.fc_baud, source_system=a.src_sysid,
                             chaser_sysid=a.chaser_sysid, target_sysid=a.target_sysid)
    except ValueError as e:
        print(f"[X] {e}")
        return 2
    return Orchestrator(a, hub).run()


if __name__ == "__main__":
    sys.exit(main())
