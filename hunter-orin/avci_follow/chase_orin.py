#!/usr/bin/env python3
"""
chase_orin.py — the chase brain running ON the chaser's Orin (Asama 1).

RADIO layout (field, from 24 Oct 2026) — the chaser's 3DR is on the Orin USB:

    target source --> Victus (relay_radio.py) --3DR--> Orin USB --> THIS SCRIPT
    THIS SCRIPT --UDP 14551--> MAVProxy --USB--> TAKIPCI FC   (commands + telemetry)

    python3 chase_orin.py --target-link /dev/serial/by-id/usb-...  --dry-run
    python3 chase_orin.py --target-link /dev/serial/by-id/usb-...  --sep-bias <m> --set-mode

LEGACY layout (radio soldered on the chaser FC, laptop runs relay_target.py):

    HEDEF FC --SiK--> laptop (relay_target.py) --SiK--> TAKIPCI FC --USB-->
        MAVProxy --UDP--> THIS SCRIPT --UDP--> MAVProxy --USB--> TAKIPCI FC

    python3 chase_orin.py --dry-run

Same brain as the laptop's follow_gps.py (chase_core.TrackController: the
vendored trajectory planner, unrestricted INTERCEPT/FOLLOW, adaptive speed,
altitude offset); the chaser is commanded over its own cable, so no radio
latency or loss on the command path.

Phase machine — a port of the sim session (trajectory_approach_session.py):

    TRACK_PLAN --(bbox lock)--> VISUAL --(bbox lost)--> REALIGN --> TRACK_PLAN

Asama 1 wires TRACK_PLAN only. VISUAL (Asama 3) and REALIGN (Asama 4) have
their places in the loop but are gated off (VISUAL_ENABLED / REALIGN_ENABLED);
the vision shadow observer (Asama 2) plugs into the per-tick hook.

RUN IT INSIDE tmux. The pilot's RC mode switch is both the safety and the
pause button: the moment the chaser leaves GUIDED (LOITER, ALT_HOLD, ...) the
script stops sending and never touches the mode; it keeps running (target
stream, logs, status line) and takes the chase up again, from a clean planner
state, as soon as the pilot puts the chaser back in GUIDED. No restart, no
landing. (--exit-on-pilot restores the old behaviour: exit instead of wait.)
Ctrl-C, SIGTERM or SIGHUP (SSH dropped outside tmux) -> chaser LOITER, if it
was ours in GUIDED at that moment.
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time
import traceback
from enum import Enum

from chase_core import (
    StateLog,
    Tick,
    TrackController,
    add_planner_args,
    build_traj_config,
    format_altitude,
    has_fix,
    haversine_m,
    print_plan_banner,
    print_initial_separation,
    validate_altitude_options,
    warn_offset_vs_minsep,
)
from chaser_env import ChaserEnv
from link_log import LinkLogger
from mav_hub import MavHub


class Phase(Enum):
    WAIT = "wait"
    TRACK_PLAN = "track_plan"
    PAUSED = "paused"      # pilot has the chaser (mode != GUIDED): no commands at all
    VISUAL = "visual"      # Asama 3: bbox lock -> HCAir handoff (not wired)
    REALIGN = "realign"    # Asama 4: bbox lost -> pipeline realign (not wired)


# Stage gates. Each flips only together with its own offline tests.
VISUAL_ENABLED = False
REALIGN_ENABLED = False

# Set by the signal handler so the exit event can say why we stopped.
_SIGNAL = {"name": None}


def _say(*a, **k) -> None:
    """print() that never raises (a dead SSH terminal must not skip the
    LOITER hand-back)."""
    try:
        print(*a, **k, flush=True)
    except Exception:
        pass


def install_signal_handlers() -> None:
    """SIGTERM / SIGHUP -> KeyboardInterrupt, i.e. the same clean exit as
    Ctrl-C (GUIDED -> LOITER if we were commanding). SIGHUP also detaches
    stdout/stderr from the dead terminal first. A SIGHUP that was already
    ignored (nohup) stays ignored. Only the first signal acts."""

    def handler(signum, _frame):
        if _SIGNAL["name"] is not None:
            return
        try:
            _SIGNAL["name"] = signal.Signals(signum).name
        except Exception:
            _SIGNAL["name"] = str(signum)
        if signum == getattr(signal, "SIGHUP", None):
            try:
                null = open(os.devnull, "w")
                sys.stdout = null
                sys.stderr = null
            except Exception:
                pass
        raise KeyboardInterrupt

    for name in ("SIGTERM", "SIGHUP"):
        sig = getattr(signal, name, None)
        if sig is None:
            continue
        try:
            if name == "SIGHUP" and signal.getsignal(sig) == signal.SIG_IGN:
                continue
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass


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
    ap.add_argument("--src-sysid", type=int, default=251,
                    help="bu scriptin kendi sysid'i (MAVProxy ve laptop relay 250 kullanir). "
                         "FC'de MAV_GCS_SYSID bu olursa GCS failsafe bu scripte baglanir.")

    # --- target input ---
    ap.add_argument("--target-link", default=None,
                    help="RADYO MODU: hedef verisinin geldigi 3DR (Orin USB'sinde), "
                         "orn /dev/serial/by-id/usb-...  (ls -l /dev/serial/by-id/). "
                         "Verilmezse ESKI yol: hedef paketleri takipci FC uzerinden gelir "
                         "(radyo FC'de, laptopta relay_target.py).")
    ap.add_argument("--target-baud", type=int, default=57600,
                    help="radyonun seri hizi (SiK SERIAL_SPEED=57 -> 57600)")
    ap.add_argument("--target-sysid", type=int, default=None,
                    help="hedef sysid. Radyo modunda varsayilan 0 = radyoda gorulen ilk arac "
                         "(Talon gibi takipciyle ayni sysid'de olabilir). Eski modda varsayilan 2.")
    ap.add_argument("--downlink-hz", type=float, default=2.0,
                    help="RADYO MODU: takipcinin konumunu yere bu hizda bas (Hz); HEARTBEAT 1 Hz, "
                         "fix 0.5 Hz ve olay metinleri de gider. 0 = radyoya sadece "
                         "TIMESYNC cevabi (Victus'un RTT olcumu).")

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
    ap.add_argument("--exit-on-pilot", action="store_true",
                    help="ESKI davranis: pilot takipciyi GUIDED'dan cikarirsa (LOITER vb.) script "
                         "CIKAR. Varsayilan: script DURAKLAR (komut gondermez, moda dokunmaz, "
                         "loglamaya devam eder); kumandadan tekrar GUIDED'a gecince takibe temiz "
                         "planlayici durumuyla DEVAM eder.")
    return ap


def resolve_args(a):
    """Defaults that depend on the layout (idempotent)."""
    if getattr(a, "target_sysid", None) is None:
        a.target_sysid = 0 if getattr(a, "target_link", None) else 2
    if not hasattr(a, "exit_on_pilot"):      # args built by hand (older callers)
        a.exit_on_pilot = False
    return a


class Orchestrator:
    def __init__(self, a, hub, *, observer=None, clock=time):
        self.a = resolve_args(a)
        self.hub = hub
        self.observer = observer or NullObserver()
        self.clock = clock
        self.radio = bool(getattr(a, "target_link", None))
        self.phase = Phase.WAIT
        self.link_log = None
        self.state_log = None
        self.cfg = None           # TrajectoryConfig (fixed chaser-home frame), kept across resumes
        self.ctl = None
        self.env = None
        self.commanding = False   # True once the chaser is GUIDED under our control
        self.exit_reason = "?"
        self._ev_key = None
        # pause/resume: the pilot took the chaser out of GUIDED (see _enter_pause/_resume)
        self.paused = False
        self.pauses = 0
        self.paused_total_s = 0.0
        self._pause_t0 = None
        self._pause_mode = None
        self._pause_say_t = 0.0
        self._sent_prev = 0       # counters of controllers retired by a resume
        self._holds_prev = 0

    # ------------------------------------------------ helpers
    def _on_msg(self, name, msg, t_mono):
        self.link_log.note_message(name, msg, t_mono)
        if name == "TAKIPCI" and msg.get_type() == "COMMAND_ACK":
            tgt = getattr(msg, "target_system", 0) or 0
            if tgt in (0, self.a.src_sysid):  # ours, not MAVProxy's or the laptop's
                self.link_log.cmd_lat.note_ack(msg, t_mono)

    def _event(self, text, *, severity=6, force=False) -> None:
        """Short status text to Victus (radio mode; no-op otherwise)."""
        fn = getattr(self.hub, "send_event", None)
        if fn is None:
            return
        try:
            fn(text, severity=severity, force=force)
        except Exception:
            pass

    def _tick_event(self, k) -> None:
        """One event per state change (behaviour / hold), never per tick."""
        key = "BEKLE" if k.why else k.behavior
        if key == self._ev_key:
            return
        self._ev_key = key
        if k.why:
            self._event(f"BEKLE {k.why}", severity=4)
        else:
            d = f" d={k.d:.0f}m" if k.d is not None else ""
            self._event(f"{k.behavior}{d}")

    def _print_vehicle(self, tag, s):
        if s.get("lat") is None:
            print(f"  {tag}: konum yok")
            return
        print(f"  {tag}: {s['lat']:.7f}, {s['lon']:.7f}  amsl={format_altitude(s.get('alt_amsl'))} "
              f"rel={format_altitude(s.get('alt_rel'))}  fix={s['fix']}/{s['nsat']}  mod={s['mode']}  "
              f"gelen={s['hz']:.1f} Hz")

    # ------------------------------------------------ controller + pause/resume
    def _make_controller(self) -> TrackController:
        """A brand-new TrackController (Kalman filter, speed/direction slews,
        setpoint hold and catch-up latch all start clean). Used at start and at
        every resume. self.cfg is reused, so the chaser-home NED frame stays the
        one the flight started with."""
        a = self.a
        return TrackController(
            self.cfg, self.env, alt_offset=a.alt_offset, sep_bias=a.sep_bias, alt_ref=a.alt_ref,
            min_alt=a.min_alt, min_sep=a.min_sep, max_dist=a.max_dist,
            max_age=a.max_age, posvel=a.posvel, dry_run=a.dry_run,
            cmd_lat=self.link_log.cmd_lat,
            att_max_age_s=max(1.0, 3.0 / a.att_hz) if a.att_hz > 0 else 1.0)

    def _totals(self):
        """(sent, holds) over the whole run, across controller restarts."""
        sent, holds = self._sent_prev, self._holds_prev
        if self.ctl is not None:
            sent += self.ctl.sent
            holds += self.ctl.holds
        return sent, holds

    def _enter_pause(self, cs, now) -> None:
        """The pilot took the chaser out of GUIDED (RC switch). From here we send
        NOTHING and never touch the mode: their switch is the safety. The loop
        keeps running (target stream, logs, status) so that the chase can be
        picked up again the moment GUIDED comes back."""
        self.paused = True
        self.pauses += 1
        self.phase = Phase.PAUSED
        self._pause_t0 = now
        self._pause_mode = cs["mode"]
        self._pause_say_t = now
        _say(f"\n[!] Takipci modu {cs['mode']} oldu -> DURAKLATILDI (komut yok, moda dokunulmuyor). "
             f"Kumandadan GUIDED'a alinca takip kendiliginden devam eder. Cikmak icin Ctrl-C.")
        self._event(f"chase_orin DURAKLADI: pilot ({cs['mode']})", severity=4, force=True)

    def _resume(self, cs, now) -> None:
        """The chaser is in GUIDED again. Start a fresh controller: nothing the
        planner remembered from before the pause (filter, slewed speed and
        direction, held setpoint, latches) is valid after the pilot flew the
        chaser. The speed is seeded from the real groundspeed on the first send,
        and DO_CHANGE_SPEED is re-sent for the first ticks because the FC resets
        its active speed to WP_SPD when GUIDED starts again."""
        dur = now - (self._pause_t0 if self._pause_t0 is not None else now)
        self.paused_total_s += dur
        self.paused = False
        self.phase = Phase.TRACK_PLAN
        self._sent_prev, self._holds_prev = self._totals()
        self.ctl = self._make_controller()
        self.env.force_speed_resend(3)
        self._ev_key = None          # the first tick after a resume announces its behaviour
        _say(f"\n[i] Takipci tekrar GUIDED ({dur:.0f} s duraklama, #{self.pauses}). "
             f"Takip temiz planlayici durumuyla devam ediyor.")
        self._event(f"chase_orin GUIDED: takip devam (#{self.pauses})", force=True)

    def _paused_tick(self, cs) -> Tick:
        """What a paused tick logs (no planner call, no command)."""
        a = self.a
        k = Tick(alt_ref=a.alt_ref, alt_offset_m=a.alt_offset, sep_bias_m=a.sep_bias)
        k.why = f"PILOT {cs['mode']}"
        return k

    # ------------------------------------------------ main flow
    def run(self) -> int:
        code, pilot_exit = 1, False
        try:
            code, pilot_exit = self._run()
            self.exit_reason = "pilot" if pilot_exit else "bitti"
        except KeyboardInterrupt:
            self.exit_reason = _SIGNAL["name"] or "Ctrl-C"
            _say("\n[i] Durduruldu." + (f" ({_SIGNAL['name']})" if _SIGNAL["name"] else ""))
            code = 0
        except Exception:
            self.exit_reason = "HATA"
            try:
                traceback.print_exc()
            except Exception:
                pass
            _say("[X] Beklenmeyen hata - durdu.")
            code = 1
        finally:
            self._finish(pilot_exit=pilot_exit)
        return code

    def _run(self):
        a, hub, C = self.a, self.hub, self.clock
        warn_offset_vs_minsep(a)
        self.link_log = LinkLogger(
            a.log_dir,
            meta={"script": "chase_orin", "fc": a.fc,
                  "target_link": a.target_link or "fc-forward",
                  "target_sysid": a.target_sysid, "max_speed_mps": a.max_speed,
                  "deadline_range_m": a.deadline_range, "alt_offset_m": a.alt_offset,
                  "alt_ref": a.alt_ref, "sep_bias_m": a.sep_bias,
                  "alt_match": int(a.alt_match), "posvel": int(a.posvel),
                  "rate_hz": a.rate, "pos_hz": a.pos_hz, "att_hz": a.att_hz,
                  "downlink_hz": a.downlink_hz if self.radio else 0,
                  "exit_on_pilot": int(a.exit_on_pilot)},
            enabled=not a.no_log)
        hub.on_msg = self._on_msg
        hub.start()
        hub.request_streams(pos_hz=a.pos_hz, att_hz=a.att_hz)

        if self.radio:
            tsys = "oto (radyodaki ilk arac)" if not a.target_sysid else str(a.target_sysid)
            print(f"\n[i] Takipci (sysid {a.chaser_sysid}) {a.fc} uzerinden, hedef RADYODAN "
                  f"({a.target_link} @ {a.target_baud}, sysid {tsys}) bekleniyor...")
        else:
            print(f"\n[i] {a.fc} uzerinden takipci (sysid {a.chaser_sysid}) ve "
                  f"hedef (sysid {a.target_sysid}) bekleniyor...")
        c_ok, t_ok = hub.wait_ready(a.wait_s)
        others = dict(hub.others)
        if others:
            print(f"  [i] FC baglantisinda baska sistemler de var (sysid:paket): {others} "
                  f"- 255 MAVProxy/QGC, 250 laptop relay olagan")
        if self.radio:
            r_others = dict(getattr(hub, "radio_others", {}))
            if r_others:
                print(f"  [i] radyoda hedef disi sistemler (sysid:paket): {r_others} "
                      f"- 250 Victus relay (TIMESYNC), 51 SiK radyonun kendi RADIO_STATUS'u olagan")
        if not c_ok:
            print("[X] Takipci heartbeat'i yok. MAVProxy calisiyor mu, --fc portu dogru mu?")
            return 2, False
        if not t_ok:
            if self.radio:
                n = getattr(hub, "radio_rx", 0)
                seen = (f"radyodan {n} paket geldi ama hedef konumu yok"
                        if n else "radyodan HIC paket gelmedi")
                print(f"[X] Hedef konumu gelmedi ({seen}). Kontrol et: (1) Victus'ta "
                      f"relay_radio.py calisiyor ve 'aktarim' artiyor mu, (2) iki 3DR eslesmis mi "
                      f"(LED'ler sabit; NetID ayni, hedef ciftininkinden farkli), (3) --target-link "
                      f"dogru mu (ls -l /dev/serial/by-id/), baud 57600 mu, (4) Victus'a hedef "
                      f"geliyor mu (Kirmizi/MP -> UDP 14550).")
            else:
                print("[X] Hedef konumu gelmedi. Kontrol et: (1) laptop'ta relay_target.py calisiyor "
                      "mu, (2) hedefte SYSID_THISMAV=2 mi, (3) takipci radyo linki saglam mi, "
                      "(4) takipci FC'de MAVLink yonlendirmesi kapali olmasin (SERIALx_OPTIONS).")
            return 2, False

        self._wait_fixes(a.wait_s)
        ts, cs = hub.target.snapshot(), hub.chaser.snapshot()
        self._print_vehicle("HEDEF  ", ts)
        self._print_vehicle("TAKIPCI", cs)
        if has_fix(ts) and has_fix(cs):
            print(f"  aralarindaki mesafe: "
                  f"{haversine_m(cs['lat'], cs['lon'], ts['lat'], ts['lon']):.0f} m")
            print_initial_separation(a, ts, cs)
        if not has_fix(cs):
            print("[X] Takipci 3D fix olmadan yorunge cercevesi kurulamaz.")
            return 2, False

        self.cfg = build_traj_config(a, cs["lat"], cs["lon"])
        self.env = ChaserEnv(hub.chaser_handle(), verbose_speed=False,
                             cmd_latency=self.link_log.cmd_lat)
        self.env.enable_posvel(a.posvel)
        self.ctl = self._make_controller()
        self.state_log = StateLog(self.link_log.log_dir / f"chase_state_{self.link_log.stamp}.csv",
                                  meta={"script": "chase_orin"}, enabled=not a.no_log)

        print_plan_banner(a)
        print("  beyin  : Orin (komutlar USB'den, radyodan degil). Faz: TRACK_PLAN. "
              "VISUAL/REALIGN: kapali (Asama 3/4).")
        if self.radio:
            dl = (f"{a.downlink_hz:g} Hz takipci konumu + olaylar" if a.downlink_hz > 0
                  else "kapali (sadece TIMESYNC cevabi)")
            print(f"  hedef  : radyo {a.target_link} @ {a.target_baud}; yere: {dl}")
        else:
            print("  hedef  : takipci FC uzerinden (eski yol, relay_target.py)")
        if a.dry_run:
            print("\n--dry-run: FC'ye komut gonderilmiyor.\n")
            self._event("chase_orin DRY-RUN basladi", force=True)
        else:
            print("\nOnay icin 'evet' yaz:")
            if input("> ").strip().lower() != "evet":
                print("[i] Iptal.")
                return 0, False
            # From here on the chaser may be GUIDED under our control: any exit
            # (Ctrl-C/signal even mid mode-switch) hands it back to LOITER if,
            # and only if, it is actually in GUIDED at that moment.
            self.commanding = True
            if a.set_mode:
                if not hub.set_chaser_mode("GUIDED"):
                    print("[X] Takipci GUIDED moduna gecmedi.")
                    return 2, False
            else:
                print("[i] Takipciyi kumandadan GUIDED moduna al. Bekleniyor...")
                while hub.chaser.snapshot()["mode"] != "GUIDED":
                    C.sleep(0.3)
                print("[i] Takipci GUIDED modunda.")
            self.env.configure_guided_speed_limits(speed_mps=float(a.max_speed),
                                                   accel_mps2=float(a.accel))
            self.env.verify_wp_spd(float(a.max_speed))
            self._event("chase_orin GUIDED: takip basladi", force=True)

        if a.exit_on_pilot:
            print("Ctrl-C -> takipci LOITER. Kumanda her an ezer: GUIDED'dan cikarirsan script CIKAR "
                  "(--exit-on-pilot).\n")
        else:
            print("Ctrl-C -> takipci LOITER. Kumanda her an ezer: GUIDED'dan cikarirsan (LOITER vb.) "
                  "script DURAKLAR, komut gondermez; GUIDED'a alinca takip DEVAM eder.\n")
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

    def _wait_fixes(self, timeout_s: float) -> None:
        """GPS_RAW_INT (the fix) arrives at ~1 Hz, after HEARTBEAT/position:
        give both fixes up to timeout_s to show up before judging them
        (field bug 25 Oct: 'Takipci 3D fix olmadan' on a chaser that had GPS)."""
        hub, C = self.hub, self.clock
        t_end = C.monotonic() + float(timeout_s)
        last_say = -1e9
        while True:
            cs, ts = hub.chaser.snapshot(), hub.target.snapshot()
            if has_fix(cs) and has_fix(ts):
                return
            now = C.monotonic()
            if now >= t_end:
                return
            if now - last_say >= 2.0:
                last_say = now
                print(f"  [i] GPS fix bekleniyor: TAKIPCI fix={cs['fix']}/{cs['nsat']}  "
                      f"HEDEF fix={ts['fix']}/{ts['nsat']}  (en fazla {timeout_s:.0f} s)")
            C.sleep(0.2)

    def _track_plan_until_lock(self) -> str:
        """TRACK_PLAN until a lock (Asama 3, gated off). The pilot leaving GUIDED does
        NOT end it: the loop PAUSES (no commands, mode untouched, logging goes on)
        and resumes with a fresh controller when GUIDED is back. Only
        --exit-on-pilot makes it return "pilot" instead (the old behaviour)."""
        a, hub, C = self.a, self.hub, self.clock
        self.phase = Phase.PAUSED if self.paused else Phase.TRACK_PLAN
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

            # Who has the chaser? Dry-run never commands, so it never pauses.
            pilot_has_it = (not a.dry_run) and cs["mode"] != "GUIDED"
            if pilot_has_it and a.exit_on_pilot:
                print(f"\n[!] Takipci modu {cs['mode']} oldu. Kontrol kumandada, cikiliyor.")
                return "pilot"
            if pilot_has_it and not self.paused:
                self._enter_pause(cs, now)
            elif pilot_has_it and cs["mode"] != self._pause_mode:
                self._pause_mode = cs["mode"]            # pilot moved on to another mode
                _say(f"  [i] DURAKLATILDI: pilot modu simdi {cs['mode']}")
            elif not pilot_has_it and self.paused:
                self._resume(cs, now)
                last_beh, last_print = None, -1e9        # print the first resumed tick at once

            if now - last_stream_req > 5.0:
                try:
                    hub.request_streams(pos_hz=a.pos_hz, att_hz=a.att_hz)
                except Exception:
                    pass
                last_stream_req = now
            if (not self.paused and not a.dry_run and a.rtt_ping_s > 0
                    and now - last_ping >= a.rtt_ping_s):
                try:
                    self.env.ping_rtt(now)
                except Exception:
                    pass
                last_ping = now

            # While paused nothing is planned and nothing is sent.
            k = self._paused_tick(cs) if self.paused else self.ctl.tick(ts, cs, now)
            self.observer.on_tick(t_mono=now, t_wall=wall, phase=self.phase, tick=k, ts=ts, cs=cs)
            self.state_log.write(t_wall=wall, t_mono=now, phase=self.phase.value, k=k, ts=ts, cs=cs)
            self.link_log.note_relay(
                t_mono=now, updated=k.updated,
                target_age_ms=ts["age"] * 1e3, chaser_age_ms=cs["age"] * 1e3,
                compute_ms=k.compute_ms, send_ms=k.send_ms,
                behavior=("pilot" if self.paused else ("hold" if k.held else "wait"))
                if k.why else k.behavior,
                follow_phase=k.why[:16] if k.why else k.follow_phase,
                v_cmd_mps=k.v_cmd, dist_m=k.d, sep_m=k.sep, gate_alt_m=k.cmd_alt,
                target_hz=ts["hz"], chaser_hz=cs["hz"])
            if not self.paused:
                self._tick_event(k)

            changed = (k.behavior != last_beh)
            last_beh = k.behavior
            if self.paused:
                if now - self._pause_say_t >= 5.0:       # a reminder, not a flood
                    self._pause_say_t = now
                    _say(f"  [DURAKLATILDI] mod={cs['mode']} {now - self._pause_t0:.0f} s - "
                         f"komut yok; kumandadan GUIDED'a al -> takip devam")
            elif changed or now - last_print >= print_dt:
                last_print = now
                if k.why:
                    print(f"  [BEKLE] {k.why}" + ("  -> takipci yerinde tutuluyor" if k.held else ""))
                else:
                    v = k.v_cmd if k.v_cmd is not None else 0.0
                    print(f"  d={k.d:6.1f} m  ayrim={k.sep:+5.1f} m  beh={k.behavior:9s} "
                          f"fp={k.follow_phase:5s}  v_cmd={v:4.1f}  komut_alt={k.cmd_alt:5.1f} m  "
                          f"hedef_hiz={k.target_speed:4.1f}  hedef_yas={ts['age']:.2f}s "
                          f"({ts['hz']:.1f}Hz)  yon={k.heading_src}  n={self.ctl.sent}")

            # Asama 3 handoff point (gated): stable lock AND the planner allows it.
            if (VISUAL_ENABLED and not self.paused
                    and self.observer.lock_ready() and k.allow_visual_lock):
                return "lock"

    def _visual_phase(self) -> str:  # Asama 3
        raise NotImplementedError("VISUAL fazi Asama 3'te baglanacak")

    def _finish(self, *, pilot_exit: bool) -> None:
        """Runs exactly once. The mode is touched ONLY if we were commanding
        and the pilot did not take over: then GUIDED -> LOITER. A pilot's
        mode choice is never overridden (their RC abort is the safety)."""
        hub = self.hub
        if self.paused and self._pause_t0 is not None:   # left while the pilot had it
            try:
                self.paused_total_s += self.clock.monotonic() - self._pause_t0
            except Exception:
                pass
            self._pause_t0 = None
        if self.ctl is not None:
            sent, holds = self._totals()
            more = f" duraklama={self.pauses} ({self.paused_total_s:.0f} s)" if self.pauses else ""
            _say(f"[i] gonderim={sent} yerinde-tut={holds}{more}")
        event = f"chase_orin cikis ({self.exit_reason})"
        try:
            if self.commanding:
                mode = hub.chaser.snapshot()["mode"]
                if pilot_exit or mode != "GUIDED":
                    _say(f"[i] Kontrol pilotta (mod={mode}); mod degistirilmiyor.")
                    event = (f"chase_orin cikis: pilot ({mode})" if pilot_exit
                             else f"chase_orin cikis ({self.exit_reason}), mod={mode}")
                else:
                    _say("[i] Takipci LOITER'a aliniyor...")
                    try:
                        hub.set_chaser_mode("LOITER")
                    except Exception:
                        pass
                    event = f"chase_orin cikis ({self.exit_reason}) -> LOITER"
        finally:
            self._event(event, severity=4 if self.commanding else 6, force=True)
            self.observer.close()
            if self.state_log is not None:
                self.state_log.close()
            if self.link_log is not None:
                self.link_log.close()
            hub.stop()


def main(argv=None) -> int:
    parser = build_parser()
    a = resolve_args(parser.parse_args(argv))
    try:
        validate_altitude_options(a.alt_ref, a.sep_bias, a.alt_offset)
    except ValueError as e:
        parser.error(str(e))
    install_signal_handlers()
    try:
        hub = MavHub.connect(a.fc, baud=a.fc_baud, source_system=a.src_sysid,
                             chaser_sysid=a.chaser_sysid, target_sysid=a.target_sysid,
                             target_device=a.target_link, target_baud=a.target_baud,
                             downlink_hz=a.downlink_hz)
    except ValueError as e:
        print(f"[X] {e}")
        return 2
    except OSError as e:  # serial.SerialException is an OSError
        print(f"[X] Radyo/FC portu acilamadi: {e}\n"
              f"    ls -l /dev/serial/by-id/  (yol dogru mu?)  |  groups  (dialout var mi?)  |  "
              f"dmesg | tail  (CH340 ise brltty portu kapmis olabilir)")
        return 2
    return Orchestrator(a, hub).run()


if __name__ == "__main__":
    sys.exit(main())
