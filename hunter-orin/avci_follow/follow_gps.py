#!/usr/bin/env python3
"""
follow_gps.py — laptop-side GPS chase for drone-to-drone filming.

The chaser drone autonomously chases (and cuts in front of) a target drone the
pilot flies by hand, using GPS only — no camera, no companion computer on the
chaser. Every command is computed on a Windows laptop from two telemetry radios
and sent to the chaser over MAVLink GUIDED.

  HEDEF (pilot flies)  --telem-->  laptop  --telem-->  TAKIPCI (GUIDED)
       COM_target                    |                    COM_chaser
                                     | UDP trigger (optional)
                                     v
                            ORIN (record_agent.py)

The "brain" is the vendored fpv_gate_il.trajectory planner (radar/GPS -> Kalman
-> INTERCEPT/FOLLOW -> sliding carrot), driven against the real chaser through
chaser_env.ChaserEnv. Behaviour selection is unrestricted: when the target
comes at the chaser head-on the planner picks INTERCEPT and flies to a point
*ahead* of the target (cuts in front); crossing/opening geometry falls to
FOLLOW (trails on the course line). Speed is adaptive — up to --max-speed far
out, tapering to --close-approach inside --deadline-range by design.

The transport layer (two Vehicles, receive threads, snapshots, sysid check,
recording trigger, safety gates) is lifted from the proven follow_relay.py; a
small on_msg hook feeds link_log for telemetry diagnostics. The per-tick brain
is chase_core.TrackController, shared with the Orin front-end (chase_orin.py).

Gate failures (target stale / no fix / beyond --max-dist / --min-sep) HOLD the
chaser at its current position (the sim's --stale-hold), instead of letting it
keep flying toward the last carrot.

ALWAYS dry-run first to read the true vertical separation, then set --sep-bias:
  python follow_gps.py --target-port COM12 --chaser-port COM5 --dry-run --no-rec

Typical filming run (defaults are filming-ready: clean altitude mode on, 3 m
below, full 20 m/s until ~200 m, 10 m/s inside 100 m):
  python follow_gps.py --target-port COM12 --chaser-port COM5 --sep-bias <m> --set-mode

Ctrl-C (or an unexpected error) -> chaser LOITER. The RC mode switch overrides
at any instant; if the chaser leaves GUIDED this script exits and never touches
the mode again (the pilot's choice stands).

SAFETY (read once): two real drones, one cutting in front of the other at up to
20 m/s with a few metres of GPS/EKF error, is a genuine collision risk —
especially INTERCEPT with a small |--alt-offset|. The default -3 m keeps you off
exactly-level (the worst case), but for aggressive head-on cut-ins give yourself
more margin (e.g. --alt-offset -8) and keep a hand on the RC. --min-sep is an
optional measured-vertical-separation backstop (0 = off); with a same-altitude
run (--alt-offset 0) you must set it 0 (it would otherwise trip on the intended
close pass). See README.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import socket
import sys
import threading
import time
import traceback

from pymavlink import mavutil

# The trajectory brain (vendored planner + session glue) lives in chase_core so
# the laptop (this file) and the Orin (chase_orin.py) fly identical logic.
from chase_core import (
    TrackController,
    add_planner_args,
    build_traj_config,
    chaser_home_amsl,
    has_fix,
    haversine_m,
    print_plan_banner,
    warn_offset_vs_minsep,
)
from chaser_env import ChaserEnv
from link_log import LinkLogger


# ---------------------------------------------------------------- recording trigger
class Recorder:
    """UDP trigger to record_agent.py on the Orin (lifted from follow_relay).

    IDLE   -> dist < start AND closing -> ACTIVE  (start)
    ACTIVE -> dist < stop              -> LOCKOUT (stop)
           -> dist > start + hyst      -> IDLE    (cancel a fly-by)
    LOCKOUT-> dist > start + hyst      -> IDLE    (re-arm)
    """

    def __init__(self, host, port, start_d, stop_d, hyst=30.0, window=4.0, deadband=2.0):
        self.addr = (host, port)
        self.start_d, self.stop_d = start_d, stop_d
        self.hyst, self.window, self.deadband = hyst, window, deadband
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.4)
        self.hist = collections.deque()
        self.state = "IDLE"
        self.tag = None

    def closing(self, d, now):
        self.hist.append((now, d))
        while self.hist and now - self.hist[0][0] > self.window:
            self.hist.popleft()
        if len(self.hist) < 3:
            return False
        return (self.hist[0][1] - d) > self.deadband

    def send(self, payload):
        raw = json.dumps(payload).encode()
        for _ in range(3):
            try:
                self.sock.sendto(raw, self.addr)
                ack, _ = self.sock.recvfrom(1024)
                return json.loads(ack.decode())
            except socket.timeout:
                continue
            except Exception as e:
                return {"ok": False, "err": str(e)}
        return {"ok": False, "err": "ACK yok - Orin ulasilamiyor?"}

    def update(self, d, now):
        closing = self.closing(d, now)
        rearm_d = self.start_d + self.hyst
        if self.state == "IDLE":
            if d < self.start_d and closing:
                self.tag = time.strftime("%Y%m%d_%H%M%S")
                r = self.send({"cmd": "start", "tag": self.tag, "dist": round(d, 1)})
                if r.get("ok"):
                    self.state = "ACTIVE"
                    return f"KAYIT BASLADI  d={d:.0f} m  dosya={r.get('file', '?')}"
                return f"[!] KAYIT BASLATILAMADI: {r.get('err')}"
        elif self.state == "ACTIVE":
            if d < self.stop_d:
                r = self.send({"cmd": "stop"})
                self.state = "LOCKOUT"
                return (f"KAYIT DURDU (hedefe {d:.0f} m)  sure={r.get('secs', '?')}s  "
                        f"boyut={r.get('mb', '?')} MB")
            if d > rearm_d:
                self.send({"cmd": "stop"})
                self.state = "IDLE"
                return f"KAYIT IPTAL (uzaklasildi, d={d:.0f} m)"
        elif self.state == "LOCKOUT":
            if d > rearm_d:
                self.state = "IDLE"
                return f"kayit yeniden kuruldu (d={d:.0f} m > {rearm_d:.0f} m)"
        return None

    def shutdown(self):
        if self.state == "ACTIVE":
            print(f"[i] Kayit kapatiliyor: {self.send({'cmd': 'stop'})}")


# ---------------------------------------------------------------- vehicle
class Vehicle:
    """Telemetry link + receive thread (lifted from follow_relay, plus on_msg).

    The receive thread is the ONLY reader of the link. It records what the
    brain needs (position with an exact arrival stamp, fix, mode, attitude)
    and PARAM_VALUE replies, which ChaserEnv waits on via wait_param() rather
    than reading the link itself (two readers on one link race each other).

    on_msg(name, msg, t_mono) is called for every received message so link_log
    can record per-message health without changing the snapshot contract.
    """

    def __init__(self, name, port, baud, stream_hz=10.0, on_msg=None, att_hz=0.0):
        self.name = name
        self.stream_hz = stream_hz
        self.att_hz = float(att_hz or 0.0)
        self.on_msg = on_msg
        print(f"[{name}] {port} @ {baud} baglaniliyor...")
        self.conn = mavutil.mavlink_connection(port, baud=baud, source_system=250)
        if self.conn.wait_heartbeat(timeout=15) is None:
            sys.exit(f"[X] {name}: heartbeat yok. Port/baud dogru mu?")
        t0 = time.time()
        while self.conn.target_system == 0 and time.time() - t0 < 5:
            self.conn.wait_heartbeat(timeout=1)
        self.sysid = self.conn.target_system
        print(f"[{name}] BAGLI sysid={self.sysid}")

        self.lat = self.lon = None
        self.alt_amsl = self.alt_rel = None
        self.vx = self.vy = self.vz = 0.0
        self.fix = self.nsat = 0
        self.mode = "?"
        self.armed = False
        self.last_pos_t = 0.0
        self.pos_mono = None          # exact monotonic arrival of the newest position
        self.last_raw = None
        self.roll = self.pitch = self.yaw = None
        self.last_att_mono = None
        self.pos_times = collections.deque(maxlen=40)
        self.lock = threading.Lock()
        self._params = {}             # name -> (value, t_mono)
        self._param_cv = threading.Condition()

        self._request_streams()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _request_streams(self):
        # Addressed to our own vehicle explicitly (sysid locked at connect).
        # 33 = GLOBAL_POSITION_INT, 24 = GPS_RAW_INT, 30 = ATTITUDE (yaw = the
        # planner's "nose" while hovering, as in the sim).
        tsys, tcomp = self.sysid, self.conn.target_component
        intervals = [(33, self.stream_hz), (24, 1.0)]
        if self.att_hz > 0:
            intervals.append((30, self.att_hz))
        for msg_id, hz in intervals:
            self.conn.mav.command_long_send(
                tsys, tcomp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        self.conn.mav.request_data_stream_send(
            tsys, tcomp, mavutil.mavlink.MAV_DATA_STREAM_POSITION, int(self.stream_hz), 1)

    def _rx_loop(self):
        while True:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1)
            except Exception as e:
                print(f"\n[{self.name}] link hatasi: {e}")
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            t = msg.get_type()
            t_mono = time.monotonic()
            with self.lock:
                if t == "GLOBAL_POSITION_INT":
                    self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
                    self.alt_amsl = msg.alt / 1000.0
                    self.alt_rel = msg.relative_alt / 1000.0
                    self.vx = msg.vx / 100.0
                    self.vy = msg.vy / 100.0
                    self.vz = msg.vz / 100.0
                    self.last_pos_t = time.time()
                    self.pos_mono = t_mono
                    self.pos_times.append(self.last_pos_t)
                    self.last_raw = msg.get_msgbuf()
                elif t == "GPS_RAW_INT":
                    self.fix, self.nsat = msg.fix_type, msg.satellites_visible
                elif t == "HEARTBEAT" and msg.get_srcSystem() == self.sysid:
                    self.mode = mavutil.mode_string_v10(msg)
                    self.armed = bool(msg.base_mode &
                                      mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "ATTITUDE" and msg.get_srcSystem() == self.sysid:
                    self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
                    self.last_att_mono = t_mono
                elif t == "STATUSTEXT":
                    print(f"\n[{self.name} FC] {msg.text}")
            if t == "PARAM_VALUE" and msg.get_srcSystem() == self.sysid:
                self._note_param(msg, t_mono)
            # link telemetry logging (outside the state lock)
            if self.on_msg is not None:
                try:
                    self.on_msg(self.name, msg, t_mono)
                except Exception:
                    pass

    def _note_param(self, msg, t_mono):
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("utf-8", errors="ignore")
        pid = pid.rstrip("\x00")
        with self._param_cv:
            self._params[pid] = (float(msg.param_value), t_mono)
            self._param_cv.notify_all()

    def wait_param(self, name, *, since_mono, timeout):
        """Value of `name` from a PARAM_VALUE that arrived after since_mono,
        or None on timeout. Fed by the receive thread (the only reader)."""
        deadline = time.monotonic() + float(timeout)
        with self._param_cv:
            while True:
                rec = self._params.get(name)
                if rec is not None and rec[1] >= since_mono:
                    return rec[0]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._param_cv.wait(remaining)

    def snapshot(self):
        with self.lock:
            ts = list(self.pos_times)
            hz = 0.0
            if len(ts) >= 3 and ts[-1] > ts[0]:
                hz = (len(ts) - 1) / (ts[-1] - ts[0])
            now_mono = time.monotonic()
            att_age = (now_mono - self.last_att_mono) if self.last_att_mono is not None else None
            return dict(lat=self.lat, lon=self.lon, alt_amsl=self.alt_amsl,
                        alt_rel=self.alt_rel, vx=self.vx, vy=self.vy, vz=self.vz,
                        fix=self.fix, nsat=self.nsat, mode=self.mode,
                        armed=self.armed, age=time.time() - self.last_pos_t,
                        hz=hz, raw=self.last_raw, pos_mono=self.pos_mono,
                        roll=self.roll, pitch=self.pitch, yaw=self.yaw, att_age=att_age)

    @staticmethod
    def home_amsl(s):
        """Chaser home AMSL = alt_amsl - alt_rel (see chase_core)."""
        return chaser_home_amsl(s)

    def set_mode(self, name, timeout=4.0):
        mapping = self.conn.mode_mapping()
        if name not in mapping:
            print(f"[{self.name}] bilinmeyen mod: {name}")
            return False
        self.conn.set_mode(mapping[name])
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.2)
            if self.mode == name:
                print(f"[{self.name}] mod = {name}")
                return True
        print(f"[{self.name}] mod {name} dogrulanamadi (su an: {self.mode})")
        return False


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # --- links ---
    ap.add_argument("--target-port", required=True, help="hedef drone, or: COM12")
    ap.add_argument("--chaser-port", required=True, help="takipci drone, or: COM5")
    ap.add_argument("--target-baud", type=int, default=57600)
    ap.add_argument("--chaser-baud", type=int, default=57600)
    ap.add_argument("--rate", type=float, default=10.0,
                    help="kontrol dongusu / carrot yeniden gonderme hizi (Hz). "
                         "Carrot her tikte kaydigi icin stream-hz'ten yuksek olabilir.")
    ap.add_argument("--stream-hz", type=float, default=10.0,
                    help="telemetriden konum isteme hizi (Hz). Link bogulursa dusur.")
    ap.add_argument("--att-hz", type=float, default=2.0,
                    help="takipci ATTITUDE isteme hizi (Hz). Planlayici hover'da "
                         "burun yonu olarak yaw kullanir (sim ile ayni). 0 = isteme.")

    # --- planner / altitude / safety / posvel: shared with the Orin (chase_core) ---
    add_planner_args(ap)

    # --- recording (kept as flags) ---
    ap.add_argument("--rec-start", type=float, default=150.0, help="bu mesafede kayda basla (m)")
    ap.add_argument("--rec-stop", type=float, default=20.0, help="bu mesafede kaydi durdur (m)")
    ap.add_argument("--rec-host", default="192.168.144.50", help="Orin IP")
    ap.add_argument("--rec-port", type=int, default=5599)
    ap.add_argument("--no-rec", action="store_true", help="kayit tetigini kapat")

    # --- logging ---
    ap.add_argument("--no-log", action="store_true",
                    help="telemetri/gecikme loglamayi kapat (varsayilan ACIK)")
    ap.add_argument("--log-dir", default="logs", help="log klasoru")
    ap.add_argument("--rtt-ping-s", type=float, default=1.0,
                    help="komut->tepki RTT icin bu araliklarla takipciye zararsiz "
                         "ACK-uyandiran ping (s). 0 = kapali. Sadece gercek uctuda "
                         "(dry-run'da komut gonderilmez).")

    # --- run control ---
    ap.add_argument("--set-mode", action="store_true", help="modu script degistirsin (GUIDED)")
    ap.add_argument("--dry-run", action="store_true", help="FC'ye komut gonderme")
    a = ap.parse_args()

    if a.rec_stop >= a.rec_start:
        sys.exit("[X] --rec-stop, --rec-start'tan kucuk olmali.")
    warn_offset_vs_minsep(a)

    # link logger first, so we hook both vehicles' receive threads
    link_log = LinkLogger(
        a.log_dir,
        meta={
            "script": "follow_gps",
            "max_speed_mps": a.max_speed,
            "deadline_range_m": a.deadline_range,
            "close_approach_mps": a.close_approach,
            "alt_offset_m": a.alt_offset,
            "alt_match": int(a.alt_match),
            "posvel": int(a.posvel),
            "stream_hz": a.stream_hz,
            "rate_hz": a.rate,
        },
        enabled=not a.no_log,
    )

    target = Vehicle("HEDEF  ", a.target_port, a.target_baud, a.stream_hz,
                     on_msg=link_log.note_message)

    def _chaser_on_msg(name, msg, t_mono):
        # link telemetry for the chaser radio + command->response ACK matching
        link_log.note_message(name, msg, t_mono)
        link_log.cmd_lat.note_ack(msg, t_mono)

    chaser = Vehicle("TAKIPCI", a.chaser_port, a.chaser_baud, a.stream_hz,
                     on_msg=_chaser_on_msg, att_hz=a.att_hz)

    if target.sysid == chaser.sysid:
        print(f"\n[!] IKI DRONE DE sysid={target.sysid}. GUIDED'da calisir ama "
              f"karisikliga acik; hedefte SYSID_THISMAV=2 onerilir.")

    rec = None
    if not a.no_rec:
        rec = Recorder(a.rec_host, a.rec_port, a.rec_start, a.rec_stop)
        print(f"\n[i] Kayit ajani yoklaniyor ({a.rec_host}:{a.rec_port})...")
        r = rec.send({"cmd": "ping"})
        if r.get("ok"):
            print(f"[i] Orin ajani hazir: {r.get('msg', '')}")
        else:
            print(f"[!] Orin ajani cevap vermiyor: {r.get('err')}")
            if input("Kayitsiz devam edilsin mi? (evet) > ").strip().lower() != "evet":
                sys.exit("[i] Iptal.")
            rec = None

    print("\ndurum okunuyor...")
    time.sleep(3)
    ts, cs = target.snapshot(), chaser.snapshot()
    if ts["lat"] is None or cs["lat"] is None:
        sys.exit("[X] Iki drone'dan da konum gelmedi.")

    print(f"  HEDEF  : {ts['lat']:.7f}, {ts['lon']:.7f}  amsl={ts['alt_amsl']:.1f} "
          f"rel={ts['alt_rel']:.1f}  fix={ts['fix']}/{ts['nsat']}  mod={ts['mode']}  "
          f"gelen={ts['hz']:.1f} Hz")
    print(f"  TAKIPCI: {cs['lat']:.7f}, {cs['lon']:.7f}  amsl={cs['alt_amsl']:.1f} "
          f"rel={cs['alt_rel']:.1f}  fix={cs['fix']}/{cs['nsat']}  mod={cs['mode']}  "
          f"gelen={cs['hz']:.1f} Hz")
    print(f"  [i] istenen {a.stream_hz:.0f} Hz - 'gelen' bunun cok altindaysa "
          f"telemetri linki doymus demektir")
    if has_fix(ts) and has_fix(cs):
        print(f"  aralarindaki mesafe: "
              f"{haversine_m(cs['lat'], cs['lon'], ts['lat'], ts['lon']):.0f} m")
        print(f"  olculen dikey ayrim: {ts['alt_amsl'] - cs['alt_amsl']:+.1f} m "
              f"(+ = takipci altta)")
    else:
        for tag, st in (("HEDEF", ts), ("TAKIPCI", cs)):
            if not has_fix(st):
                print(f"  [!] {tag} 3D fix'e sahip degil (fix={st['fix']}, sat={st['nsat']})")
        print("  [!] GUIDED iki tarafta da GPS ister.")

    # --- fixed NED origin = chaser startup lat/lon (planner works in this frame) ---
    if not has_fix(cs):
        sys.exit("[X] Takipci 3D fix olmadan yorunge cercevesi kurulamaz.")
    cfg = build_traj_config(a, cs["lat"], cs["lon"])
    chaser_env = ChaserEnv(chaser, verbose_speed=False, cmd_latency=link_log.cmd_lat)
    chaser_env.enable_posvel(a.posvel)
    ctl = TrackController(
        cfg, chaser_env, alt_offset=a.alt_offset, sep_bias=a.sep_bias,
        min_alt=a.min_alt, min_sep=a.min_sep, max_dist=a.max_dist,
        max_age=a.max_age, posvel=a.posvel, dry_run=a.dry_run,
        cmd_lat=link_log.cmd_lat,
        att_max_age_s=max(1.0, 3.0 / a.att_hz) if a.att_hz > 0 else 1.0,
    )

    print_plan_banner(a)
    if rec:
        print(f"  kayit  : {a.rec_start:.0f} m'de basla, {a.rec_stop:.0f} m'de dur, "
              f"sadece yaklasirken")

    if a.dry_run:
        print("\n--dry-run: FC'ye komut gonderilmiyor.\n")
    else:
        print("\nOnay icin 'evet' yaz:")
        if input("> ").strip().lower() != "evet":
            sys.exit("[i] Iptal.")
        if a.set_mode:
            if not chaser.set_mode("GUIDED"):
                sys.exit("[X] Takipci GUIDED moduna gecmedi.")
        else:
            print("[i] Takipciyi kumandadan GUIDED moduna al. Bekleniyor...")
            while chaser.mode != "GUIDED":
                time.sleep(0.3)
            print("[i] Takipci GUIDED modunda.")
        # raise WP_SPD ceiling + verify (this is the 'always 10 m/s' fix)
        chaser_env.configure_guided_speed_limits(
            speed_mps=float(a.max_speed), accel_mps2=float(a.accel))
        chaser_env.verify_wp_spd(float(a.max_speed))

    print("Ctrl-C -> takipci LOITER. Kumanda her an ezer.\n")
    period = 1.0 / a.rate
    skipped = 0
    pilot_exit = False
    last_stream_req = time.monotonic()
    last_ping = time.monotonic()
    warned_course = False

    try:
        while True:
            loop_t = time.time()
            now = time.monotonic()
            ts, cs = target.snapshot(), chaser.snapshot()

            # pilot took over? -> hand back, never touch the mode again
            if not a.dry_run and cs["mode"] not in ("GUIDED",):
                print(f"\n[!] Takipci modu {cs['mode']} oldu. Kontrol kumandada, cikiliyor.")
                pilot_exit = True
                break

            # periodic stream re-request (safe on a lossy link; never re-asserts mode)
            if now - last_stream_req > 5.0:
                try:
                    target._request_streams()
                    chaser._request_streams()
                except Exception:
                    pass
                last_stream_req = now

            # periodic command->response RTT probe (real run only; read-only on FC)
            if (not a.dry_run and a.rtt_ping_s > 0
                    and now - last_ping >= a.rtt_ping_s):
                try:
                    chaser_env.ping_rtt(now)
                except Exception:
                    pass
                last_ping = now

            k = ctl.tick(ts, cs, now)

            if k.why:
                skipped += 1
                print(f"  [BEKLE] {k.why}" + ("  -> takipci yerinde tutuluyor" if k.held else ""))
                link_log.note_relay(
                    t_mono=now, updated=False,
                    target_age_ms=ts["age"] * 1e3, chaser_age_ms=cs["age"] * 1e3,
                    compute_ms=None, send_ms=None,
                    behavior="hold" if k.held else "wait", follow_phase=k.why[:16],
                    v_cmd_mps=None, dist_m=k.d, sep_m=k.sep,
                    gate_alt_m=None, target_hz=ts["hz"], chaser_hz=cs["hz"])
            else:
                if rec:
                    evt = rec.update(k.d, loop_t)
                    if evt:
                        print(f"  >>> {evt}")
                if k.heading_src == "course" and not warned_course and a.att_hz > 0:
                    print("  [!] takipci ATTITUDE gelmiyor - hover'da burun yerine hiz "
                          "yonu kullaniliyor (--att-hz / link kontrol)")
                    warned_course = True
                rstate = rec.state if rec else "-"
                vshow = k.v_cmd if k.v_cmd is not None else 0.0
                print(f"  d={k.d:6.1f} m  ayrim={k.sep:+5.1f} m  beh={k.behavior:9s} "
                      f"fp={k.follow_phase:5s}  v_cmd={vshow:4.1f}  komut_alt={k.cmd_alt:5.1f} m  "
                      f"hedef_hiz={k.target_speed:4.1f}  yas={ts['age']:.2f}s  "
                      f"hz={ts['hz']:.1f}  rec={rstate}  n={ctl.sent}")
                link_log.note_relay(
                    t_mono=now, updated=k.updated,
                    target_age_ms=ts["age"] * 1e3, chaser_age_ms=cs["age"] * 1e3,
                    compute_ms=k.compute_ms, send_ms=k.send_ms, behavior=k.behavior,
                    follow_phase=k.follow_phase, v_cmd_mps=k.v_cmd, dist_m=k.d,
                    sep_m=k.sep, gate_alt_m=k.cmd_alt,
                    target_hz=ts["hz"], chaser_hz=cs["hz"])

            time.sleep(max(0.0, period - (time.time() - loop_t)))

    except KeyboardInterrupt:
        print("\n[i] Durduruldu.")
    except Exception:
        traceback.print_exc()
        print("[X] Beklenmeyen hata - dongu durdu.")
    finally:
        print(f"[i] gonderim={ctl.sent} yerinde-tut={ctl.holds} atlanan={skipped}")
        link_log.close()
        if rec:
            rec.shutdown()
        if not a.dry_run:
            if pilot_exit or chaser.mode != "GUIDED":
                # The pilot chose a mode: never override it (their abort is the safety).
                print(f"[i] Kontrol pilotta (mod={chaser.mode}); mod degistirilmiyor.")
            else:
                print("[i] Takipci LOITER'a aliniyor...")
                chaser.set_mode("LOITER")
                time.sleep(1)


if __name__ == "__main__":
    main()
