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
recording trigger, LOITER-on-exit, safety gates) is lifted from the proven
follow_relay.py; a small on_msg hook feeds link_log for telemetry diagnostics.

ALWAYS dry-run first to read the true vertical separation, then set --sep-bias:
  python follow_gps.py --target-port COM12 --chaser-port COM5 --dry-run --no-rec

Typical filming run (defaults are filming-ready: clean altitude mode on, 3 m
below, full 20 m/s until ~200 m, 10 m/s inside 100 m):
  python follow_gps.py --target-port COM12 --chaser-port COM5 --sep-bias <m> --set-mode

Ctrl-C -> chaser LOITER. The RC mode switch overrides at any instant; if the
chaser leaves GUIDED this script exits and hands control back to the pilot.

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

from pymavlink import mavutil

# vendored trajectory planner (byte-for-byte; Gazebo dep neutralised)
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

from chaser_env import ChaserEnv
from link_log import LinkLogger

EARTH_R = 6371000.0


# ---------------------------------------------------------------- geometry
def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def has_fix(s):
    """3D fix and a real coordinate (reject 0,0 'null island')."""
    if s["lat"] is None or s["fix"] < 3:
        return False
    return not (abs(s["lat"]) < 1e-6 and abs(s["lon"]) < 1e-6)


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

    on_msg(name, msg, t_mono) is called for every received message so link_log
    can record per-message health without changing the snapshot contract.
    """

    def __init__(self, name, port, baud, stream_hz=10.0, on_msg=None):
        self.name = name
        self.stream_hz = stream_hz
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
        self.last_raw = None
        self.pos_times = collections.deque(maxlen=40)
        self.lock = threading.Lock()

        self._request_streams()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _request_streams(self):
        # 33 = GLOBAL_POSITION_INT, 24 = GPS_RAW_INT
        for msg_id, hz in ((33, self.stream_hz), (24, 1)):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, int(self.stream_hz), 1)

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
                    self.pos_times.append(self.last_pos_t)
                    self.last_raw = msg.get_msgbuf()
                elif t == "GPS_RAW_INT":
                    self.fix, self.nsat = msg.fix_type, msg.satellites_visible
                elif t == "HEARTBEAT" and msg.get_srcSystem() == self.sysid:
                    self.mode = mavutil.mode_string_v10(msg)
                    self.armed = bool(msg.base_mode &
                                      mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "STATUSTEXT":
                    print(f"\n[{self.name} FC] {msg.text}")
            # link telemetry logging (outside the state lock)
            if self.on_msg is not None:
                try:
                    self.on_msg(self.name, msg, t_mono)
                except Exception:
                    pass

    def snapshot(self):
        with self.lock:
            ts = list(self.pos_times)
            hz = 0.0
            if len(ts) >= 3 and ts[-1] > ts[0]:
                hz = (len(ts) - 1) / (ts[-1] - ts[0])
            return dict(lat=self.lat, lon=self.lon, alt_amsl=self.alt_amsl,
                        alt_rel=self.alt_rel, vx=self.vx, vy=self.vy, vz=self.vz,
                        fix=self.fix, nsat=self.nsat, mode=self.mode,
                        armed=self.armed, age=time.time() - self.last_pos_t,
                        hz=hz, raw=self.last_raw)

    @staticmethod
    def home_amsl(s):
        """Chaser home AMSL = alt_amsl - alt_rel. Constant through a flight even
        as the baro drifts (both fields drift together), so it is the stable
        reference for cross-vehicle altitude math."""
        if s["alt_amsl"] is None or s["alt_rel"] is None:
            return None
        return s["alt_amsl"] - s["alt_rel"]

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
                    help="OLCULEN dikey ayrim |sep| bunun altina inerse komut kesilir "
                         "(m). 0 = KAPALI. Intercept dogal olarak yaklastigi icin "
                         "varsayilan kapali; ayni irtifada gecis icin 0 birak.")
    ap.add_argument("--max-dist", type=float, default=1000.0, help="guvenlik kelepcesi (m)")
    ap.add_argument("--max-age", type=float, default=3.0,
                    help="hedef konumu bayatlama siniri (s)")

    # --- recording (kept as flags) ---
    ap.add_argument("--rec-start", type=float, default=150.0, help="bu mesafede kayda basla (m)")
    ap.add_argument("--rec-stop", type=float, default=20.0, help="bu mesafede kaydi durdur (m)")
    ap.add_argument("--rec-host", default="192.168.144.50", help="Orin IP")
    ap.add_argument("--rec-port", type=int, default=5599)
    ap.add_argument("--no-rec", action="store_true", help="kayit tetigini kapat")

    # --- posvel (kept as flag) ---
    ap.add_argument("--posvel", action="store_true",
                    help="komuta hedefin hizini da kat (ileri besleme). Carrot "
                         "planlayicida nadiren gerekir; sabit hedefte gereksiz.")

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
    if a.min_sep and a.min_sep > 0 and abs(a.alt_offset) < a.min_sep:
        print(f"[!] UYARI: |--alt-offset|={abs(a.alt_offset):.0f} m < --min-sep={a.min_sep:.0f} m. "
              f"Dikey ayrim sigortasi amaclanan yakin geciste tetiklenir. "
              f"Ayni irtifada intercept istiyorsan --min-sep 0 ver, ya da "
              f"|--alt-offset|'i buyut.", flush=True)

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
                     on_msg=_chaser_on_msg)

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
    cfg = TrajectoryConfig(
        home_lat_deg=float(cs["lat"]),
        home_lon_deg=float(cs["lon"]),
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

    pipeline = TrajectoryPipeline(cfg)
    pipeline.soft_reset_guidance(realign=False)
    chaser_env = ChaserEnv(chaser, verbose_speed=False, cmd_latency=link_log.cmd_lat)
    chaser_env.enable_posvel(a.posvel)
    speed_slew = SpeedCommandSlew.from_config(cfg)
    pos_hold = PositionSetpointHold.from_config(cfg)
    dir_slew = CarrotDirSlew.from_config(cfg)
    catchup = CatchupSpeedLatch()

    # --- plan banner ---
    where = "ustunde" if a.alt_offset > 0 else ("altinda" if a.alt_offset < 0 else "ayni irtifada")
    print(f"\n  plan   : takipci hedefi kovalar; karsidan gelirsen ONUNU KESER (intercept), "
          f"caprazda ARKADAN takip eder (follow) - kisitsiz.")
    print(f"  hiz    : uzakta {a.max_speed:.0f} m/s, {a.deadline_range:.0f} m icinde "
          f"{a.close_approach:.0f} m/s'e iner (adaptif). WP_ACC={a.accel:.0f}.")
    print(f"  irtifa : hedefin {abs(a.alt_offset):.0f} m {where}"
          + (" (alt-match: dogrudan)" if a.alt_match else " (dosya temizlik/tavan mantigi)")
          + f"; alt sinir {a.min_alt:.0f} m")
    print(f"  komut  : {'pozisyon + hiz (posvel)' if a.posvel else 'pozisyon + yaw'}")
    if a.min_sep and a.min_sep > 0:
        print(f"  sigorta: olculen |dikey ayrim| < {a.min_sep:.0f} m ise komut kesilir")
    else:
        print(f"  sigorta: --min-sep 0 (KAPALI). Carpisma sorumlulugu pilotta - "
              f"RC'de moda hazir dur, dikey ofset ver.")
    if a.sep_bias:
        print(f"  duzeltme: ayrim okumasindan {a.sep_bias:+.1f} m cikariliyor")
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
    sent = skipped = 0
    last_out = None
    last_target_pos_t = None
    speed_seeded = False
    last_stream_req = time.monotonic()
    last_ping = time.monotonic()
    last_armed_speed = None
    last_speed = None

    try:
        while True:
            loop_t = time.time()
            now = time.monotonic()
            ts, cs = target.snapshot(), chaser.snapshot()

            # --- safety gates ---
            why, d, sep = None, 0.0, 0.0
            if ts["lat"] is None or cs["lat"] is None:
                why = "konum paketi henuz gelmedi"
            elif not has_fix(ts):
                why = f"HEDEF GPS yok (fix={ts['fix']}, sat={ts['nsat']})"
            elif not has_fix(cs):
                why = f"TAKIPCI GPS yok (fix={cs['fix']}, sat={cs['nsat']})"
            elif ts["age"] > a.max_age:
                why = f"hedef konumu {ts['age']:.1f} s bayat"
            else:
                d = haversine_m(cs["lat"], cs["lon"], ts["lat"], ts["lon"])
                sep = (ts["alt_amsl"] - cs["alt_amsl"]) - a.sep_bias
                if d > a.max_dist:
                    why = f"mesafe {d:.0f} m > kelepce {a.max_dist:.0f} m"
                elif a.min_sep and a.min_sep > 0 and abs(sep) < a.min_sep:
                    why = (f"!! DIKEY AYRIM {sep:+.1f} m ({abs(sep):.1f} < {a.min_sep:.0f}) "
                           f"- KOMUT KESILDI, KUMANDAYA GEC")

            # pilot took over?
            if not a.dry_run and cs["mode"] not in ("GUIDED",):
                print(f"\n[!] Takipci modu {cs['mode']} oldu. Kontrol kumandada, cikiliyor.")
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

            if why:
                skipped += 1
                print(f"  [BEKLE] {why}")
                link_log.note_relay(
                    t_mono=now, updated=False,
                    target_age_ms=ts["age"] * 1e3, chaser_age_ms=cs["age"] * 1e3,
                    compute_ms=None, send_ms=None, behavior="wait", follow_phase=why[:16],
                    v_cmd_mps=None, dist_m=d if d else None, sep_m=sep if sep else None,
                    gate_alt_m=None, target_hz=ts["hz"], chaser_hz=cs["hz"])
                time.sleep(max(0.0, period - (time.time() - loop_t)))
                continue

            if rec:
                evt = rec.update(d, loop_t)
                if evt:
                    print(f"  >>> {evt}")

            # --- drone state in the fixed chaser-home NED frame ---
            chome = Vehicle.home_amsl(cs)
            if chome is None:
                skipped += 1
                print("  [BEKLE] takipci home AMSL cozulemedi")
                time.sleep(max(0.0, period - (time.time() - loop_t)))
                continue
            c_ned = latlonalt_to_local(cs["lat"], cs["lon"], cs["alt_rel"],
                                       home_lat_deg=cfg.home_lat_deg,
                                       home_lon_deg=cfg.home_lon_deg,
                                       home_alt_m=cfg.home_alt_m)
            drone = DroneState(
                position=c_ned,
                velocity=NedVelocity(float(cs["vx"]), float(cs["vy"]), float(cs["vz"])),
                timestamp_s=now,
                speed_mps=math.hypot(cs["vx"], cs["vy"]),
                heading_rad=math.atan2(cs["vy"], cs["vx"]),
            )

            # --- only feed the KF on a genuinely new target fix (no duplicates) ---
            compute_ms = None
            updated = False
            if ts["last_raw"] is not None:
                pass  # (raw not needed; we build the measurement from fields)
            target_pos_t = loop_t - ts["age"]  # wall time of the newest target fix
            if last_target_pos_t is None or (target_pos_t - last_target_pos_t) > 1e-3:
                last_target_pos_t = target_pos_t
                # target altitude expressed in the chaser-home-relative frame
                gate_alt = (ts["alt_amsl"] - a.sep_bias - chome) + a.alt_offset
                spd = math.hypot(ts["vx"], ts["vy"])
                meas = RadarMeasurement(
                    timestamp_s=now,
                    latitude_deg=float(ts["lat"]),
                    longitude_deg=float(ts["lon"]),
                    altitude_up_m=float(gate_alt),
                    ground_speed_mps=spd if spd > 0.05 else None,
                    course_rad=(math.atan2(ts["vy"], ts["vx"]) if spd > 0.05 else None),
                )
                t_c0 = time.monotonic()
                last_out = pipeline.update(meas, drone)
                compute_ms = (time.monotonic() - t_c0) * 1e3
                updated = True

            # --- gate sample (chaser-relative alt) every tick; min-alt floor ---
            gate_alt = (ts["alt_amsl"] - a.sep_bias - chome) + a.alt_offset
            rng = last_out.distance_to_target_m if last_out is not None else None
            preview = traj_hold_alt_m(gate_alt, cfg, range_m=rng)
            if preview < a.min_alt:
                gate_alt += (a.min_alt - preview)  # bump so commanded alt >= floor
            gate = GateGpsSample(lat=float(ts["lat"]), lon=float(ts["lon"]),
                                 alt=float(gate_alt), recv_mono=now)

            # --- drive the chaser ---
            send_ms = None
            if last_out is not None and not a.dry_run:
                chaser_env.refresh_from_snapshot(cs)
                if a.posvel:
                    chaser_env.set_feedforward_velocity(cs["vx"], cs["vy"], cs["vz"])
                if not speed_seeded:
                    speed_slew.reset(float(drone.speed_mps))
                    speed_seeded = True
                t_s0 = time.monotonic()
                last_speed = apply_planner_output(
                    chaser_env, last_out, gate=gate, drone=drone, config=cfg,
                    last_speed_cmd=last_speed, speed_slew=speed_slew, pos_hold=pos_hold,
                    dir_slew=dir_slew, catchup=catchup, now_s=now,
                )
                send_ms = (time.monotonic() - t_s0) * 1e3
                sent += 1

            # command->response speed-reaction sampling (includes FC + accel)
            chaser_gs = math.hypot(cs["vx"], cs["vy"])
            if last_speed is not None:
                if last_armed_speed is None or abs(last_speed - last_armed_speed) >= 1.5:
                    link_log.cmd_lat.arm_speed(chaser_gs, last_speed, now)
                    last_armed_speed = last_speed
            link_log.cmd_lat.note_gs(chaser_gs, now)

            # --- status + latency log ---
            beh = last_out.behavior.value if (last_out and last_out.behavior) else "-"
            fp = last_out.follow_phase.value if last_out else "-"
            cmd_alt = traj_hold_alt_m(gate_alt, cfg, range_m=rng)
            spd = math.hypot(ts["vx"], ts["vy"])
            rstate = rec.state if rec else "-"
            vshow = last_speed if last_speed is not None else 0.0
            print(f"  d={d:6.1f} m  ayrim={sep:+5.1f} m  beh={beh:9s} fp={fp:5s}  "
                  f"v_cmd={vshow:4.1f}  komut_alt={cmd_alt:5.1f} m  hedef_hiz={spd:4.1f}  "
                  f"yas={ts['age']:.2f}s  hz={ts['hz']:.1f}  rec={rstate}  n={sent}")
            link_log.note_relay(
                t_mono=now, updated=updated,
                target_age_ms=ts["age"] * 1e3, chaser_age_ms=cs["age"] * 1e3,
                compute_ms=compute_ms, send_ms=send_ms, behavior=beh, follow_phase=fp,
                v_cmd_mps=last_speed, dist_m=d, sep_m=sep, gate_alt_m=cmd_alt,
                target_hz=ts["hz"], chaser_hz=cs["hz"])

            time.sleep(max(0.0, period - (time.time() - loop_t)))

    except KeyboardInterrupt:
        print("\n[i] Durduruldu.")

    print(f"[i] gonderim={sent} atlanan={skipped}")
    link_log.close()
    if rec:
        rec.shutdown()
    if not a.dry_run:
        print("[i] Takipci LOITER'a aliniyor...")
        chaser.set_mode("LOITER")
        time.sleep(1)


if __name__ == "__main__":
    main()
