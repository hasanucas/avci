#!/usr/bin/env python3
"""
follow_relay.py v2 — hedef drone'un konumunu takipci drone'a aktarir,
mesafeye gore Orin'deki kaydi tetikler.

  HEDEF (LOITER, sabit)  --telem-->  laptop  --telem-->  TAKIPCI
        COM12                          |                   COM5
                                       | UDP tetik
                                       v
                              ORIN (record_agent.py)
                              rtsp://192.168.144.25 -> disk

Ornek — hedefin 15 m altindan yaklas, 150 m'de kayda basla, 20 m'de durdur:

  python follow_relay.py --target-port COM12 --chaser-port COM5 \
      --below 15 --rec-start 150 --rec-stop 20 --rec-host 192.168.144.50

Once daima:
  python follow_relay.py --target-port COM12 --chaser-port COM5 \
      --below 15 --dry-run --no-rec

Ctrl-C -> takipci LOITER. Kumandada mod anahtari her an ezer.

v1'den farklar:
  --alt-offset yerine --below (pozitif = hedefin N metre ALTI)
  --standoff varsayilani 40 -> 0 (dogrudan hedefe git)
  --min-sep    OLCULEN dikey ayrim sigortasi (komut edilen degil)
  --speed      seyir hizi
  --rec-*      mesafe tetikli kayit, sadece yaklasirken
"""
import argparse
import collections
import json
import math
import socket
import sys
import threading
import time

from pymavlink import mavutil

# type_mask bitleri: 0-2 pos, 3-5 vel, 6-8 accel, 10 yaw, 11 yaw_rate (1 = yok say)
POS_ONLY_MASK = 0b110111111000   # sadece pozisyon
POSVEL_MASK   = 0b110111000000   # pozisyon + hiz (ArduCopter set_destination_posvel)
EARTH_R = 6371000.0


# ---------------------------------------------------------------- geometri
def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def dest_point(lat, lon, brg, dist_m):
    br, d = math.radians(brg), dist_m / EARTH_R
    p1, l1 = math.radians(lat), math.radians(lon)
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(br))
    l2 = l1 + math.atan2(math.sin(br) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), math.degrees(l2)


class SpeedSlew:
    """DO_CHANGE_SPEED'i basamak yerine rampayla degistirir (m/s^2)."""

    def __init__(self, rate_mps2=5.0):
        self.rate = float(rate_mps2)
        self.value = None
        self.last_t = None

    def step(self, target, now):
        if self.value is None or self.last_t is None:
            self.value, self.last_t = float(target), float(now)
            return self.value
        dt = max(0.0, float(now) - self.last_t)
        self.last_t = float(now)
        cap = self.rate * dt
        delta = float(target) - self.value
        self.value += max(-cap, min(cap, delta))
        return self.value


def speed_for_range(range_m, cruise, close, start_m, end_m):
    """Mesafeye bagli hiz programi: uzakta cruise, yaklasirken close.

    range >= start_m  -> cruise
    range <= end_m    -> close
    arada             -> dogrusal rampa
    """
    if start_m < end_m:
        start_m = end_m
    r = max(0.0, float(range_m))
    if r >= start_m:
        return float(cruise)
    if r <= end_m:
        return float(close)
    span = max(1.0, float(start_m) - float(end_m))
    a = (r - float(end_m)) / span
    return float(close) + a * (float(cruise) - float(close))


def has_fix(s):
    """3D fix var mi VE koordinat gercek mi (0,0 = 'null island')."""
    if s['lat'] is None or s['fix'] < 3:
        return False
    return not (abs(s['lat']) < 1e-6 and abs(s['lon']) < 1e-6)


# ---------------------------------------------------------------- kayit tetikleyici
class Recorder:
    """Orin'deki record_agent.py'ye UDP tetik yollar.

    Durum makinesi:
      IDLE    -> mesafe < start VE yaklasiyor  -> ACTIVE  (basla)
      ACTIVE  -> mesafe < stop                 -> LOCKOUT (dur)
              -> mesafe > start + hist         -> IDLE    (pas iptal)
      LOCKOUT -> mesafe > start + hist         -> IDLE    (yeniden kur)
    """

    def __init__(self, host, port, start_d, stop_d, hyst=30.0, window=4.0, deadband=2.0):
        self.addr = (host, port)
        self.start_d, self.stop_d = start_d, stop_d
        self.hyst, self.window, self.deadband = hyst, window, deadband
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.4)
        self.hist = collections.deque()
        self.state = 'IDLE'
        self.tag = None

    def closing(self, d, now):
        """Yaklasiyor mu? GPS gurultusune karsi deadband'li."""
        self.hist.append((now, d))
        while self.hist and now - self.hist[0][0] > self.window:
            self.hist.popleft()
        if len(self.hist) < 3:
            return False
        return (self.hist[0][1] - d) > self.deadband

    def send(self, payload):
        """Tetigi 3 kez yolla (UDP kayipli), ACK bekle."""
        raw = json.dumps(payload).encode()
        for _ in range(3):
            try:
                self.sock.sendto(raw, self.addr)
                ack, _ = self.sock.recvfrom(1024)
                return json.loads(ack.decode())
            except socket.timeout:
                continue
            except Exception as e:
                return {'ok': False, 'err': str(e)}
        return {'ok': False, 'err': 'ACK yok - Orin ulasilamiyor?'}

    def update(self, d, now):
        """Her donguden cagrilir. Durum degistiyse mesaj dondurur."""
        closing = self.closing(d, now)
        rearm_d = self.start_d + self.hyst

        if self.state == 'IDLE':
            if d < self.start_d and closing:
                self.tag = time.strftime('%Y%m%d_%H%M%S')
                r = self.send({'cmd': 'start', 'tag': self.tag, 'dist': round(d, 1)})
                if r.get('ok'):
                    self.state = 'ACTIVE'
                    return f"KAYIT BASLADI  d={d:.0f} m  dosya={r.get('file', '?')}"
                return f"[!] KAYIT BASLATILAMADI: {r.get('err')}"

        elif self.state == 'ACTIVE':
            if d < self.stop_d:
                r = self.send({'cmd': 'stop'})
                self.state = 'LOCKOUT'
                return (f"KAYIT DURDU (hedefe {d:.0f} m)  sure={r.get('secs', '?')}s  "
                        f"boyut={r.get('mb', '?')} MB")
            if d > rearm_d:
                self.send({'cmd': 'stop'})
                self.state = 'IDLE'
                return f"KAYIT IPTAL (uzaklasildi, d={d:.0f} m)"

        elif self.state == 'LOCKOUT':
            if d > rearm_d:
                self.state = 'IDLE'
                return f"kayit yeniden kuruldu (d={d:.0f} m > {rearm_d:.0f} m)"
        return None

    def shutdown(self):
        if self.state == 'ACTIVE':
            print(f"[i] Kayit kapatiliyor: {self.send({'cmd': 'stop'})}")


# ---------------------------------------------------------------- arac
class Vehicle:
    def __init__(self, name, port, baud, stream_hz=10.0):
        self.name = name
        self.stream_hz = stream_hz
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
        self.pos_times = collections.deque(maxlen=40)   # gercek gelis hizi olcumu
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
        # eski firmware yedegi
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
            with self.lock:
                if t == 'GLOBAL_POSITION_INT':
                    self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
                    self.alt_amsl = msg.alt / 1000.0
                    self.alt_rel = msg.relative_alt / 1000.0
                    self.vx = msg.vx / 100.0
                    self.vy = msg.vy / 100.0
                    self.vz = msg.vz / 100.0
                    self.last_pos_t = time.time()
                    self.pos_times.append(self.last_pos_t)
                    self.last_raw = msg.get_msgbuf()
                elif t == 'GPS_RAW_INT':
                    self.fix, self.nsat = msg.fix_type, msg.satellites_visible
                elif t == 'HEARTBEAT' and msg.get_srcSystem() == self.sysid:
                    self.mode = mavutil.mode_string_v10(msg)
                    self.armed = bool(msg.base_mode &
                                      mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == 'STATUSTEXT':
                    print(f"\n[{self.name} FC] {msg.text}")

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
        if s['alt_amsl'] is None or s['alt_rel'] is None:
            return None
        return s['alt_amsl'] - s['alt_rel']

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
    ap = argparse.ArgumentParser()
    ap.add_argument('--target-port', required=True, help='hedef drone, or: COM12')
    ap.add_argument('--chaser-port', required=True, help='takipci drone, or: COM5')
    ap.add_argument('--target-baud', type=int, default=57600)
    ap.add_argument('--chaser-baud', type=int, default=57600)
    ap.add_argument('--mode', choices=['follow', 'guided'], default='guided')
    ap.add_argument('--rate', type=float, default=4.0,
                    help='FC\'ye komut gonderme hizi (Hz)')
    ap.add_argument('--stream-hz', type=float, default=10.0,
                    help='telemetriden konum isteme hizi (Hz). Link bogulursa '
                         'yas= buyur; o zaman dusur.')

    ap.add_argument('--below', type=float, required=True,
                    help='takipci hedefin KAC METRE ALTINDA ucsun (negatif = ustunde)')
    ap.add_argument('--min-sep', type=float, default=6.0,
                    help='OLCULEN dikey ayrim bunun altina inerse komut kesilir (m)')
    ap.add_argument('--sep-bias', type=float, default=0.0,
                    help='iki drone GERCEKTEN ayni seviyedeyken okunan ayrim degeri. '
                         'Once --dry-run ile olc, sonra buraya gir.')
    ap.add_argument('--standoff', type=float, default=0.0,
                    help='hedefin bu kadar gerisinde dur (0 = dogrudan hedefe)')
    ap.add_argument('--lead', type=float, default=0.0, help='hedefi hiz vektoruyle ileri sar (s)')
    ap.add_argument('--posvel', action='store_true',
                    help='hedefin hizini da komuta kat (ileri besleme). Hareketli '
                         'hedefte gecikmeyi azaltir, sabit hedefte gereksiz.')
    ap.add_argument('--min-delta', type=float, default=1.0,   # olu bant: 30 = referans
                    help='komut noktasi bu kadar kaymadikca yeniden gonderme (m). '
                         'Sabit hedefte yorunge planlayicisini bosuna tetiklemez.')
    ap.add_argument('--resend-s', type=float, default=3.0,
                    help='kaymasa bile bu araliklarla guvenlik tekrari (s)')
    ap.add_argument('--speed', type=float, default=None, help='sabit seyir hizi (m/s)')

    # --- opsiyonel iyilestirmeler, HEPSI VARSAYILAN KAPALI ---
    ap.add_argument('--speed-schedule', action='store_true',
                    help='mesafeye bagli hiz programi. ACIKKEN --speed yok sayilir '
                         've hiz surekli degisir, sabit hiz karsilastirmasini bozar.')
    ap.add_argument('--cruise-speed', type=float, default=20.0, help='program: uzak hiz')
    ap.add_argument('--close-speed', type=float, default=10.0, help='program: yakin hiz')
    ap.add_argument('--decel-start', type=float, default=300.0,
                    help='program: bu mesafeden itibaren yavaslamaya basla (m)')
    ap.add_argument('--decel-end', type=float, default=100.0,
                    help='program: bu mesafede close-speed\'e ulas (m)')
    ap.add_argument('--slew', type=float, default=5.0,
                    help='program: hiz degisim rampasi (m/s^2)')

    ap.add_argument('--max-dist', type=float, default=1000.0, help='guvenlik kelepcesi (m)')
    ap.add_argument('--max-age', type=float, default=3.0, help='hedef konumu bayatlama siniri (s)')
    ap.add_argument('--min-alt', type=float, default=10.0, help='komut irtifasi alt siniri (m)')

    ap.add_argument('--rec-start', type=float, default=150.0, help='bu mesafede kayda basla (m)')
    ap.add_argument('--rec-stop', type=float, default=20.0, help='bu mesafede kaydi durdur (m)')
    ap.add_argument('--rec-host', default='192.168.144.50', help='Orin IP')
    ap.add_argument('--rec-port', type=int, default=5599)
    ap.add_argument('--no-rec', action='store_true', help='kayit tetigini tamamen kapat')

    ap.add_argument('--set-mode', action='store_true', help='modu script degistirsin')
    ap.add_argument('--dry-run', action='store_true', help="FC'ye komut gonderme")
    a = ap.parse_args()

    if abs(a.below) < 5.0:
        sys.exit("[X] --below 5 m'den kucuk olamaz. Iki EKF'nin irtifa kestirimi "
                 "birkac metre saparsa dikey ayrim kalmaz. 10-15 m onerilir.")
    if a.rec_stop >= a.rec_start:
        sys.exit("[X] --rec-stop, --rec-start'tan kucuk olmali.")

    target = Vehicle("HEDEF  ", a.target_port, a.target_baud, a.stream_hz)
    chaser = Vehicle("TAKIPCI", a.chaser_port, a.chaser_baud, a.stream_hz)

    if target.sysid == chaser.sysid:
        print(f"\n[!] IKI DRONE DE sysid={target.sysid}.")
        if a.mode == 'follow':
            sys.exit("[X] follow modu ayni sysid ile calismaz. Hedefte SYSID_THISMAV=2 yap.")
        print("[!] guided'da calisir ama karisikliga acik.")

    rec = None
    if not a.no_rec:
        rec = Recorder(a.rec_host, a.rec_port, a.rec_start, a.rec_stop)
        print(f"\n[i] Kayit ajani yoklaniyor ({a.rec_host}:{a.rec_port})...")
        r = rec.send({'cmd': 'ping'})
        if r.get('ok'):
            print(f"[i] Orin ajani hazir: {r.get('msg', '')}")
        else:
            print(f"[!] Orin ajani cevap vermiyor: {r.get('err')}")
            print("[!] Laptop SIYI agina bagli mi? record_agent.py Orin'de kosuyor mu?")
            if input("Kayitsiz devam edilsin mi? (evet) > ").strip().lower() != 'evet':
                sys.exit("[i] Iptal.")
            rec = None

    print("\ndurum okunuyor...")
    time.sleep(3)
    ts, cs = target.snapshot(), chaser.snapshot()
    if ts['lat'] is None or cs['lat'] is None:
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
        print("  [!] GUIDED/FOLLOW iki tarafta da GPS ister.")

    slew = SpeedSlew(a.slew) if a.speed_schedule else None
    last_speed_sent = None
    last_speed_sent = None

    yer = 'dogrudan hedefe' if a.standoff == 0 else f'{a.standoff:.0f} m gerisine'
    print(f"\n  plan   : takipci, hedefin {a.below:.0f} m ALTINDAN {yer} gidecek")
    print(f"  komut  : {'pozisyon + hiz (ileri besleme)' if a.posvel else 'sadece pozisyon'}"
          f", {a.min_delta:.1f} m'den az kaymada tekrar yok")
    print(f"  sigorta: olculen ayrim < {a.min_sep:.0f} m ise komut kesilir")
    if a.sep_bias:
        print(f"  duzeltme: ayrim okumasindan {a.sep_bias:+.1f} m cikariliyor")
    else:
        print("  duzeltme: --sep-bias 0. Ikisi ayni seviyedeyken 'ayrim' sifir "
              "okumuyorsa o degeri --sep-bias ile ver.")
    if rec:
        print(f"  kayit  : {a.rec_start:.0f} m'de basla, {a.rec_stop:.0f} m'de dur, "
              f"sadece yaklasirken")
    if a.speed_schedule:
        print(f"  hiz prog: {a.decel_start:.0f} m'ye kadar {a.cruise_speed:.0f} m/s, "
              f"{a.decel_end:.0f} m'de {a.close_speed:.0f} m/s  (--speed yok sayildi)")

    if a.dry_run:
        print("\n--dry-run: FC'ye komut gonderilmiyor.\n")
    else:
        print("\nOnay icin 'evet' yaz:")
        if input("> ").strip().lower() != 'evet':
            sys.exit("[i] Iptal.")
        want = 'FOLLOW' if a.mode == 'follow' else 'GUIDED'
        if a.set_mode:
            if not chaser.set_mode(want):
                sys.exit(f"[X] Takipci {want} moduna gecmedi.")
        else:
            print(f"[i] Takipciyi kumandadan {want} moduna al. Bekleniyor...")
            while chaser.mode != want:
                time.sleep(0.3)
            print(f"[i] Takipci {want} modunda.")
        if a.speed and not a.speed_schedule:
            chaser.conn.mav.command_long_send(
                chaser.conn.target_system, chaser.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, a.speed, -1, 0, 0, 0, 0)
            print(f"[i] Seyir hizi {a.speed} m/s gonderildi.")

    print("Ctrl-C -> takipci LOITER. Kumanda her an ezer.\n")
    period = 1.0 / a.rate
    sent = skipped = 0
    last_pt = None          # (lat, lon, rel, t) en son GONDERILEN komut

    try:
        while True:
            loop_t = time.time()
            ts, cs = target.snapshot(), chaser.snapshot()

            why, d, sep = None, 0.0, 0.0
            if ts['lat'] is None or cs['lat'] is None:
                why = "konum paketi henuz gelmedi"
            elif not has_fix(ts):
                why = f"HEDEF GPS yok (fix={ts['fix']}, sat={ts['nsat']})"
            elif not has_fix(cs):
                why = f"TAKIPCI GPS yok (fix={cs['fix']}, sat={cs['nsat']})"
            elif ts['age'] > a.max_age:
                why = f"hedef konumu {ts['age']:.1f} s bayat"
            else:
                d = haversine_m(cs['lat'], cs['lon'], ts['lat'], ts['lon'])
                # ham fark eksi olculen sistematik sapma = gercek ayrim
                sep = (ts['alt_amsl'] - cs['alt_amsl']) - a.sep_bias
                if d > a.max_dist:
                    why = f"mesafe {d:.0f} m > kelepce {a.max_dist:.0f} m"
                elif a.below > 0 and sep < a.min_sep:
                    why = (f"!! DIKEY AYRIM {sep:+.1f} m < {a.min_sep:.0f} m "
                           f"- KOMUT KESILDI, KUMANDAYA GEC")

            if not a.dry_run and cs['mode'] not in ('GUIDED', 'FOLLOW'):
                print(f"\n[!] Takipci modu {cs['mode']} oldu. Kontrol kumandada, cikiliyor.")
                break

            if why:
                skipped += 1
                print(f"  [BEKLE] {why}")
                time.sleep(max(0.0, period - (time.time() - loop_t)))
                continue

            if rec:
                evt = rec.update(d, loop_t)
                if evt:
                    print(f"  >>> {evt}")

            spd = math.hypot(ts['vx'], ts['vy'])
            rstate = rec.state if rec else '-'

            if a.mode == 'follow':
                if not a.dry_run and ts['raw']:
                    chaser.conn.write(ts['raw'])
                    sent += 1
                print(f"  d={d:6.1f} m  ayrim={sep:+5.1f} m  hedef_hiz={spd:4.1f}  "
                      f"yas={ts['age']:.2f}s  hz={ts['hz']:.1f}  rec={rstate}  n={sent}")
            else:
                tlat, tlon = ts['lat'], ts['lon']
                if a.lead > 0:
                    tlat += (ts['vx'] * a.lead) / 111320.0
                    tlon += (ts['vy'] * a.lead) / (111320.0 * math.cos(math.radians(tlat)))
                if a.standoff > 0:
                    brg = bearing_deg(tlat, tlon, cs['lat'], cs['lon'])
                    glat, glon = dest_point(tlat, tlon, brg, a.standoff)
                else:
                    glat, glon = tlat, tlon

                chome = Vehicle.home_amsl(cs)
                if chome is None:
                    skipped += 1
                    print("  [BEKLE] takipci home AMSL cozulemedi")
                    time.sleep(max(0.0, period - (time.time() - loop_t)))
                    continue
                goal_rel = (ts['alt_amsl'] - a.sep_bias - a.below) - chome
                if goal_rel < a.min_alt:
                    skipped += 1
                    print(f"  [BEKLE] komut irtifasi {goal_rel:.1f} m < {a.min_alt:.0f} m")
                    time.sleep(max(0.0, period - (time.time() - loop_t)))
                    continue

                # Ayni noktayi tekrar tekrar gondermek WPNav yorungesini her
                # seferinde sifirlar; arac tam seyir hizina cikamaz. Sadece
                # kaydiysa ya da guvenlik tekrari zamani geldiyse gonder.
                # --- mesafeye bagli hiz programi (opsiyonel) ---
                cmd_speed = None
                if a.speed_schedule and not a.dry_run:
                    want = speed_for_range(d, a.cruise_speed, a.close_speed,
                                           a.decel_start, a.decel_end)
                    cmd_speed = slew.step(want, loop_t)
                    if last_speed_sent is None or abs(cmd_speed - last_speed_sent) >= 0.45:
                        chaser.conn.mav.command_long_send(
                            chaser.conn.target_system, chaser.conn.target_component,
                            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                            1, cmd_speed, -1, 0, 0, 0, 0)
                        last_speed_sent = cmd_speed

                due = True
                if last_pt is not None:
                    moved = haversine_m(last_pt[0], last_pt[1], glat, glon)
                    moved = math.hypot(moved, goal_rel - last_pt[2])
                    due = (moved > a.min_delta or
                           (loop_t - last_pt[3]) > a.resend_s)

                if not a.dry_run and due:
                    if a.posvel:
                        mask, vx, vy, vz = POSVEL_MASK, ts['vx'], ts['vy'], ts['vz']
                    else:
                        mask, vx, vy, vz = POS_ONLY_MASK, 0.0, 0.0, 0.0
                    chaser.conn.mav.set_position_target_global_int_send(
                        0, chaser.conn.target_system, chaser.conn.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        mask,
                        int(glat * 1e7), int(glon * 1e7), float(goal_rel),
                        vx, vy, vz, 0, 0, 0, 0, 0)
                    sent += 1
                    last_pt = (glat, glon, goal_rel, loop_t)

                extra = f"  v_cmd={cmd_speed:4.1f}" if cmd_speed is not None else ""
                print(f"  d={d:6.1f} m  ayrim={sep:+5.1f} m  komut_rel={goal_rel:5.1f} m  "
                      f"hedef_hiz={spd:4.1f}  yas={ts['age']:.2f}s  hz={ts['hz']:.1f}  "
                      f"rec={rstate}  n={sent}{extra}")

            time.sleep(max(0.0, period - (time.time() - loop_t)))

    except KeyboardInterrupt:
        print("\n[i] Durduruldu.")

    print(f"[i] gonderim={sent} atlanan={skipped}")
    if rec:
        rec.shutdown()
    if not a.dry_run:
        print("[i] Takipci LOITER'a aliniyor...")
        chaser.set_mode('LOITER')
        time.sleep(1)


if __name__ == '__main__':
    main()
