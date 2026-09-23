#!/usr/bin/env python3
"""
record_agent.py v3 — ORIN'de arka planda kosar.

  * Goruntuyu HAM kaydeder (stream copy, 30 Hz, EK KAYIP SIFIR, ~0 CPU)
  * Konumlari ayri CSV'ye yazar (1-2 Hz, zaman damgali)
  * Video saati ile duvar saati arasindaki kaymayi olcup yan dosyaya koyar

  Overlay UCUSTA YAPILMAZ. Indikten sonra:
      python3 render_overlay.py ~/recordings/chase_<tag>

MESAFEYI NEREDEN BILIYOR
  Kendi konumu : FC -> GLOBAL_POSITION_INT
  Hedefin yeri : FC -> POSITION_TARGET_GLOBAL_INT  (FC'nin "suraya gidiyorum"
                 raporu). follow_relay --standoff 0 ile bu nokta hedef
                 drone'un tam altidir, yani yatay mesafe dogrudan cikar.
  Laptop'a / SIYI agina / UDP tetige gerek yok.

ONCE BUNU DOGRULA (masada, takipci GUIDED'da ve bir hedefe gidiyorken):
      python3 record_agent.py --probe

KULLANIM
      python3 record_agent.py --rec-start 150 --rec-stop 20
      python3 record_agent.py --always          # esik bekleme, sureki kaydet
      python3 record_agent.py --fc udpin:127.0.0.1:14551   # MAVProxy kosuyorsa

CIKTILAR (~/recordings/chase_<tag>.*)
      _raw.mkv   ham video, kameranin byte'lari
      .csv       t_wall, t_video, kendi konum, hedef konum, mesafe
      .json      senkron bilgisi (render icin)

CODEC / COZUNURLUK
      Kaynak ne verirse o kaydedilir (stream copy). SIYI kamerasi 'main.264'
      adresinde H.265/HEVC ve 720p verebiliyor -- adres adi yaniltici.
      Ne kaydettigini once dogrula:
          ffprobe -rtsp_transport tcp rtsp://192.168.144.25:8554/main.264
      Cozunurluk/codec kamera ayaridir; SIYI FPV uygulamasindan degistirilir.
"""
import argparse
import collections
import csv
import json
import math
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

EARTH_R = 6371000.0
MSG_GLOBAL_POSITION_INT = 33
MSG_POSITION_TARGET_GLOBAL_INT = 87


_IS_TTY = sys.stdout.isatty()
_last_status = [0.0]


def status(text, every=10.0):
    """Ekranda tek satiri gunceller; log dosyasina yazarken (HDMI sokulu,
    nohup ile arka planda) her 'every' saniyede bir zaman damgali satir atar.
    Yoksa '\r' karakterleri log'u devasa tek satir yapiyor."""
    if _IS_TTY:
        print("\r" + text, end='', flush=True)
    else:
        now = time.time()
        if now - _last_status[0] >= every:
            _last_status[0] = now
            print(time.strftime('%H:%M:%S ') + text.strip(), flush=True)


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


# ------------------------------------------------------------------ FC
class FCLink:
    def __init__(self, device, baud, stream_hz=10.0):
        self.stream_hz = stream_hz
        print(f"[FC] {device} baglaniliyor...")
        self.conn = mavutil.mavlink_connection(device, baud=baud, source_system=251)
        if self.conn.wait_heartbeat(timeout=15) is None:
            sys.exit("[X] FC heartbeat yok. --fc dogru mu? MAVProxy portu tutuyor olabilir.")
        print(f"[FC] BAGLI sysid={self.conn.target_system}")
        self.lat = self.lon = self.alt_amsl = self.alt_rel = None
        self.tgt_lat = self.tgt_lon = self.tgt_alt = None
        self.fix = 0
        self.mode = "?"
        self.pos_t = self.tgt_t = 0.0
        self.pos_times = collections.deque(maxlen=40)
        self.tgt_times = collections.deque(maxlen=40)
        self.lock = threading.Lock()
        self._request()
        threading.Thread(target=self._loop, daemon=True).start()

    def _request(self):
        # MAVProxy --streamrate=-1 ile kosuyorsa akislari kimse istemez
        for mid in (MSG_GLOBAL_POSITION_INT, MSG_POSITION_TARGET_GLOBAL_INT):
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mid, int(1e6 / self.stream_hz), 0, 0, 0, 0, 0)
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, int(self.stream_hz), 1)

    def _loop(self):
        while True:
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1)
            except Exception as e:
                print(f"\n[FC] link hatasi: {e}")
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
                    self.pos_t = time.time()
                    self.pos_times.append(self.pos_t)
                elif t == 'POSITION_TARGET_GLOBAL_INT':
                    self.tgt_lat, self.tgt_lon = msg.lat_int / 1e7, msg.lon_int / 1e7
                    self.tgt_alt = msg.alt
                    self.tgt_t = time.time()
                    self.tgt_times.append(self.tgt_t)
                elif t == 'GPS_RAW_INT':
                    self.fix = msg.fix_type
                elif t == 'HEARTBEAT' and msg.get_srcSystem() == self.conn.target_system:
                    self.mode = mavutil.mode_string_v10(msg)

    @staticmethod
    def _hz(ts):
        return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) >= 3 and ts[-1] > ts[0] else 0.0

    def snap(self):
        with self.lock:
            now = time.time()
            return dict(lat=self.lat, lon=self.lon, alt_amsl=self.alt_amsl,
                        alt_rel=self.alt_rel, fix=self.fix, mode=self.mode,
                        tgt_lat=self.tgt_lat, tgt_lon=self.tgt_lon, tgt_alt=self.tgt_alt,
                        pos_age=now - self.pos_t if self.pos_t else 999.0,
                        tgt_age=now - self.tgt_t if self.tgt_t else 999.0,
                        pos_hz=self._hz(list(self.pos_times)),
                        tgt_hz=self._hz(list(self.tgt_times)))


# ------------------------------------------------------------------ kayit
class Recorder:
    """Ham video + CSV + senkron yan dosyasi."""

    def __init__(self, a, backend):
        self.a = a
        self.backend = backend
        self.proc = None
        self.base = self.raw = None
        self.t0_wall = None
        self.csvf = self.csvw = None
        self.offsets = collections.deque(maxlen=40)   # duvar - video saati
        os.makedirs(a.outdir, exist_ok=True)

    def _progress_reader(self):
        """ffmpeg -progress ciktisindan video saatini okuyup duvar saatiyle esler."""
        pat = re.compile(rb'out_time_us=(\d+)')
        for line in self.proc.stdout:
            m = pat.search(line)
            if m:
                vid = int(m.group(1)) / 1e6
                if vid > 0:
                    self.offsets.append(time.time() - vid)

    def start(self, tag, d0):
        if self.running():
            return
        self.base = os.path.join(self.a.outdir, f'chase_{tag}')
        self.raw = self.base + '_raw.mkv'

        if self.backend == 'ffmpeg':
            # -progress: video saati <-> duvar saati eslemesi icin (hassas senkron)
            cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'warning',
                   '-rtsp_transport', 'tcp', '-i', self.a.rtsp,
                   '-map', '0:v', '-c', 'copy', '-f', 'matroska', '-y', self.raw,
                   '-progress', 'pipe:1', '-nostats']
        else:
            # parsebin codec'i kendi bulur (H.264 / H.265 farketmez) ve yalnizca
            # ayristirir -- cozme/kodlama YOK, ffmpeg '-c copy' ile ayni.
            # SIYI 'main.264' adresinde HEVC verebiliyor, o yuzden sabit
            # rtph264depay kullanmiyoruz.
            # -e sart: Ctrl-C'de EOS yollar, MKV finalize olur.
            cmd = ['gst-launch-1.0', '-e', '-q',
                   'rtspsrc', f'location={self.a.rtsp}', 'protocols=tcp',
                   'latency=200', '!', 'parsebin', '!',
                   'matroskamux', '!', 'filesink', f'location={self.raw}']
        try:
            self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                         stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE)
        except FileNotFoundError:
            print(f"[!] {cmd[0]} bulunamadi. ffmpeg icin: sudo apt install ffmpeg")
            self.proc = None
            return

        if self.backend == 'ffmpeg':
            threading.Thread(target=self._progress_reader, daemon=True).start()

        deadline = time.time() + self.a.open_timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                err = self.proc.stderr.read().decode(errors='replace')[-400:]
                print(f"[!] ffmpeg cikti:\n{err.strip()}")
                self.proc = None
                return
            if os.path.exists(self.raw) and os.path.getsize(self.raw) > 8192:
                break
            time.sleep(0.2)
        else:
            print(f"[!] {self.a.open_timeout:.0f}s icinde veri gelmedi ({self.a.rtsp})")
            self.stop()
            return

        self.t0_wall = time.time()
        self.csvf = open(self.base + '.csv', 'w', newline='')
        self.csvw = csv.writer(self.csvf)
        self.csvw.writerow(['t_wall', 'own_lat', 'own_lon', 'own_alt_amsl',
                            'own_alt_rel', 'tgt_lat', 'tgt_lon', 'tgt_alt',
                            'dist_m', 'mode'])
        print(f"\n[REC] BASLADI  d={d0:.0f} m  -> {os.path.basename(self.raw)}")

    def log(self, d, s):
        if self.csvw:
            self.csvw.writerow([f"{time.time():.3f}", s['lat'], s['lon'],
                                s['alt_amsl'], s['alt_rel'], s['tgt_lat'],
                                s['tgt_lon'], s['tgt_alt'], round(d, 2), s['mode']])
            self.csvf.flush()

    def stop(self):
        if not self.proc:
            return
        # senkron: video saati = duvar saati - offset. Medyan, tek tuk sicramaya dayanikli.
        # ffmpeg: -progress ornekleri (hassas). gst: dosya buyume ani (kaba).
        if self.offsets:
            offset = statistics.median(self.offsets)
            method = 'ffmpeg_progress'
        elif self.t0_wall is not None:
            offset = self.t0_wall
            method = 'file_growth'
        else:
            offset, method = None, 'none' 
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)   # ffmpeg dosyayi duzgun kapatir
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
        if self.csvf:
            self.csvf.close()
            self.csvf = self.csvw = None

        if self.base:
            meta = {'raw': os.path.basename(self.raw),
                    'csv': os.path.basename(self.base + '.csv'),
                    'wall_at_video_zero': offset,
                    'sync_samples': len(self.offsets),
                    'sync_method': method,
                    'backend': self.backend,
                    'rtsp': self.a.rtsp}
            with open(self.base + '.json', 'w') as f:
                json.dump(meta, f, indent=2)
            mb = round(os.path.getsize(self.raw) / 1e6, 1) if os.path.exists(self.raw) else 0
            if method == 'ffmpeg_progress':
                sync = f"{len(self.offsets)} ornek (hassas)"
            elif method == 'file_growth':
                sync = "dosya buyumesi (kaba, ~0.3 s)"
            else:
                sync = "YOK"
            print(f"[REC] DURDU  {mb} MB  senkron={sync}  -> {os.path.basename(self.base)}.*")

    def running(self):
        return self.proc is not None and self.proc.poll() is None


# ------------------------------------------------------------------ probe
def probe(fc, secs=15):
    print("\n=== PROBE ===")
    print("Takipci GUIDED'da olmali ve bir hedefe gidiyor olmali,")
    print("yoksa FC POSITION_TARGET_GLOBAL_INT yayinlamaz.\n")
    for i in range(secs):
        time.sleep(1)
        s = fc.snap()
        line = (f"  {i+1:2d}s  mod={s['mode']:<10} "
                f"kendi_konum={'VAR' if s['pos_age'] < 2 else 'yok'} "
                f"({s['pos_hz']:4.1f} Hz)  "
                f"hedef_raporu={'VAR' if s['tgt_age'] < 2 else 'yok'} "
                f"({s['tgt_hz']:4.1f} Hz)")
        if s['tgt_age'] < 2 and s['lat'] is not None:
            line += f"  -> {haversine_m(s['lat'], s['lon'], s['tgt_lat'], s['tgt_lon']):.1f} m"
        print(line)
    print()
    if fc.snap()['tgt_age'] < 2:
        print("[OK] POSITION_TARGET_GLOBAL_INT akiyor. Ajan bagimsiz calisabilir.")
    else:
        print("[!] Gelmedi. Takipci GUIDED'da ve hedef gonderilmis miydi?")
        print("    Degilse laptop tetikli surume donmek gerekir.")


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fc', default='/dev/ttyACM0',
                    help='FC baglantisi; MAVProxy kosuyorsa udpin:127.0.0.1:14551')
    ap.add_argument('--fc-baud', type=int, default=115200)
    ap.add_argument('--rtsp', default='rtsp://192.168.144.25:8554/main.264')
    ap.add_argument('--outdir', default=os.path.expanduser('~/recordings'))

    ap.add_argument('--rec-start', type=float, default=150.0, help='bu mesafede basla (m)')
    ap.add_argument('--rec-stop', type=float, default=20.0, help='bu mesafede dur (m)')
    ap.add_argument('--hyst', type=float, default=30.0, help='yeniden kurulma marji (m)')
    ap.add_argument('--deadband', type=float, default=2.0, help='yaklasma esigi (m)')
    ap.add_argument('--window', type=float, default=4.0, help='yaklasma penceresi (s)')
    ap.add_argument('--always', action='store_true',
                    help='esik bekleme, baslar baslamaz surekli kaydet')
    ap.add_argument('--max-secs', type=float, default=900.0, help='azami kayit suresi')
    ap.add_argument('--log-rate', type=float, default=5.0,
                    help='CSV yazma / mesafe hesaplama hizi (Hz)')
    ap.add_argument('--stream-hz', type=float, default=10.0,
                    help='FC\'den konum isteme hizi (Hz). USB linki genis, 10 rahat.')
    ap.add_argument('--open-timeout', type=float, default=6.0)
    ap.add_argument('--backend', choices=['auto', 'ffmpeg', 'gst'], default='auto',
                    help='kayit motoru. auto = ffmpeg varsa onu, yoksa GStreamer. '
                         'ffmpeg senkronu daha hassas.')

    ap.add_argument('--probe', action='store_true',
                    help='sadece FC mesajlarini dinle, olcup cik (masa teshisi)')
    ap.add_argument('--probe-secs', type=float, default=15.0,
                    help='--probe kac saniye dinlesin')
    ap.add_argument('--status-every', type=float, default=10.0,
                    help='ekran yerine log dosyasina yazarken durum satiri araligi (s)')
    ap.add_argument('--dry-run', action='store_true', help='ffmpeg calistirma, tetigi goster')
    a = ap.parse_args()

    if a.rec_stop >= a.rec_start:
        sys.exit("[X] --rec-stop, --rec-start'tan kucuk olmali.")

    fc = FCLink(a.fc, a.fc_baud, a.stream_hz)
    if a.probe:
        probe(fc, a.probe_secs)
        return

    backend = a.backend
    if backend == 'auto':
        backend = 'ffmpeg' if shutil.which('ffmpeg') else 'gst'
    if not shutil.which({'ffmpeg': 'ffmpeg', 'gst': 'gst-launch-1.0'}[backend]):
        sys.exit(f"[X] {backend} bulunamadi. ffmpeg icin: sudo apt install ffmpeg")
    print(f"[i] Kayit motoru: {backend}"
          + ("" if backend == 'ffmpeg' else "  (senkron kaba, ~0.3 s)"))

    rec = Recorder(a, backend)
    state = 'IDLE'
    hist = collections.deque()
    rearm = a.rec_start + a.hyst
    period = 1.0 / a.log_rate

    if a.always:
        print("\n[i] --always: esik beklenmiyor, konum gelir gelmez kayda basliyor")
    else:
        print(f"\n[i] Esikler: basla<{a.rec_start:.0f} m  dur<{a.rec_stop:.0f} m  "
              f"yeniden kur>{rearm:.0f} m")
    print("[i] Sure siniri YOK - hedef raporu gelene kadar bekler. Drone'u "
          "yerlestirip laptop'tan baslatmak icin acele etmene gerek yok.")
    print(f"[i] Cikti: {a.outdir}")
    print(f"[i] Akis istegi {a.stream_hz:.0f} Hz   CSV {a.log_rate:.0f} Hz   "
          f"video 30 Hz ham")
    print("[i] Ctrl-C ile cik.\n")

    try:
        while True:
            loop_t = time.time()
            s = fc.snap()
            have_own = s['lat'] is not None and s['pos_age'] < 3
            have_tgt = s['tgt_lat'] is not None and s['tgt_age'] < 3

            if a.always:
                if have_own and not rec.running() and not a.dry_run:
                    rec.start(time.strftime('%Y%m%d_%H%M%S'), 0)
                d = (haversine_m(s['lat'], s['lon'], s['tgt_lat'], s['tgt_lon'])
                     if (have_own and have_tgt) else float('nan'))
                if rec.running():
                    rec.log(d, s)
                status(f"  d={d:7.1f} m  alt_rel={s['alt_rel'] or 0:6.1f}  "
                       f"hz={s['pos_hz']:4.1f}/{s['tgt_hz']:4.1f}  "
                       f"mod={s['mode']:<9} {'REC' if rec.running() else '   '}",
                       a.status_every)
                time.sleep(max(0.0, period - (time.time() - loop_t)))
                continue

            if not (have_own and have_tgt):
                if rec.running():
                    print("\n[!] Konum/hedef bayat - kayit durduruluyor")
                    rec.stop()
                    state = 'IDLE'
                status(f"  [BEKLE] kendi={'ok' if have_own else 'yok'} "
                       f"hedef={'ok' if have_tgt else 'yok'}  mod={s['mode']:<10}",
                       a.status_every)
                time.sleep(period)
                continue

            d = haversine_m(s['lat'], s['lon'], s['tgt_lat'], s['tgt_lon'])
            hist.append((loop_t, d))
            while hist and loop_t - hist[0][0] > a.window:
                hist.popleft()
            closing = len(hist) >= 3 and (hist[0][1] - d) > a.deadband

            if state == 'IDLE':
                if d < a.rec_start and closing:
                    if a.dry_run:
                        print(f"\n[DRY] KAYIT BASLARDI  d={d:.0f} m")
                        state = 'ACTIVE'
                    else:
                        rec.start(time.strftime('%Y%m%d_%H%M%S'), d)
                        state = 'ACTIVE' if rec.running() else 'IDLE'

            elif state == 'ACTIVE':
                if not a.dry_run:
                    rec.log(d, s)
                if d < a.rec_stop:
                    print(f"\n[i] Hedefe {d:.0f} m - esik asildi")
                    if not a.dry_run:
                        rec.stop()
                    state = 'LOCKOUT'
                elif d > rearm:
                    print(f"\n[i] Uzaklasildi ({d:.0f} m) - pas iptal")
                    if not a.dry_run:
                        rec.stop()
                    state = 'IDLE'
                elif rec.t0_wall and time.time() - rec.t0_wall > a.max_secs:
                    print(f"\n[!] {a.max_secs:.0f}s siniri - otomatik durduruldu")
                    rec.stop()
                    state = 'LOCKOUT'

            elif state == 'LOCKOUT':
                if d > rearm:
                    print(f"\n[i] Yeniden kuruldu (d={d:.0f} m)")
                    state = 'IDLE'

            status(f"  d={d:7.1f} m  alt_rel={s['alt_rel']:6.1f}  "
                   f"hz={s['pos_hz']:4.1f}/{s['tgt_hz']:4.1f}  mod={s['mode']:<9} "
                   f"{state:<8} {'REC' if rec.running() else '   '}", a.status_every)
            time.sleep(max(0.0, period - (time.time() - loop_t)))

    except KeyboardInterrupt:
        print("\n[i] Cikiliyor...")
        rec.stop()


if __name__ == '__main__':
    main()
