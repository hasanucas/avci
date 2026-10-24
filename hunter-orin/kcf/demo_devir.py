#!/usr/bin/env python3
"""
demo_devir.py — follow devrini MASADA, donanımsız gösterir (8 Ekim).

GERÇEK FC'ye, MAVProxy'ye ve runtracker'a DOKUNMAZ. Kendi sahte FC'sini
(UDP 127.0.0.1:14561) ve sahte FRAME sunucusunu (TCP 127.0.0.1:9998) kurar,
controller_guided.py'yi SADECE onlara bağlar (--fc / --cam-port). Gerçek sistem
14550-14552 ve 9999 kullanır: çakışmaz, MAVProxy ya da runtracker açıkken de çalışır.

Sahte FC GUIDED'da başlar (follow_gps uçuruyormuş gibi), DO_SET_MODE'a uyar ve
gelen SET_ATTITUDE_TARGET'leri moda göre sayar. FRAME sunucusu sürekli geçerli
kilit yayınlar (pilot imleçle kilitlemiş gibi).

Senaryolar
  A) varsayılan (devir KAPALI)
     t=2.0   CH6 HIGH (GUIDED, kilit var)        -> FC'ye mod komutu YOK
     t=4.5   pilot LOITER'a aldı, CH6 hâlâ HIGH  -> mod komutu YOK
     t=7.0   CH6 LOW, t=8.0 CH6 HIGH             -> GUIDED_NOGPS (balon avı akışı)
     t=10.5  CH6 LOW                             -> LOITER (çıkış modu)
  B) --follow-handover
     t=2.0   CH6 HIGH (GUIDED, kilit var)        -> GUIDED_NOGPS (devir)
     t=4.5   CH6 LOW                             -> LOITER

Kullanım
  python3 demo_devir.py          # A ve B, ~30 s
  python3 demo_devir.py A        # tek senaryo (A ya da B)
  python3 demo_devir.py -v       # controller çıktısını da göster
Çıkış kodu: 0 = beklenen davranış, 1 = sapma.
"""
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("HATA: pymavlink yok. pip3 install --user pymavlink")
    sys.exit(1)

HERE = os.path.dirname(os.path.abspath(__file__))
FC_PORT = 14561          # gerçek sistemde KULLANILMAYAN port
FRAME_PORT = 9998        # runtracker 9999 kullanır
MODE_NAME = {0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 4: 'GUIDED', 5: 'LOITER',
             20: 'GUIDED_NOGPS'}
VERBOSE = '-v' in sys.argv

SCENARIOS = {
    'A': dict(
        title="A) VARSAYILAN — devir KAPALI (follow_gps uçarken sadece takip + log)",
        flags=[],
        plan=[(2.0, 'ch6', 1950), (4.5, 'mode', 5), (7.0, 'ch6', 1050),
              (8.0, 'ch6', 1950), (10.5, 'ch6', 1050)],
        dur=12.5),
    'B': dict(
        title="B) --follow-handover — kilit + CH6 HIGH = GUIDED_NOGPS devri",
        flags=['--follow-handover'],
        plan=[(2.0, 'ch6', 1950), (4.5, 'ch6', 1050)],
        dur=6.5),
}


def frame_server(srv, stop):
    """Pilot imleçle kilitlemiş gibi: sürekli geçerli 'tracking' FRAME'i."""
    srv.settimeout(0.5)
    conn = None
    while not stop.is_set() and conn is None:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
    fid = 0
    while conn is not None and not stop.is_set():
        try:
            conn.sendall(f"FRAME {fid} 1 0.62000 0.45000 0.05000 0.05000 "
                         f"9.60000 -2.52670 tracking\n".encode())
        except OSError:
            break
        fid += 1
        time.sleep(1 / 30)
    if conn is not None:
        conn.close()


class FakeFC:
    def __init__(self):
        self.m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{FC_PORT}", source_system=1,
                                            source_component=1, dialect="ardupilotmega")
        self.mode = 4            # GUIDED: follow_gps uçuruyor
        self.ch6 = 1500          # MID (CH6 güvenliği kalkar)
        self.mode_cmds = []      # (t, mode)
        self.att = {}            # mode -> SET_ATTITUDE_TARGET sayısı
        self.lock = threading.Lock()
        self.t0 = None
        self.stop = threading.Event()

    def now(self):
        return time.monotonic() - self.t0 if self.t0 else 0.0

    def rx_loop(self):
        while not self.stop.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.3)
            except OSError:
                break                      # soket kapandı (senaryo bitti)
            if msg is None:
                continue
            t = msg.get_type()
            with self.lock:
                if t == "COMMAND_LONG" and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    newm = int(msg.param2)
                    # sender 5 paket atar: aynı komutu tek olay say
                    if not self.mode_cmds or self.mode_cmds[-1][1] != newm \
                            or self.now() - self.mode_cmds[-1][0] > 0.3:
                        self.mode_cmds.append((self.now(), newm))
                    self.mode = newm
                elif t == "SET_MODE":
                    self.mode_cmds.append((self.now(), int(msg.custom_mode)))
                    self.mode = int(msg.custom_mode)
                elif t == "SET_ATTITUDE_TARGET":
                    self.att[self.mode] = self.att.get(self.mode, 0) + 1
                elif t == "PARAM_REQUEST_READ":
                    name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
                    self.m.mav.param_value_send(name.encode(), 0.125,
                                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)

    def run(self, plan, dur, log):
        self.t0 = time.monotonic()
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        pi, last_hb, k, hb_mode = 0, -10.0, 0, None
        while self.now() < dur:
            t = self.now()
            while pi < len(plan) and t >= plan[pi][0]:
                _, what, val = plan[pi]
                with self.lock:
                    if what == 'ch6':
                        self.ch6 = val
                    else:
                        self.mode = val
                log(t, what, val)
                pi += 1
            with self.lock:
                mode, ch6 = self.mode, self.ch6
            if t - last_hb >= 1.0 or mode != hb_mode:      # mod değişince hemen duyur
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                          mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                                          | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                                          mode, mavutil.mavlink.MAV_STATE_ACTIVE)
                last_hb, hb_mode = t, mode
            tb = int(t * 1000)
            self.m.mav.attitude_send(tb, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0)
            if k % 3 == 0:
                self.m.mav.global_position_int_send(tb, 395000000, 327800000, 1100000,
                                                    40000, 0, 0, 0, 9000)
                self.m.mav.local_position_ned_send(tb, 0.0, 0.0, -40.0, 0.0, 0.0, 0.0)
                chans = [1495, 1495, 1500, 1495, 1500, ch6, 1500, 1050] + [1500] * 8
                self.m.mav.rc_channels_send(tb, 16, *chans, 0, 0, 200)
            if k % 15 == 0:
                self.m.mav.sys_status_send(0, 0, 0, 500, 32000, -1, -1, 0, 0, 0, 0, 0, 0)
            k += 1
            time.sleep(1 / 30)
        self.stop.set()


def first_cmd(cmds, t_from, t_to):
    for t, m in cmds:
        if t_from <= t < t_to:
            return t, m
    return None


def run_scenario(key, tmpdir):
    sc = SCENARIOS[key]
    print("\n" + "=" * 74)
    print(" " + sc['title'])
    print("=" * 74)

    try:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", FRAME_PORT))
        srv.listen(1)
        fc = FakeFC()
    except OSError as e:
        print(f"HATA: demo portu meşgul ({FRAME_PORT}/{FC_PORT}): {e}")
        return False
    stop = threading.Event()
    threading.Thread(target=frame_server, args=(srv, stop), daemon=True).start()

    cmd = [sys.executable, "controller_guided.py",
           "--teacher", "hc_air", "--action-units", "rad_s",
           "--teacher-config", "teacher_hc_air.yaml",
           "--hover", "0.101", "--params-set-manually", "--floor-alt", "0",
           "--exit-mode", "loiter", "--enable-override",
           "--fc", f"udpout:127.0.0.1:{FC_PORT}", "--cam-port", str(FRAME_PORT),
           "--log", os.path.join(tmpdir, f"demo_{key}.csv")] + sc['flags']
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    proc = subprocess.Popen(cmd, cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, env=env)
    out = []

    def reader():
        for line in proc.stdout:
            line = line.rstrip()
            out.append(line)
            if VERBOSE:
                print(f"      | {line}")
    threading.Thread(target=reader, daemon=True).start()

    print(f" sahte FC GUIDED'da (follow_gps uçuruyor), kilit var. controller bağlanıyor...")
    if fc.m.wait_heartbeat(timeout=40) is None:
        print("HATA: controller sahte FC'ye bağlanmadı. Çıktısı:")
        for line in out[-20:]:
            print("      | " + line)
        proc.kill()
        stop.set()
        return False

    def log(t, what, val):
        if what == 'ch6':
            s = {1950: 'HIGH', 1050: 'LOW', 1500: 'MID'}.get(val, str(val))
            print(f"  t={t:5.1f}  CH6 {s}")
        else:
            print(f"  t={t:5.1f}  pilot modu kumandadan {MODE_NAME.get(val, val)}'a aldı")
    fc.run(sc['plan'], sc['dur'], log)

    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    stop.set()
    fc.rx_thread.join(timeout=1.0)        # alıcı thread soketi bırakmadan kapatma
    srv.close()
    fc.m.close()

    cmds = fc.mode_cmds

    def said(c):
        return f"{MODE_NAME.get(c[1], c[1])} @ {c[0]:.2f} s" if c else "komut yok"

    checks = []
    if key == 'A':
        c = first_cmd(cmds, 2.0, 4.5)
        checks.append(("GUIDED'da CH6 HIGH -> FC'ye mod komutu YOK", c is None, said(c)))
        c = first_cmd(cmds, 4.5, 8.0)
        checks.append(("LOITER'a alındı, CH6 hâlâ HIGH -> mod komutu YOK", c is None, said(c)))
        c = first_cmd(cmds, 8.0, 10.5)
        checks.append(("CH6 kapat-aç sonrası -> GUIDED_NOGPS",
                       c is not None and c[1] == 20, said(c)))
        c = first_cmd(cmds, 10.5, 99)
        checks.append(("CH6 LOW -> LOITER (çıkış modu)", c is not None and c[1] == 5, said(c)))
        checks.append(("controller [FOLLOW] uyarısını bastı",
                       any("[FOLLOW]" in ln and "KAPALI" in ln for ln in out), ""))
    else:
        c = first_cmd(cmds, 2.0, 4.5)
        checks.append(("GUIDED'da CH6 HIGH -> GUIDED_NOGPS (devir)",
                       c is not None and c[1] == 20 and c[0] < 3.5, said(c)))
        c = first_cmd(cmds, 4.5, 99)
        checks.append(("CH6 LOW -> LOITER (çıkış modu)", c is not None and c[1] == 5, said(c)))
    n20 = fc.att.get(20, 0)
    others = {MODE_NAME.get(k, k): v for k, v in fc.att.items() if k != 20}
    checks.append(("tutum komutu SADECE GUIDED_NOGPS'te", n20 > 0 and not others,
                   f"GUIDED_NOGPS'te {n20} adet" + (f", BAŞKA MODDA {others}" if others else "")))

    print(" " + "-" * 72)
    ok_all = True
    for name, ok, detail in checks:
        ok_all &= bool(ok)
        print(f"  {'✓' if ok else '✗'} {name:52s} {detail}")
    for ln in out:
        if ln.startswith("[FOLLOW]") or "Otonom istendi" in ln or "OTONOM KESILDI" in ln:
            print(f"    controller: {ln[:110]}")
    return ok_all


def main():
    keys = [a.upper() for a in sys.argv[1:] if a.upper() in SCENARIOS] or ['A', 'B']
    print("demo_devir: gerçek FC / MAVProxy / runtracker'a DOKUNMAZ "
          f"(sahte FC udp {FC_PORT}, FRAME tcp {FRAME_PORT}).")
    tmpdir = tempfile.mkdtemp(prefix="demo_devir_")
    results = {k: run_scenario(k, tmpdir) for k in keys}
    print("\n" + "=" * 74)
    for k, ok in results.items():
        print(f" Senaryo {k}: {'GEÇTİ' if ok else 'KALDI'}")
    print(f" (controller logları: {tmpdir})")
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
