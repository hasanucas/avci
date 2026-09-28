#!/usr/bin/env python3
"""
serial_udp_bridge.py — copy USB telemetry bytes to the other computer.

No planner, no MAVLink parsing, no commands. Bytes read from the local radio
are written to the other computer's radio, and bytes from that radio come back
the same way.

  radio --USB-->  PC1 (listen)  --UDP-->  PC2 (--peer)  --USB-->  radio

Start the listener first. The --peer side sends a small hello so PC1 learns
the return address; that hello is not written to either radio.

  PC1: python serial_udp_bridge.py --serial COM12 --baud 57600 --udp-port 14560
  PC2: python serial_udp_bridge.py --serial COM5 --baud 57600 --peer 192.168.50.1:14560
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time

# Internal only. Never written to a radio.
HELLO = b"AVCI-PING"


def parse_hostport(endpoint):
    """'192.168.50.1:14560' -> ('192.168.50.1', 14560)."""
    text = (endpoint or "").strip()
    host, sep, port_s = text.rpartition(":")
    if sep != ":" or not host or not port_s.isdigit():
        raise ValueError(
            "UDP adresi host:port olmali (ornek 192.168.50.1:14560), "
            f"gelen: {endpoint!r}")
    port = int(port_s)
    if not 1 <= port <= 65535:
        raise ValueError(f"UDP port aralik disi: {port}")
    return host, port


class SerialUdpBridge:
    """Bidirectional raw-byte copy between one serial port and one UDP peer.

    Until a peer is known, bytes from the radio are counted and dropped.
    """

    def __init__(self, ser, sock, peer=None, announce=False):
        self.ser = ser
        self.sock = sock
        self.sock.settimeout(0.2)
        self.stop = threading.Event()
        self.peer = peer
        self.announce = announce
        self.n_up = 0        # serial -> udp
        self.n_down = 0      # udp -> serial
        self.n_dropped = 0   # serial bytes dropped before a peer exists
        self._lock = threading.Lock()
        self._threads = []

    def start(self):
        self._threads = [
            threading.Thread(target=self._udp_to_serial, name="udp-to-serial", daemon=True),
            threading.Thread(target=self._serial_to_udp, name="serial-to-udp", daemon=True),
        ]
        if self.announce:
            self._threads.append(
                threading.Thread(target=self._hello_loop, name="udp-hello", daemon=True))
        for t in self._threads:
            t.start()

    def close(self):
        self.stop.set()
        try:
            self.sock.close()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        for t in self._threads:
            t.join(timeout=1.0)

    def _hello_loop(self):
        while not self.stop.is_set():
            with self._lock:
                peer = self.peer
            if peer is not None:
                try:
                    self.sock.sendto(HELLO, peer)
                except OSError:
                    if self.stop.is_set():
                        break
            if self.stop.wait(1.0):
                break

    def _udp_to_serial(self):
        while not self.stop.is_set():
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if self.stop.is_set():
                    break
                time.sleep(0.2)
                continue
            if not data:
                continue
            with self._lock:
                if self.peer != addr:
                    self.peer = addr
                    print(f"[bridge] UDP karsi uc: {addr[0]}:{addr[1]}", flush=True)
            if data == HELLO:
                continue
            self._write_all(data)
            self.n_down += len(data)

    def _serial_to_udp(self):
        while not self.stop.is_set():
            try:
                # One datagram per quiet gap. read() returns early on timeout,
                # so a burst becomes a single UDP packet instead of one byte each.
                data = self.ser.read(1024)
            except Exception as e:
                if self.stop.is_set():
                    break
                print(f"[bridge] seri okuma hatasi: {e}", flush=True)
                time.sleep(0.2)
                continue
            if not data:
                continue
            with self._lock:
                peer = self.peer
            if peer is None:
                self.n_dropped += len(data)
                continue
            try:
                self.sock.sendto(data, peer)
            except OSError:
                if self.stop.is_set():
                    break
                time.sleep(0.2)
                continue
            self.n_up += len(data)

    def _write_all(self, data):
        view = memoryview(data)
        while view and not self.stop.is_set():
            n = self.ser.write(view)
            if not n:
                break
            view = view[n:]

    def status_line(self):
        with self._lock:
            peer = self.peer
        peer_s = f"{peer[0]}:{peer[1]}" if peer else "yok (karsi uc bekleniyor)"
        return (f"[bridge] karsi={peer_s}  serial->udp={self.n_up} B  "
                f"udp->serial={self.n_down} B  atilan={self.n_dropped} B")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", required=True, help="telemetri telsizi, or: COM12")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--bind", default="0.0.0.0",
                    help="dinlenen adres (varsayilan tum arayuzler)")
    ap.add_argument("--udp-port", type=int, default=14560,
                    help="dinlenen UDP portu (yalniz --peer yokken)")
    ap.add_argument("--peer", default=None,
                    help="karsi bilgisayar host:port, or: 192.168.50.1:14560. "
                         "Verilirse bu makine dinlemez, oraya baglanir.")
    a = ap.parse_args()

    peer = None
    announce = False
    local_port = a.udp_port
    if a.peer:
        try:
            peer = parse_hostport(a.peer)
        except ValueError as e:
            sys.exit(f"[X] {e}")
        announce = True
        local_port = 0
    elif not 1 <= a.udp_port <= 65535:
        sys.exit(f"[X] UDP port aralik disi: {a.udp_port}")

    try:
        import serial
    except ImportError:
        sys.exit("[X] pyserial yok. Kur: pip install pyserial")

    try:
        # 20 ms collects a short run of bytes already on the wire (~100 B at 57600).
        ser = serial.Serial(a.serial, a.baud, timeout=0.02)
    except Exception as e:
        sys.exit(f"[X] seri port acilamadi ({a.serial} @ {a.baud}): {e}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((a.bind, local_port))
    except OSError as e:
        ser.close()
        sys.exit(f"[X] UDP {a.bind}:{local_port} acilamadi: {e}")

    bridge = SerialUdpBridge(ser, sock, peer=peer, announce=announce)
    bound = sock.getsockname()
    print(f"[bridge] {a.serial} @ {a.baud} acildi", flush=True)
    if peer is None:
        print(f"[bridge] UDP dinleniyor {a.bind}:{a.udp_port}", flush=True)
        print("[bridge] Karsi bilgisayar bu makineye baglanir:", flush=True)
        print(f"         serial_udp_bridge.py --serial COMx --peer <bu_makine_ip>:{a.udp_port}",
              flush=True)
    else:
        print(f"[bridge] UDP yerel port {bound[1]}, hedef {peer[0]}:{peer[1]}", flush=True)
        print("[bridge] Gelen bayt hedef telsize, telsizden gelen bayt hedefe gider.",
              flush=True)
    print("[bridge] Hesap yok. Ctrl-C ile cik.", flush=True)
    bridge.start()
    try:
        while not bridge.stop.is_set():
            time.sleep(2.0)
            print(bridge.status_line(), flush=True)
    except KeyboardInterrupt:
        print("\n[bridge] durduruldu.", flush=True)
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
