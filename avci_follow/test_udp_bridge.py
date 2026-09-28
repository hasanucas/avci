#!/usr/bin/env python3
"""
Offline test for the serial<->UDP bridge. No radio, no pymavlink.

Proves the two-computer insert: bytes from the "radio" are forwarded only
after a UDP peer appears, and bytes from that peer are written back to the
radio unchanged.

Run:  python test_udp_bridge.py
"""
from __future__ import annotations

import socket
import sys
import threading
import time

sys.path.insert(0, ".")

from serial_udp_bridge import HELLO, SerialUdpBridge, parse_hostport

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
    else:
        FAIL += 1
    print(f"  {'[OK]  ' if cond else '[FAIL]'} {name}{('  — ' + detail) if detail else ''}")


class FakeSerial:
    """Byte pipe standing in for the USB radio."""

    def __init__(self):
        self.timeout = 0.05
        self._rx = bytearray()  # radio -> bridge
        self._tx = bytearray()  # bridge -> radio
        self._lock = threading.Lock()
        self.closed = False

    def inject(self, data):
        with self._lock:
            self._rx.extend(data)

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._rx)

    def read(self, n):
        deadline = time.time() + (self.timeout or 0)
        while not self.closed:
            with self._lock:
                if self._rx:
                    take = min(n, len(self._rx)) if n else len(self._rx)
                    out = bytes(self._rx[:take])
                    del self._rx[:take]
                    return out
            if time.time() >= deadline:
                return b""
            time.sleep(0.005)
        return b""

    def write(self, data):
        buf = bytes(data)
        with self._lock:
            self._tx.extend(buf)
        return len(buf)

    def written(self):
        with self._lock:
            return bytes(self._tx)

    def close(self):
        self.closed = True


def wait_until(pred, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


def _sock():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    return sock


def test_hostport():
    host, port = parse_hostport("192.168.50.1:14560")
    check("host:port ayrisir", host == "192.168.50.1" and port == 14560)
    bad = False
    try:
        parse_hostport("not-a-port")
    except ValueError:
        bad = True
    check("host:port degilse hata", bad)


def test_forward():
    ser = FakeSerial()
    sock = _sock()
    port = sock.getsockname()[1]
    bridge = SerialUdpBridge(ser, sock)
    bridge.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.bind(("127.0.0.1", 0))
    client.settimeout(2.0)
    try:
        ser.inject(b"early-telemetry")
        dropped = wait_until(lambda: bridge.n_dropped >= len(b"early-telemetry"))
        check("karsi uc yokken seri bayt UDP'ye gitmez",
              dropped and bridge.n_up == 0 and bridge.peer is None,
              f"dropped={bridge.n_dropped} up={bridge.n_up}")

        client.sendto(b"stream-request", ("127.0.0.1", port))
        wrote = wait_until(lambda: ser.written() == b"stream-request")
        check("UDP bayti seriye aynen yazilir", wrote, repr(ser.written()))
        check("karsi uc ogrenilir", bridge.peer == client.getsockname(), str(bridge.peer))

        ser.inject(b"gps-fix")
        data, addr = client.recvfrom(65535)
        check("seri bayti karsi uca aynen gider", data == b"gps-fix", repr(data))
        check("cevap karsi ucun portuna gider", addr[0] == "127.0.0.1")
        check("yon sayaclari", bridge.n_down == len(b"stream-request")
              and bridge.n_up == len(b"gps-fix"),
              f"down={bridge.n_down} up={bridge.n_up}")
    finally:
        client.close()
        bridge.close()


def test_two_radios():
    """PC1 listen + PC2 --peer: each radio's bytes appear on the other, hello does not."""
    radio_a = FakeSerial()
    radio_b = FakeSerial()
    sock_a = _sock()
    port_a = sock_a.getsockname()[1]
    sock_b = _sock()
    pc1 = SerialUdpBridge(radio_a, sock_a)
    pc2 = SerialUdpBridge(
        radio_b, sock_b, peer=("127.0.0.1", port_a), announce=True)
    pc1.start()
    pc2.start()
    try:
        learned = wait_until(lambda: pc1.peer is not None)
        check("PC2 hello ile PC1 karsi ucu ogrenir", learned, str(pc1.peer))
        time.sleep(0.05)
        check("hello telsize yazilmaz", HELLO not in radio_a.written()
              and radio_a.written() == b"", repr(radio_a.written()))

        radio_a.inject(b"from-pc1")
        got_b = wait_until(lambda: radio_b.written() == b"from-pc1")
        check("PC1 telsizi PC2 telsizine aynen gider", got_b, repr(radio_b.written()))

        radio_b.inject(b"from-pc2")
        got_a = wait_until(lambda: radio_a.written() == b"from-pc2")
        check("PC2 telsizi PC1 telsizine aynen gider", got_a, repr(radio_a.written()))
    finally:
        pc1.close()
        pc2.close()


def main():
    print("serial<->UDP bridge test (no hardware)")
    test_hostport()
    test_forward()
    test_two_radios()
    print(f"\n{PASS} gecti, {FAIL} kaldi")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
