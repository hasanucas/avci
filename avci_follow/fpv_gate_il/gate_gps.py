"""External gate GPS feed (MAVLink GLOBAL_POSITION_INT) + geo helpers.

The companion listens on its own link (USB telemetry serial, or UDP in sim).
That stream is not the drone's MAVLink connection. ``GateGpsSample.alt`` is
metres above our drone's home: target AMSL minus our home AMSL.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass, replace
from typing import Any


# Default SITL / iris_runway spherical_coordinates origin.
DEFAULT_HOME_LAT_DEG = -35.3632621
DEFAULT_HOME_LON_DEG = 149.1652374
DEFAULT_HOME_ALT_M = 10.0

# WGS84 metres per degree (local flat approx; fine for SITL km-scale).
_METERS_PER_DEG_LAT = 111320.0


@dataclass(frozen=True)
class GateGpsSample:
    lat: float
    lon: float
    # Metres above our home once that home is known. Before then this is the
    # target's own relative_alt and must not be commanded.
    alt: float
    recv_mono: float
    # Target GLOBAL_POSITION_INT.alt (AMSL, metres). None for samples that are
    # already in our frame (coast / JSON).
    alt_amsl_m: float | None = None


def enu_to_latlon(
    x_east_m: float,
    y_north_m: float,
    z_up_m: float,
    *,
    home_lat_deg: float = DEFAULT_HOME_LAT_DEG,
    home_lon_deg: float = DEFAULT_HOME_LON_DEG,
    home_alt_m: float = DEFAULT_HOME_ALT_M,
) -> tuple[float, float, float]:
    """
    Gazebo ENU (x=East, y=North, z=Up) → WGS84 lat/lon + home-relative alt.

    alt return is relative to home (positive up), matching GLOBAL_RELATIVE_ALT.
    """
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(home_lat_deg))
    lat = home_lat_deg + float(y_north_m) / _METERS_PER_DEG_LAT
    lon = home_lon_deg + float(x_east_m) / max(1e-6, meters_per_deg_lon)
    # Gazebo z is absolute world height; home elevation offsets AMSL origin.
    # Relative alt for ArduPilot home ≈ world z (pad near z≈0) when home
    # elevation matches spherical_coordinates elevation. Prefer z as AGL.
    alt_rel = float(z_up_m)
    _ = home_alt_m  # reserved if AMSL conversion is needed later
    return lat, lon, alt_rel


def haversine_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1_deg)
    p2 = math.radians(lat2_deg)
    dp = math.radians(lat2_deg - lat1_deg)
    dl = math.radians(lon2_deg - lon1_deg)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


def bearing_ned_rad(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
) -> float:
    """Initial bearing from point1 → point2, radians from North toward East."""
    p1 = math.radians(lat1_deg)
    p2 = math.radians(lat2_deg)
    dl = math.radians(lon2_deg - lon1_deg)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.atan2(y, x)


def speed_for_range_m(
    distance_m: float,
    *,
    far_m: float = 400.0,
    mid_m: float = 300.0,
    near_m: float = 200.0,
    speed_far_mps: float = 20.0,
    speed_mid_mps: float = 15.0,
    speed_near_mps: float = 10.0,
) -> float:
    speed, _band = speed_band_for_range_m(
        distance_m,
        far_m=far_m,
        mid_m=mid_m,
        near_m=near_m,
        speed_far_mps=speed_far_mps,
        speed_mid_mps=speed_mid_mps,
        speed_near_mps=speed_near_mps,
    )
    return speed


def speed_band_for_range_m(
    distance_m: float,
    *,
    far_m: float = 400.0,
    mid_m: float = 300.0,
    near_m: float = 200.0,
    speed_far_mps: float = 20.0,
    speed_mid_mps: float = 15.0,
    speed_near_mps: float = 10.0,
) -> tuple[float, str]:
    """
    Return (speed_mps, band_label).

    Bands: d>=far → far speed; d>=mid → mid; else near (includes d<near / visual zone).
    """
    d = float(distance_m)
    if d >= float(far_m):
        return float(speed_far_mps), f">={far_m:.0f}m"
    if d >= float(mid_m):
        return float(speed_mid_mps), f"{mid_m:.0f}-{far_m:.0f}m"
    if d >= float(near_m):
        return float(speed_near_mps), f"{near_m:.0f}-{mid_m:.0f}m"
    return float(speed_near_mps), f"<{near_m:.0f}m"


def sample_from_global_position_int(
    *,
    lat_e7: int,
    lon_e7: int,
    relative_alt_mm: int,
    recv_mono: float,
    alt_mm: int | None = None,
) -> GateGpsSample:
    """MAVLink GLOBAL_POSITION_INT → gate sample.

    ``lat``/``lon`` are degE7. ``alt`` is AMSL millimetres. ``relative_alt``
    is the target's own home. ``GateGpsSample.alt`` stays on that relative
    value until ``GateMavlinkReceiver.set_our_home_amsl_m`` subtracts our home.
    """
    amsl = None if alt_mm is None else float(alt_mm) / 1000.0
    return GateGpsSample(
        lat=float(lat_e7) * 1e-7,
        lon=float(lon_e7) * 1e-7,
        alt=float(relative_alt_mm) / 1000.0,
        recv_mono=float(recv_mono),
        alt_amsl_m=amsl,
    )


def alt_above_our_home(sample: GateGpsSample, our_home_amsl_m: float | None) -> float:
    """Target height above our home. Falls back to ``sample.alt`` without AMSL."""
    if our_home_amsl_m is None or sample.alt_amsl_m is None:
        return float(sample.alt)
    return float(sample.alt_amsl_m) - float(our_home_amsl_m)


def adopt_our_home_amsl(receiver: GateMavlinkReceiver, home_amsl_m: float | None) -> None:
    """Push the drone's home AMSL into the gate receiver when it is known."""
    if home_amsl_m is None:
        return
    receiver.set_our_home_amsl_m(float(home_amsl_m))


def mavlink_link_needs_baud(connection: str) -> bool:
    """True for a serial device path. UDP/TCP connection strings ignore baud."""
    text = str(connection).strip()
    lower = text.lower()
    if lower.startswith(("udp", "tcp")):
        return False
    if lower.startswith("serial:"):
        # serial:/dev/ttyUSB0:57600 already carries the rate.
        rest = text.split(":", 1)[1]
        return rest.count(":") == 0
    return text.startswith("/") or text.upper().startswith("COM")


def parse_gate_gps_payload(raw: bytes | str) -> tuple[float, float, float]:
    if isinstance(raw, bytes):
        text = raw.decode("utf-8", errors="replace").strip()
    else:
        text = str(raw).strip()
    if not text:
        raise ValueError("empty gate GPS payload")
    data: Any = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError("gate GPS JSON must be an object")
    try:
        lat = float(data["lat"])
        lon = float(data["lon"])
        alt = float(data["alt"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"gate GPS needs lat/lon/alt numbers: {exc}") from exc
    return lat, lon, alt


class GateGpsReceiver:
    """Background UDP listener; keeps the latest gate lat/lon/alt sample."""

    def __init__(
        self,
        *,
        bind_host: str = "0.0.0.0",
        port: int = 14560,
        stale_timeout_s: float = 3.0,
    ) -> None:
        self.bind_host = bind_host
        self.port = int(port)
        self.stale_timeout_s = float(stale_timeout_s)
        self._lock = threading.Lock()
        self._latest: GateGpsSample | None = None
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._packets = 0
        self._parse_errors = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_host, self.port))
        sock.settimeout(0.2)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop,
            name="GateGpsReceiver",
            daemon=True,
        )
        self._thread.start()
        print(
            f"[GATE-GPS] listening udp://{self.bind_host}:{self.port} "
            f"(stale>{self.stale_timeout_s:.1f}s)",
            flush=True,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _loop(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                continue
            try:
                lat, lon, alt = parse_gate_gps_payload(data)
            except (ValueError, json.JSONDecodeError):
                self._parse_errors += 1
                continue
            sample = GateGpsSample(lat=lat, lon=lon, alt=alt, recv_mono=time.monotonic())
            with self._lock:
                self._latest = sample
                self._packets += 1

    def latest(self, *, require_fresh: bool = False) -> GateGpsSample | None:
        with self._lock:
            sample = self._latest
        if sample is None:
            return None
        if require_fresh and self.is_stale(sample):
            return None
        return sample

    def is_stale(self, sample: GateGpsSample | None = None) -> bool:
        with self._lock:
            s = sample if sample is not None else self._latest
        if s is None:
            return True
        if self.stale_timeout_s <= 0:
            return False
        return (time.monotonic() - s.recv_mono) > self.stale_timeout_s

    def wait_first(self, *, timeout_s: float = 120.0) -> GateGpsSample:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            sample = self.latest()
            if sample is not None:
                print(
                    f"[GATE-GPS] first fix lat={sample.lat:.7f} lon={sample.lon:.7f} "
                    f"alt={sample.alt:.1f}m",
                    flush=True,
                )
                return sample
            time.sleep(0.05)
        raise TimeoutError(
            f"no gate GPS on udp://{self.bind_host}:{self.port} within {timeout_s:.0f}s"
        )

    @property
    def packet_count(self) -> int:
        with self._lock:
            return self._packets


class GateMavlinkReceiver:
    """Background MAVLink listener; keeps the latest target GLOBAL_POSITION_INT.

    ``connection`` is a pymavlink string: ``udpin:0.0.0.0:14560`` in sim, or
    ``/dev/ttyUSB0`` for a USB telemetry radio (``baud`` applies). ``sysid``
    selects the target vehicle; ``0`` accepts any system id.
    """

    def __init__(
        self,
        *,
        connection: str = "udpin:0.0.0.0:14560",
        baud: int = 57600,
        sysid: int = 99,
        stale_timeout_s: float = 3.0,
    ) -> None:
        self.connection = str(connection)
        self.baud = int(baud)
        self.sysid = int(sysid)
        self.stale_timeout_s = float(stale_timeout_s)
        self._lock = threading.Lock()
        self._latest: GateGpsSample | None = None
        self._conn: Any = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._packets = 0
        self._ignored = 0
        self._our_home_amsl_m: float | None = None
        self._announced_home = False

    def set_our_home_amsl_m(self, home_amsl_m: float) -> None:
        """Our drone's home AMSL. Later ``latest()`` alts are above this home."""
        home = float(home_amsl_m)
        if not math.isfinite(home):
            return
        with self._lock:
            prev = self._our_home_amsl_m
            self._our_home_amsl_m = home
            announce = not self._announced_home
            if announce:
                self._announced_home = True
        if announce or (prev is not None and abs(home - prev) >= 1.0):
            print(
                f"[GATE-GPS] our home AMSL={home:.1f}m — "
                f"target alt = target AMSL − this home",
                flush=True,
            )

    def link_label(self) -> str:
        sys = "any" if self.sysid == 0 else str(self.sysid)
        if mavlink_link_needs_baud(self.connection):
            return f"{self.connection} @ {self.baud} baud, sysid={sys}"
        return f"{self.connection} sysid={sys}"

    def start(self) -> None:
        if self._thread is not None:
            return
        from pymavlink import mavutil

        kwargs: dict[str, Any] = {
            "source_system": 255,
            "source_component": 191,
        }
        if mavlink_link_needs_baud(self.connection):
            kwargs["baud"] = self.baud
        self._conn = mavutil.mavlink_connection(self.connection, **kwargs)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop,
            name="GateMavlinkReceiver",
            daemon=True,
        )
        self._thread.start()
        print(
            f"[GATE-GPS] listening mavlink {self.link_label()} "
            f"(stale>{self.stale_timeout_s:.1f}s)",
            flush=True,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        conn = self._conn
        self._conn = None
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    def _loop(self) -> None:
        conn = self._conn
        assert conn is not None
        while not self._stop.is_set():
            try:
                msg = conn.recv_match(
                    type="GLOBAL_POSITION_INT",
                    blocking=True,
                    timeout=0.2,
                )
            except Exception:
                if self._stop.is_set():
                    break
                continue
            if msg is None:
                continue
            src = int(msg.get_srcSystem())
            if self.sysid != 0 and src != self.sysid:
                self._ignored += 1
                continue
            if int(msg.lat) == 0 and int(msg.lon) == 0:
                continue
            sample = sample_from_global_position_int(
                lat_e7=int(msg.lat),
                lon_e7=int(msg.lon),
                alt_mm=int(msg.alt),
                relative_alt_mm=int(msg.relative_alt),
                recv_mono=time.monotonic(),
            )
            with self._lock:
                self._latest = sample
                self._packets += 1

    def latest(self, *, require_fresh: bool = False) -> GateGpsSample | None:
        with self._lock:
            sample = self._latest
            home = self._our_home_amsl_m
        if sample is None:
            return None
        if require_fresh and self.is_stale(sample):
            return None
        if home is not None and sample.alt_amsl_m is not None:
            sample = replace(sample, alt=alt_above_our_home(sample, home))
        return sample

    def is_stale(self, sample: GateGpsSample | None = None) -> bool:
        with self._lock:
            s = sample if sample is not None else self._latest
        if s is None:
            return True
        if self.stale_timeout_s <= 0:
            return False
        return (time.monotonic() - s.recv_mono) > self.stale_timeout_s

    def wait_first(self, *, timeout_s: float = 120.0) -> GateGpsSample:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            sample = self.latest()
            if sample is not None:
                print(
                    f"[GATE-GPS] first fix lat={sample.lat:.7f} lon={sample.lon:.7f} "
                    f"alt={sample.alt:.1f}m",
                    flush=True,
                )
                return sample
            time.sleep(0.05)
        raise TimeoutError(
            f"no gate GPS on {self.link_label()} within {timeout_s:.0f}s"
        )

    @property
    def packet_count(self) -> int:
        with self._lock:
            return self._packets
