"""External gate GPS feed (UDP JSON) + geo helpers for GUIDED approach."""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass
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
    alt: float
    recv_mono: float


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
