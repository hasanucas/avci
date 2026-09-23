"""Radar measurement model + optional synthetic noise."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from fpv_gate_il.gate_gps import (
    DEFAULT_HOME_ALT_M,
    DEFAULT_HOME_LAT_DEG,
    DEFAULT_HOME_LON_DEG,
)
from fpv_gate_il.trajectory.coords import latlonalt_to_local, local_to_latlonalt
from fpv_gate_il.trajectory.types import NedPosition

_EPS = 1e-9


@dataclass(frozen=True)
class RadarMeasurement:
    """One radar fix.

    Required: timestamp, lat, lon, alt (up-positive / home-relative).
    Optional: ground_speed, course (rad from North), heading (rad).
    If speed/course omitted, successive fixes can derive them.
    """

    timestamp_s: float
    latitude_deg: float
    longitude_deg: float
    altitude_up_m: float
    ground_speed_mps: Optional[float] = None
    course_rad: Optional[float] = None
    heading_rad: Optional[float] = None

    def to_ned(
        self,
        *,
        home_lat_deg: float = DEFAULT_HOME_LAT_DEG,
        home_lon_deg: float = DEFAULT_HOME_LON_DEG,
        home_alt_m: float = DEFAULT_HOME_ALT_M,
    ) -> NedPosition:
        return latlonalt_to_local(
            self.latitude_deg,
            self.longitude_deg,
            self.altitude_up_m,
            home_lat_deg=home_lat_deg,
            home_lon_deg=home_lon_deg,
            home_alt_m=home_alt_m,
        )


def add_gaussian_position_noise(
    measurement: RadarMeasurement,
    *,
    position_std_m: float,
    rng: np.random.Generator | None = None,
    home_lat_deg: float = DEFAULT_HOME_LAT_DEG,
    home_lon_deg: float = DEFAULT_HOME_LON_DEG,
    home_alt_m: float = DEFAULT_HOME_ALT_M,
) -> RadarMeasurement:
    """
    Synthetic radar noise: true local NED + N(0, std^2 I) → lat/lon/alt.

    Speed/course/heading fields are copied unchanged (caller may re-derive).
    """
    std = float(position_std_m)
    if std <= 0.0:
        return measurement

    gen = rng if rng is not None else np.random.default_rng()
    noise = gen.normal(0.0, std, size=3)
    ned = measurement.to_ned(
        home_lat_deg=home_lat_deg,
        home_lon_deg=home_lon_deg,
        home_alt_m=home_alt_m,
    )
    noisy = NedPosition(
        north_m=ned.north_m + float(noise[0]),
        east_m=ned.east_m + float(noise[1]),
        down_m=ned.down_m + float(noise[2]),
    )
    lat, lon, alt = local_to_latlonalt(
        noisy,
        home_lat_deg=home_lat_deg,
        home_lon_deg=home_lon_deg,
        home_alt_m=home_alt_m,
    )
    return RadarMeasurement(
        timestamp_s=measurement.timestamp_s,
        latitude_deg=lat,
        longitude_deg=lon,
        altitude_up_m=alt,
        ground_speed_mps=measurement.ground_speed_mps,
        course_rad=measurement.course_rad,
        heading_rad=measurement.heading_rad,
    )


def derive_speed_course_from_fixes(
    prev: RadarMeasurement,
    curr: RadarMeasurement,
    *,
    home_lat_deg: float = DEFAULT_HOME_LAT_DEG,
    home_lon_deg: float = DEFAULT_HOME_LON_DEG,
    home_alt_m: float = DEFAULT_HOME_ALT_M,
) -> tuple[float, float]:
    """
    Return (ground_speed_mps, course_rad) from two consecutive fixes.

    Uses horizontal NED displacement / dt. Course is atan2(east, north).
    """
    dt = float(curr.timestamp_s) - float(prev.timestamp_s)
    if dt <= _EPS:
        raise ValueError(f"derive_speed_course_from_fixes requires dt > 0, got {dt}")

    p0 = prev.to_ned(
        home_lat_deg=home_lat_deg,
        home_lon_deg=home_lon_deg,
        home_alt_m=home_alt_m,
    )
    p1 = curr.to_ned(
        home_lat_deg=home_lat_deg,
        home_lon_deg=home_lon_deg,
        home_alt_m=home_alt_m,
    )
    dn = p1.north_m - p0.north_m
    de = p1.east_m - p0.east_m
    speed = math.hypot(dn, de) / dt
    course = math.atan2(de, dn)
    return float(speed), float(course)


def velocity_from_fix_window(
    fixes: list[tuple[float, float, float, float]],
) -> Optional[tuple[float, float, float]]:
    """
    Least-squares NED velocity (vn, ve, vd) from timed fixes.

    Each fix is ``(timestamp_s, north_m, east_m, down_m)``. Needs ≥2 points
    with positive time span. Horizontal slopes come from n(t)/e(t); down
    from d(t).
    """
    if len(fixes) < 2:
        return None
    t0 = float(fixes[0][0])
    ts = np.array([float(f[0]) - t0 for f in fixes], dtype=float)
    span = float(ts[-1] - ts[0])
    if span <= _EPS:
        return None
    north = np.array([float(f[1]) for f in fixes], dtype=float)
    east = np.array([float(f[2]) for f in fixes], dtype=float)
    down = np.array([float(f[3]) for f in fixes], dtype=float)
    # deg=1 → [slope, intercept]; slope is metres per second.
    vn = float(np.polyfit(ts, north, 1)[0])
    ve = float(np.polyfit(ts, east, 1)[0])
    vd = float(np.polyfit(ts, down, 1)[0])
    if not (math.isfinite(vn) and math.isfinite(ve) and math.isfinite(vd)):
        return None
    return vn, ve, vd
