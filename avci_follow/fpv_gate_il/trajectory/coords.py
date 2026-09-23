"""Local NED ↔ geodetic helpers for the trajectory stack.

Frame: NED relative to a fixed home/reference
  x = North (m)
  y = East  (m)
  z = Down  (m)   # altitude_up = -z

Do not difference raw lat/lon for planning — convert first.
"""

from __future__ import annotations

import math

from fpv_gate_il.gate_gps import (
    DEFAULT_HOME_ALT_M,
    DEFAULT_HOME_LAT_DEG,
    DEFAULT_HOME_LON_DEG,
)
from fpv_gate_il.trajectory.types import NedPosition

# WGS84 metres per degree (local flat approx; consistent with gate_gps).
_METERS_PER_DEG_LAT = 111320.0


def _meters_per_deg_lon(lat_deg: float) -> float:
    return _METERS_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def latlonalt_to_local(
    lat_deg: float,
    lon_deg: float,
    alt_up_m: float,
    *,
    home_lat_deg: float = DEFAULT_HOME_LAT_DEG,
    home_lon_deg: float = DEFAULT_HOME_LON_DEG,
    home_alt_m: float = DEFAULT_HOME_ALT_M,
) -> NedPosition:
    """
    WGS84 lat/lon + up-positive altitude → local NED metres.

    ``alt_up_m`` is treated as home-relative up (same convention as
    ``gate_gps.enu_to_latlon`` / GLOBAL_RELATIVE_ALT). ``home_alt_m`` is
    reserved for future AMSL conversion.
    """
    _ = home_alt_m
    dlat = float(lat_deg) - float(home_lat_deg)
    dlon = float(lon_deg) - float(home_lon_deg)
    north_m = dlat * _METERS_PER_DEG_LAT
    east_m = dlon * _meters_per_deg_lon(home_lat_deg)
    down_m = -float(alt_up_m)
    return NedPosition(north_m=north_m, east_m=east_m, down_m=down_m)


def local_to_latlonalt(
    position: NedPosition,
    *,
    home_lat_deg: float = DEFAULT_HOME_LAT_DEG,
    home_lon_deg: float = DEFAULT_HOME_LON_DEG,
    home_alt_m: float = DEFAULT_HOME_ALT_M,
) -> tuple[float, float, float]:
    """Local NED → (lat_deg, lon_deg, alt_up_m)."""
    _ = home_alt_m
    lat = float(home_lat_deg) + float(position.north_m) / _METERS_PER_DEG_LAT
    lon = float(home_lon_deg) + float(position.east_m) / max(
        1e-6, _meters_per_deg_lon(home_lat_deg)
    )
    alt_up = -float(position.down_m)
    return lat, lon, alt_up


def enu_xyz_to_ned(
    east_m: float,
    north_m: float,
    up_m: float,
) -> NedPosition:
    """Gazebo-style ENU (x=E, y=N, z=U) → NED position."""
    return NedPosition(north_m=float(north_m), east_m=float(east_m), down_m=-float(up_m))


def ned_to_enu_xyz(position: NedPosition) -> tuple[float, float, float]:
    """NED → Gazebo ENU (east, north, up)."""
    return float(position.east_m), float(position.north_m), float(-position.down_m)


def horizontal_distance_m(a: NedPosition, b: NedPosition) -> float:
    return math.hypot(a.north_m - b.north_m, a.east_m - b.east_m)


def distance_m(a: NedPosition, b: NedPosition) -> float:
    return math.sqrt(
        (a.north_m - b.north_m) ** 2
        + (a.east_m - b.east_m) ** 2
        + (a.down_m - b.down_m) ** 2
    )
