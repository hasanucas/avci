"""World-frame line-of-sight from the bridge image angles.

The camera bridge reports ``angular_x_deg`` / ``angular_y_deg`` as a linear
pixel offset (degrees across the FOV), not as a small-angle approximation.
This module turns those angles back into a pinhole ray, then rotates that ray
into NED with the gimbal mount and the body attitude.

Azimuth is measured from north toward east. Elevation is positive above the
horizon. Their rates are the inertial LOS rates: a pure rotation of the drone
does not change them when the target is fixed in the world.

Frames:
  camera  OpenCV, x right, y down, z forward
  body    FRD, x forward, y right, z down
  NED     x north, y east, z down
``camera_pitch_deg`` uses the bridge sign: positive is depression (looking down).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from fpv_gate_il.features import (
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_HFOV_DEG,
    DEFAULT_CAMERA_PITCH_DEG,
    DEFAULT_CAMERA_WIDTH,
)


def wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def vertical_fov_rad(hfov_deg: float, width: float, height: float) -> float:
    hfov = math.radians(float(hfov_deg))
    aspect = float(width) / float(height)
    return 2.0 * math.atan(math.tan(hfov / 2.0) / aspect)


def focal_lengths_px(
    width: float,
    height: float,
    hfov_deg: float,
) -> tuple[float, float]:
    vfov = vertical_fov_rad(hfov_deg, width, height)
    fx = float(width) / (2.0 * math.tan(math.radians(float(hfov_deg)) / 2.0))
    fy = float(height) / (2.0 * math.tan(vfov / 2.0))
    return fx, fy


def image_angles_to_camera_ray(
    ang_x_deg: float,
    ang_y_deg: float,
    *,
    width: float = DEFAULT_CAMERA_WIDTH,
    height: float = DEFAULT_CAMERA_HEIGHT,
    hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
) -> np.ndarray:
    """Unit ray in the camera frame. +ang_x is right, +ang_y is down in the image."""
    vfov_deg = math.degrees(vertical_fov_rad(hfov_deg, width, height))
    px = float(ang_x_deg) * float(width) / float(hfov_deg)
    py = float(ang_y_deg) * float(height) / vfov_deg
    fx, fy = focal_lengths_px(width, height, hfov_deg)
    ray = np.array([px / fx, py / fy, 1.0], dtype=np.float64)
    return ray / np.linalg.norm(ray)


def camera_ray_to_image_angles(
    ray_cam: np.ndarray,
    *,
    width: float = DEFAULT_CAMERA_WIDTH,
    height: float = DEFAULT_CAMERA_HEIGHT,
    hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
) -> tuple[float, float]:
    """Inverse of ``image_angles_to_camera_ray`` for rays in front of the camera."""
    z = float(ray_cam[2])
    if z <= 1e-6:
        raise ValueError("ray is behind the camera")
    fx, fy = focal_lengths_px(width, height, hfov_deg)
    vfov_deg = math.degrees(vertical_fov_rad(hfov_deg, width, height))
    px = float(ray_cam[0]) / z * fx
    py = float(ray_cam[1]) / z * fy
    ang_x = px * float(hfov_deg) / float(width)
    ang_y = py * vfov_deg / float(height)
    return ang_x, ang_y


def camera_to_body(camera_pitch_deg: float) -> np.ndarray:
    """Rotation whose columns are the camera axes expressed in the body frame.

    Positive ``camera_pitch_deg`` depresses the optical axis (looks down).
    """
    depression = math.radians(float(camera_pitch_deg))
    c = math.cos(depression)
    s = math.sin(depression)
    return np.array(
        [
            [0.0, -s, c],
            [1.0, 0.0, 0.0],
            [0.0, c, s],
        ],
        dtype=np.float64,
    )


def body_to_ned(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Body-to-NED rotation. Yaw is from north toward east, pitch-up positive."""
    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


G_MPS2 = 9.80665


def horizontal_pn_accel(
    az_rad: float,
    az_rate_rad_s: float,
    vc_mps: float,
    nav_n: float,
) -> tuple[float, float]:
    """PN acceleration in the horizontal NED plane, ``(north, east)`` m/s².

    Positive azimuth rate (bearing swinging east) accelerates east when the
    target is to the north.
    """
    lateral = float(nav_n) * float(vc_mps) * float(az_rate_rad_s)
    return -math.sin(az_rad) * lateral, math.cos(az_rad) * lateral


def closing_accel(az_rad: float, accel_mps2: float) -> tuple[float, float]:
    """Acceleration toward the horizontal bearing, ``(north, east)`` m/s²."""
    return math.cos(az_rad) * float(accel_mps2), math.sin(az_rad) * float(accel_mps2)


def ned_accel_to_attitude(
    a_north: float,
    a_east: float,
    yaw_rad: float,
    g_mps2: float = G_MPS2,
) -> tuple[float, float]:
    """Map a horizontal acceleration to ``(roll_deg, pitch_deg)``.

    Positive roll banks right. Negative pitch is nose-down. Yaw is the current
    heading (0 = north, positive toward east); it only resolves the command
    into body axes, it does not choose the acceleration direction.
    """
    yaw = float(yaw_rad)
    a_fwd = float(a_north) * math.cos(yaw) + float(a_east) * math.sin(yaw)
    a_right = -float(a_north) * math.sin(yaw) + float(a_east) * math.cos(yaw)
    g = max(float(g_mps2), 1.0)
    pitch_deg = math.degrees(math.atan2(-a_fwd, g))
    roll_deg = math.degrees(math.atan2(a_right, g))
    return roll_deg, pitch_deg


def ned_los_angles(v_ned: np.ndarray) -> tuple[float, float]:
    """Return ``(azimuth_rad, elevation_rad)``. Elevation is positive upward."""
    north, east, down = float(v_ned[0]), float(v_ned[1]), float(v_ned[2])
    horiz = math.hypot(north, east)
    azimuth = math.atan2(east, north)
    elevation = math.atan2(-down, horiz)
    return azimuth, elevation


def image_to_ned_los(
    ang_x_deg: float,
    ang_y_deg: float,
    *,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    camera_pitch_deg: float = DEFAULT_CAMERA_PITCH_DEG,
    width: float = DEFAULT_CAMERA_WIDTH,
    height: float = DEFAULT_CAMERA_HEIGHT,
    hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
) -> tuple[float, float, np.ndarray]:
    """Image angles and attitude to ``(azimuth_rad, elevation_rad, unit_ned)``."""
    ray = image_angles_to_camera_ray(
        ang_x_deg, ang_y_deg, width=width, height=height, hfov_deg=hfov_deg,
    )
    v_body = camera_to_body(camera_pitch_deg) @ ray
    v_ned = body_to_ned(roll_deg, pitch_deg, yaw_deg) @ v_body
    n = float(np.linalg.norm(v_ned))
    if n < 1e-9:
        v_ned = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        v_ned = v_ned / n
    az, el = ned_los_angles(v_ned)
    return az, el, v_ned


def ned_point_to_image_angles(
    point_ned: np.ndarray,
    *,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    camera_pitch_deg: float = DEFAULT_CAMERA_PITCH_DEG,
    width: float = DEFAULT_CAMERA_WIDTH,
    height: float = DEFAULT_CAMERA_HEIGHT,
    hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
) -> tuple[float, float]:
    """Project a NED vector from the drone to the target into bridge image angles."""
    v_ned = np.asarray(point_ned, dtype=np.float64)
    r_ned_body = body_to_ned(roll_deg, pitch_deg, yaw_deg)
    v_body = r_ned_body.T @ v_ned
    v_cam = camera_to_body(camera_pitch_deg).T @ v_body
    return camera_ray_to_image_angles(
        v_cam, width=width, height=height, hfov_deg=hfov_deg,
    )


@dataclass(frozen=True)
class LosRateSample:
    az_deg: float
    el_deg: float
    az_rate_dps: float
    el_rate_dps: float
    az_rate_raw_dps: float
    el_rate_raw_dps: float
    valid: bool


class LosRateTracker:
    """α-β filter on world-frame azimuth and elevation.

    Steady-state lag on a constant rate is zero. A gap in the track re-inits
    the filter so a reacquired target does not look like a huge LOS rate.
    """

    def __init__(
        self,
        *,
        alpha: float = 0.36,
        lost_decay: float = 0.8,
        innovation_max_deg: float = 15.0,
        rate_max_dps: float = 200.0,
        reinit_after_s: float = 0.4,
        az_horizon_min: float = 0.2,
        camera_pitch_default_deg: float = DEFAULT_CAMERA_PITCH_DEG,
        width: float = DEFAULT_CAMERA_WIDTH,
        height: float = DEFAULT_CAMERA_HEIGHT,
        hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
    ):
        alpha = float(np.clip(alpha, 1e-3, 0.99))
        self.alpha = alpha
        # Benedict-Bordner: constant-rate tracking without overshoot.
        self.beta = alpha * alpha / (2.0 - alpha)
        self.lost_decay = float(lost_decay)
        self.innovation_max_rad = math.radians(float(innovation_max_deg))
        self.rate_max_rad_s = math.radians(float(rate_max_dps))
        self.reinit_after_s = float(reinit_after_s)
        self.az_horizon_min = float(az_horizon_min)
        self.camera_pitch_default_deg = float(camera_pitch_default_deg)
        self.width = float(width)
        self.height = float(height)
        self.hfov_deg = float(hfov_deg)
        self.reset()

    def reset(self) -> None:
        self._ready = False
        self._lost_s = 0.0
        self._az = 0.0
        self._el = 0.0
        self._az_rate = 0.0
        self._el_rate = 0.0

    def update(
        self,
        *,
        ang_x_deg: float,
        ang_y_deg: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        camera_pitch_deg: float,
        dt: float,
        valid: bool,
    ) -> LosRateSample:
        dt = float(dt)
        if not valid:
            self._lost_s += max(dt, 0.0)
            decay = self.lost_decay if dt > 0.0 else 1.0
            self._az_rate *= decay
            self._el_rate *= decay
            if self._lost_s >= self.reinit_after_s:
                self._ready = False
            return self._sample(False, 0.0, 0.0)

        self._lost_s = 0.0
        az, el, v_ned = image_to_ned_los(
            ang_x_deg,
            ang_y_deg,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            camera_pitch_deg=camera_pitch_deg,
            width=self.width,
            height=self.height,
            hfov_deg=self.hfov_deg,
        )
        horiz = math.hypot(float(v_ned[0]), float(v_ned[1]))
        if not self._ready or dt <= 0.0:
            self._az = az
            self._el = el
            self._az_rate = 0.0
            self._el_rate = 0.0
            self._ready = True
            return self._sample(True, 0.0, 0.0)

        raw_az = wrap_pi(az - self._az) / dt
        raw_el = (el - self._el) / dt
        az_pred = wrap_pi(self._az + self._az_rate * dt)
        el_pred = self._el + self._el_rate * dt
        innov_az = wrap_pi(az - az_pred)
        innov_el = el - el_pred
        lim = self.innovation_max_rad
        innov_az = float(np.clip(innov_az, -lim, lim))
        innov_el = float(np.clip(innov_el, -lim, lim))

        self._el = el_pred + self.alpha * innov_el
        self._el_rate = self._el_rate + (self.beta / dt) * innov_el
        if horiz >= self.az_horizon_min:
            self._az = wrap_pi(az_pred + self.alpha * innov_az)
            self._az_rate = self._az_rate + (self.beta / dt) * innov_az
        else:
            self._az_rate *= self.lost_decay

        cap = self.rate_max_rad_s
        self._az_rate = float(np.clip(self._az_rate, -cap, cap))
        self._el_rate = float(np.clip(self._el_rate, -cap, cap))
        return self._sample(True, raw_az, raw_el)

    def _sample(self, valid: bool, raw_az: float, raw_el: float) -> LosRateSample:
        return LosRateSample(
            az_deg=math.degrees(self._az),
            el_deg=math.degrees(self._el),
            az_rate_dps=math.degrees(self._az_rate),
            el_rate_dps=math.degrees(self._el_rate),
            az_rate_raw_dps=math.degrees(raw_az),
            el_rate_raw_dps=math.degrees(raw_el),
            valid=valid,
        )
