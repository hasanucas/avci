"""Constant-velocity Kalman filter for target tracking.

State: [n, e, d, vn, ve, vd] in local NED.
Measurement: position [n, e, d] (from radar after coord transform).

Course is derived from filtered velocity, not from raw radar heading.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from fpv_gate_il.trajectory.config import TrajectoryConfig
from fpv_gate_il.trajectory.radar import RadarMeasurement
from fpv_gate_il.trajectory.types import NedPosition, NedVelocity, TargetState

_EPS = 1e-9


def _transition_matrix(dt: float) -> np.ndarray:
    """Constant-velocity F for state [p, v]."""
    f = np.eye(6, dtype=float)
    f[0, 3] = dt
    f[1, 4] = dt
    f[2, 5] = dt
    return f


def _process_noise(dt: float, qa: float, q_pos: float) -> np.ndarray:
    """
    Discrete white-noise acceleration Q (same qa on N/E/D), plus small
    position random-walk from ``q_pos`` for numerical robustness.

    Per-axis block for [p, v]:
      qa^2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]
    """
    dt = max(float(dt), 0.0)
    q = float(qa) * float(qa)
    q_extra = float(q_pos) * float(q_pos) * max(dt, _EPS)
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt2 * dt2

    q_mat = np.zeros((6, 6), dtype=float)
    for i in range(3):
        q_mat[i, i] = q * dt4 / 4.0 + q_extra
        q_mat[i, i + 3] = q * dt3 / 2.0
        q_mat[i + 3, i] = q * dt3 / 2.0
        q_mat[i + 3, i + 3] = q * dt2
    return q_mat


def _measurement_matrix() -> np.ndarray:
    h = np.zeros((3, 6), dtype=float)
    h[0, 0] = 1.0
    h[1, 1] = 1.0
    h[2, 2] = 1.0
    return h


def _velocity_from_radar(measurement: RadarMeasurement) -> Optional[np.ndarray]:
    """Optional NED velocity from radar ground_speed + course (vd=0)."""
    if measurement.ground_speed_mps is None or measurement.course_rad is None:
        return None
    speed = float(measurement.ground_speed_mps)
    course = float(measurement.course_rad)
    return np.array(
        [speed * np.cos(course), speed * np.sin(course), 0.0],
        dtype=float,
    )


@dataclass
class KalmanTargetTracker:
    """Linear KF with constant-velocity process model (v1)."""

    config: TrajectoryConfig
    x: Optional[np.ndarray] = None  # (6,)
    P: Optional[np.ndarray] = None  # (6, 6)
    last_timestamp_s: Optional[float] = None
    _last_heading_rad: Optional[float] = field(default=None, repr=False)

    def reset(self) -> None:
        self.x = None
        self.P = None
        self.last_timestamp_s = None
        self._last_heading_rad = None

    @property
    def initialized(self) -> bool:
        return self.x is not None

    def predict(self, timestamp_s: float) -> None:
        """Time-update to ``timestamp_s`` (no measurement)."""
        if self.x is None or self.P is None or self.last_timestamp_s is None:
            return
        dt = float(timestamp_s) - float(self.last_timestamp_s)
        if dt <= _EPS:
            return

        f = _transition_matrix(dt)
        q = _process_noise(
            dt,
            qa=float(self.config.kf_process_noise_vel),
            q_pos=float(self.config.kf_process_noise_pos),
        )
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q
        self.last_timestamp_s = float(timestamp_s)

    def update(self, measurement: RadarMeasurement) -> TargetState:
        """
        Convert measurement → NED, run predict+update, return TargetState.

        Cold start: seed position from first fix, velocity from optional
        radar speed/course or zero (KF learns velocity on later fixes).
        """
        cfg = self.config
        z_pos = measurement.to_ned(
            home_lat_deg=cfg.home_lat_deg,
            home_lon_deg=cfg.home_lon_deg,
            home_alt_m=cfg.home_alt_m,
        ).as_array()
        t = float(measurement.timestamp_s)
        if measurement.heading_rad is not None:
            self._last_heading_rad = float(measurement.heading_rad)

        if self.x is None or self.P is None:
            self._cold_start(measurement, z_pos, t)
            state = self.state()
            assert state is not None
            return state

        self.predict(t)

        r_std = float(cfg.kf_measurement_noise_pos)
        r = np.eye(3, dtype=float) * (r_std * r_std)
        h = _measurement_matrix()
        assert self.x is not None and self.P is not None

        innovation = z_pos - h @ self.x
        s = h @ self.P @ h.T + r
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ innovation
        i_kh = np.eye(6) - k @ h
        # Joseph form for covariance update stability.
        self.P = i_kh @ self.P @ i_kh.T + k @ r @ k.T
        self.last_timestamp_s = t

        state = self.state()
        assert state is not None
        return state

    def _cold_start(
        self,
        measurement: RadarMeasurement,
        z_pos: np.ndarray,
        t: float,
    ) -> None:
        vel = _velocity_from_radar(measurement)
        vel_known = vel is not None
        if vel is None:
            vel = np.zeros(3, dtype=float)

        self.x = np.concatenate([z_pos.astype(float), vel.astype(float)])
        r_std = float(self.config.kf_measurement_noise_pos)
        v_std = 5.0 if vel_known else 50.0
        self.P = np.diag(
            [
                r_std * r_std,
                r_std * r_std,
                r_std * r_std,
                v_std * v_std,
                v_std * v_std,
                v_std * v_std,
            ]
        ).astype(float)
        self.last_timestamp_s = t

    def state(self) -> Optional[TargetState]:
        """Current filtered TargetState, or None if not initialized."""
        if self.x is None or self.last_timestamp_s is None:
            return None
        pos = NedPosition.from_array(self.x[0:3])
        vel = NedVelocity.from_array(self.x[3:6])
        return TargetState(
            position=pos,
            velocity=vel,
            timestamp_s=float(self.last_timestamp_s),
            speed_mps=vel.horizontal_speed_mps,
            course_rad=vel.course_rad,
            heading_rad=self._last_heading_rad,
            covariance=None if self.P is None else self.P.copy(),
        )
