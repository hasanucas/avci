#!/usr/bin/env python3
"""
ChaserEnv — a thin pymavlink adapter that lets the vendored trajectory planner
(`fpv_gate_il.trajectory.apply_planner_output`) drive a *real* ArduPilot drone
over a telemetry radio, with no Gazebo/SITL.

`apply_planner_output(env, ...)` only ever calls four methods on `env`:

    env.get_drone_state()                  -> {global_valid, lat_deg, lon_deg}
    env.send_position_target_global(...)   -> GUIDED position setpoint
    env.send_velocity_ned(...)             -> GUIDED velocity setpoint (realign only)
    env.set_guided_horizontal_speed_mps(...) -> DO_CHANGE_SPEED

Plus `configure_guided_speed_limits(...)` once at startup to raise WP_SPD*.

The MAVLink construction inside these methods is copied byte-for-byte from the
simulation env (`fpv_gate_il/gazebo_drone_env.py`) so the drone sees the exact
same messages the planner was validated against — only `self.mav` now points at
the chaser's telemetry link (the follow-relay Vehicle's mavutil connection), and
the live lat/lon comes from that Vehicle's receive thread instead of SITL.

WHY THIS FIXES THE "always 10 m/s" CAP
--------------------------------------
ArduCopter snapshots WP_SPD into the position controller when a GUIDED position
target starts; a mid-flight WP_SPD PARAM_SET alone does not change the active
speed, and every new position target re-reads WP_SPD. The old follow_relay set
neither a raised WP_SPD ceiling nor a per-tick DO_CHANGE_SPEED, so the vehicle
fell back to the factory WP_SPD (~10 m/s). Here:
  * configure_guided_speed_limits() PARAM_SETs WP_SPD/UP/DN/ACC to the peak once, and
  * set_guided_horizontal_speed_mps() re-issues DO_CHANGE_SPEED every control tick
    (throttled by min_interval/min_delta), which is the only thing that changes
    the *active* GUIDED groundspeed.
Together the vehicle can actually reach the commanded 20 m/s when far from the
target (the planner tapers it down near the target by design).
"""
from __future__ import annotations

import time

from pymavlink import mavutil


def _coord_valid(lat: float | None, lon: float | None, fix: int) -> bool:
    """3D fix and a real coordinate (reject 0,0 'null island')."""
    if lat is None or lon is None or fix < 3:
        return False
    return not (abs(lat) < 1e-6 and abs(lon) < 1e-6)


class ChaserEnv:
    """Duck-typed env backed by the chaser telemetry link.

    Pass the follow-relay Vehicle for the chaser; its `.conn` is the mavutil
    connection used for every send, and its snapshot supplies live lat/lon.
    """

    def __init__(self, chaser_vehicle, *, verbose_speed: bool = False,
                 verbose_param: bool = True, cmd_latency=None):
        self.vehicle = chaser_vehicle
        self.mav = chaser_vehicle.conn          # mavutil connection (source_system=250)
        self._verbose_speed = bool(verbose_speed)
        self._verbose_param = bool(verbose_param)
        self._cmd_lat = cmd_latency             # link_log.CmdLatencyTracker or None
        # DO_CHANGE_SPEED throttle state (see set_guided_horizontal_speed_mps)
        self._wp_spd_mps = -1.0
        self._wp_spd_cmd_t = 0.0
        # live global position, refreshed each tick from the Vehicle snapshot
        self._mavlink_global_valid = False
        self._mavlink_lat_deg = 0.0
        self._mavlink_lon_deg = 0.0
        # optional posvel feed-forward (off by default -> byte-identical sends).
        # When enabled, the GUIDED position target also carries the target's
        # NED velocity so ArduCopter leads a moving target (lower lag). Rarely
        # needed with the carrot planner, kept for parity with follow_relay.
        self._posvel = False
        self._ff_vel = None  # (vn, ve, vd) or None

    # ------------------------------------------------------------------ live state
    def refresh_from_snapshot(self, snap: dict) -> None:
        """Call once per control tick, before apply_planner_output."""
        valid = _coord_valid(snap.get("lat"), snap.get("lon"), int(snap.get("fix", 0)))
        self._mavlink_global_valid = valid
        if snap.get("lat") is not None:
            self._mavlink_lat_deg = float(snap["lat"])
            self._mavlink_lon_deg = float(snap["lon"])

    def ping_rtt(self, now_mono: float) -> None:
        """Send a side-effect-free ACK-eliciting command (REQUEST_MESSAGE for
        SYSTEM_TIME) and time it, so command->response RTT is sampled even when
        no DO_CHANGE_SPEED traffic is flowing (steady speed). Read-only on the FC."""
        if self.mav is None or self._cmd_lat is None:
            return
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME),
            0, 0, 0, 0, 0, 0,
        )
        self._cmd_lat.mark_command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, now_mono)

    def get_drone_state(self) -> dict:
        # apply_planner_output only reads these three keys.
        return {
            "global_valid": self._mavlink_global_valid,
            "lat_deg": self._mavlink_lat_deg,
            "lon_deg": self._mavlink_lon_deg,
        }

    def enable_posvel(self, on: bool = True) -> None:
        self._posvel = bool(on)

    def set_feedforward_velocity(self, vn: float, ve: float, vd: float) -> None:
        """Stash the target's NED velocity for the next position target
        (used only when posvel is enabled)."""
        self._ff_vel = (float(vn), float(ve), float(vd))

    # ------------------------------------------------------------------ GUIDED sends
    # NOTE: bodies below are copied verbatim from gazebo_drone_env.py so the
    # drone receives identical MAVLink; only logging is made field-friendly.
    def send_position_target_global(
        self,
        *,
        lat_deg: float,
        lon_deg: float,
        alt_rel_m: float,
        yaw_rad: float | None = None,
    ) -> None:
        """GUIDED position setpoint in MAV_FRAME_GLOBAL_RELATIVE_ALT_INT.

        Default: position (+yaw) only — identical to the sim env. With posvel
        enabled and a feed-forward velocity set, the target's NED velocity is
        added (VX/VY/VZ un-ignored) so a moving target is led."""
        if self.mav is None:
            return
        vx = vy = vz = 0.0
        vel_ignore = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        )
        if self._posvel and self._ff_vel is not None:
            vx, vy, vz = self._ff_vel
            vel_ignore = 0  # command velocity alongside position
        type_mask = (
            vel_ignore
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        yaw = 0.0
        if yaw_rad is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        else:
            yaw = float(yaw_rad)
        self.mav.mav.set_position_target_global_int_send(
            0,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(round(float(lat_deg) * 1e7)),
            int(round(float(lon_deg) * 1e7)),
            float(alt_rel_m),
            float(vx), float(vy), float(vz),
            0.0, 0.0, 0.0,
            yaw,
            0.0,
        )

    def send_velocity_ned(
        self,
        *,
        vn_mps: float,
        ve_mps: float,
        vd_mps: float,
        yaw_rad: float | None = None,
    ) -> None:
        """GUIDED velocity-only setpoint (LOCAL_NED). Used only by realign
        (never reached in this vision-free chase, but kept for fidelity)."""
        if self.mav is None:
            return
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        yaw = 0.0
        if yaw_rad is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        else:
            yaw = float(yaw_rad)
        self.mav.mav.set_position_target_local_ned_send(
            0,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0, 0.0, 0.0,
            float(vn_mps), float(ve_mps), float(vd_mps),
            0.0, 0.0, 0.0,
            yaw,
            0.0,
        )

    def set_guided_horizontal_speed_mps(
        self,
        speed_mps: float,
        *,
        min_interval_s: float = 0.10,
        min_delta_mps: float = 0.2,
    ) -> None:
        """Active GUIDED groundspeed via DO_CHANGE_SPEED (throttled)."""
        if self.mav is None:
            return
        spd = max(0.1, float(speed_mps))
        prev = float(self._wp_spd_mps)
        now = time.monotonic()
        last_t = float(self._wp_spd_cmd_t)
        min_dt = max(0.0, float(min_interval_s))
        min_dv = max(0.05, float(min_delta_mps))
        if prev >= 0.0 and abs(spd - prev) < min_dv:
            return
        if prev >= 0.0 and min_dt > 0.0 and (now - last_t) < min_dt:
            return
        self._wp_spd_mps = spd
        self._wp_spd_cmd_t = now
        # SPEED_TYPE_GROUNDSPEED = 1
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1.0,          # groundspeed
            float(spd),
            0, 0, 0, 0, 0,
        )
        if self._cmd_lat is not None:
            self._cmd_lat.mark_command(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, now)
        if self._verbose_speed:
            print(f"[MAV] DO_CHANGE_SPEED groundspeed={spd:.1f} m/s", flush=True)

    # ------------------------------------------------------------------ params
    def configure_guided_speed_limits(
        self,
        *,
        speed_mps: float = 20.0,
        speed_up_mps: float = 5.0,
        speed_dn_mps: float = 3.0,
        accel_mps2: float = 4.0,
    ) -> None:
        """Raise WP_SPD/UP/DN/ACC (ArduCopter 4.7+ SI units) then kick a
        DO_CHANGE_SPEED. WP_SPD is the ceiling; runtime speed is DO_CHANGE_SPEED.
        Call once after GUIDED is entered."""
        spd = max(0.1, float(speed_mps))
        up = max(0.1, float(speed_up_mps))
        dn = max(0.1, float(speed_dn_mps))
        accel = max(0.5, float(accel_mps2))
        for name, value in (
            ("WP_SPD", spd),
            ("WP_SPD_UP", up),
            ("WP_SPD_DN", dn),
            ("WP_ACC", accel),
        ):
            self._set_param(name, float(value), wait=False)
        self._wp_spd_mps = -1.0  # force the DO_CHANGE_SPEED below
        self.set_guided_horizontal_speed_mps(spd)
        print(
            f"[MAV] GUIDED speed limits: WP_SPD={spd:.1f} m/s "
            f"UP={up:.1f} DN={dn:.1f} ACC={accel:.1f} m/s^2",
            flush=True,
        )

    def _set_param(self, name: str, value: float, *, wait: bool = False,
                   timeout: float = 2.0) -> bool:
        if self.mav.target_system == 0:
            raise RuntimeError("MAVLink target_system=0 - reconnect pymavlink first")
        value_f = float(value)
        self.mav.mav.param_set_send(
            self.mav.target_system, self.mav.target_component,
            name.encode("utf-8"),
            value_f,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        if not wait:
            if self._verbose_param:
                print(f"[MAV] param {name} = {value_f}", flush=True)
            return True
        self.mav.mav.param_request_read_send(
            self.mav.target_system, self.mav.target_component,
            name.encode("utf-8"), -1,
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", errors="ignore")
            pid = pid.rstrip("\x00")
            if pid != name:
                continue
            ok = abs(float(msg.param_value) - value_f) <= max(0.05, abs(value_f) * 0.05)
            status = "OK" if ok else f"got {msg.param_value}"
            print(f"[MAV] param {name} = {value_f} ({status})", flush=True)
            return ok
        print(f"[MAV] WARNING: param {name} = {value_f} (no ACK within {timeout:.0f}s)",
              flush=True)
        return False

    def _get_param(self, name: str, *, timeout: float = 2.0) -> float | None:
        if self.mav is None or self.mav.target_system == 0:
            return None
        self.mav.mav.param_request_read_send(
            self.mav.target_system, self.mav.target_component,
            name.encode("utf-8"), -1,
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", errors="ignore")
            pid = pid.rstrip("\x00")
            if pid != name:
                continue
            return float(msg.param_value)
        return None

    def verify_wp_spd(self, want_mps: float, *, timeout: float = 3.0) -> None:
        """Read WP_SPD back and warn if the ceiling did not take (lossy link /
        wrong firmware param name). Non-fatal; DO_CHANGE_SPEED still applies."""
        got = self._get_param("WP_SPD", timeout=timeout)
        if got is None:
            print("[MAV] WP_SPD read-back timed out (lossy link?) - "
                  "DO_CHANGE_SPEED still drives active speed.", flush=True)
            return
        if got + 1e-6 < want_mps:
            print(f"[MAV] WARNING: WP_SPD={got:.1f} < requested {want_mps:.1f} m/s. "
                  f"Position-target speed ceiling not raised; the vehicle may cap "
                  f"below {want_mps:.0f} m/s. Check the param name for this "
                  f"firmware (WP_SPD vs WPNAV_SPEED).", flush=True)
        else:
            print(f"[MAV] WP_SPD ceiling confirmed at {got:.1f} m/s.", flush=True)
