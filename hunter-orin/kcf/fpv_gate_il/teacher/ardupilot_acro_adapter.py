#!/usr/bin/env python3
"""
ArduPilot Copter ACRO physical body-rate -> RC PWM adapter.

The public interface is physical body rate in rad/s:
    (p, q, r) [rad/s] -> integer RC PWM -> RC_CHANNELS_OVERRIDE

Design goals
------------
* The caller never commands "percent stick"; it commands physical body rate.
* ArduPilot ACRO/RC parameters are read from the connected vehicle.
* Missing or invalid parameters are fatal in strict mode.
* Every axis uses an exhaustive integer-PWM lookup table, so the selected
  PWM is the global nearest integer PWM for that axis.
* Roll/pitch are locally refined together using the complete ArduPilot
  forward path, including the circular stick limit.
* The default policy rejects commands outside a conservative common physical
  envelope instead of silently changing the requested rate.
* RCMAP_* is respected when packing MAVLink RC_CHANNELS_OVERRIDE.

ArduPilot source path represented here:
    RC_Channel::norm_input_dz()
      -> roll/pitch circular limit
      -> AP_Math::input_expo()
      -> ACRO_RP_RATE / ACRO_Y_RATE

This module intentionally does not implement IBVS, navigation, position hold,
or altitude hold.
"""

from __future__ import annotations

import bisect
import math
import statistics
import time
from dataclasses import asdict, dataclass, replace
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

try:
    from pymavlink import mavutil
except ImportError:  # Pure mapping/unit tests do not require MAVLink.
    mavutil = None


IGNORE_CHANNEL = 65535
RELEASE_CHANNEL = 0


def _require_pymavlink() -> None:
    if mavutil is None:
        raise RuntimeError(
            "pymavlink is required for MAVLink/vehicle operations. "
            "Install it with: python3 -m pip install pymavlink"
        )


class ParameterError(RuntimeError):
    """Required ArduPilot parameters were missing or invalid."""


class RateCommandError(ValueError):
    """A physical-rate command cannot be represented under the chosen policy."""


@dataclass(frozen=True)
class RCAxisCal:
    min_pwm: int
    trim_pwm: int
    max_pwm: int
    deadzone: int
    reversed: bool = False

    def validate(self, name: str = "axis") -> None:
        dz = max(0, int(self.deadzone))
        if not (800 <= self.min_pwm < self.trim_pwm - dz):
            raise ParameterError(
                f"{name}: require 800 <= MIN < TRIM-DZ; got "
                f"{self.min_pwm}, {self.trim_pwm}, DZ={dz}"
            )
        if not (self.trim_pwm + dz < self.max_pwm <= 2200):
            raise ParameterError(
                f"{name}: require TRIM+DZ < MAX <= 2200; got "
                f"{self.trim_pwm}, {self.max_pwm}, DZ={dz}"
            )


@dataclass(frozen=True)
class PhysicalRateLimits:
    """Fleet-level physical limits. Values are absolute rad/s."""

    p_max: float = 1.20
    q_max: float = 1.20
    r_max: float = 1.00
    # Reject commands whose continuous ArduPilot roll/pitch stick radius
    # exceeds this value. Staying below 1 avoids firmware circular saturation
    # and makes the same physical target portable across vehicles.
    max_rp_stick_radius: float = 0.85

    def validate(self) -> None:
        if min(self.p_max, self.q_max, self.r_max) <= 0.0:
            raise ValueError("All physical rate limits must be positive")
        if not (0.1 <= self.max_rp_stick_radius < 1.0):
            raise ValueError("max_rp_stick_radius must be in [0.1, 1.0)")


@dataclass(frozen=True)
class PhysicalRateCommand:
    """Firmware-independent body-rate command."""

    p: float  # roll rate, rad/s
    q: float  # pitch rate, rad/s
    r: float  # yaw rate, rad/s

    def validate_finite(self) -> None:
        if not all(math.isfinite(v) for v in (self.p, self.q, self.r)):
            raise RateCommandError(f"Non-finite body-rate command: {self}")


@dataclass(frozen=True)
class MappedRateCommand:
    requested: PhysicalRateCommand
    represented: PhysicalRateCommand
    roll_pwm: int
    pitch_pwm: int
    yaw_pwm: int
    throttle_pwm: int
    roll_stick: float
    pitch_stick: float
    yaw_stick: float
    rp_stick_radius: float
    quantization_error_p: float
    quantization_error_q: float
    quantization_error_r: float

    def as_dict(self) -> Dict[str, object]:
        data = asdict(self)
        return data


@dataclass(frozen=True)
class VehicleRateConfig:
    rp_rate_deg_s: float
    rp_expo: float
    yaw_rate_deg_s: float
    yaw_expo: float
    roll_cal: RCAxisCal
    pitch_cal: RCAxisCal
    throttle_cal: RCAxisCal
    yaw_cal: RCAxisCal
    roll_ch: int
    pitch_ch: int
    throttle_ch: int
    yaw_ch: int
    # Shape parameter of the pilot throttle curve: ACRO_THR_MID when positive,
    # otherwise MOT_THST_HOVER clamped the way get_throttle_hover() clamps it.
    # This is NOT "the thrust that hovers this vehicle" -- see PilotThrottleMap
    # and HoverThrustObserver. Conflating the two is worth a factor of two on a
    # vehicle whose MOT_THST_HOVER was never learned. 0.5 means "unknown", and
    # makes the curve linear.
    thr_mid_curve: float = 0.5

    def validate(self) -> None:
        if self.rp_rate_deg_s <= 0.0 or self.yaw_rate_deg_s <= 0.0:
            raise ParameterError("ACRO rate parameters must be positive")
        if not (-1.0 <= self.rp_expo <= 1.0):
            raise ParameterError(f"Invalid ACRO_RP_EXPO={self.rp_expo}")
        if not (-1.0 <= self.yaw_expo <= 1.0):
            raise ParameterError(f"Invalid ACRO_Y_EXPO={self.yaw_expo}")

        self.roll_cal.validate("roll")
        self.pitch_cal.validate("pitch")
        self.throttle_cal.validate("throttle")
        self.yaw_cal.validate("yaw")

        if not (0.0 < self.thr_mid_curve < 1.0):
            raise ParameterError(f"Invalid thr_mid_curve={self.thr_mid_curve}")

        channels = (self.roll_ch, self.pitch_ch, self.throttle_ch, self.yaw_ch)
        if any(ch < 1 or ch > 8 for ch in channels):
            raise ParameterError(f"RCMAP channels must be 1..8, got {channels}")
        if len(set(channels)) != 4:
            raise ParameterError(f"RCMAP channels must be unique, got {channels}")


# ---------------------------------------------------------------------------
# Source-equivalent ArduPilot forward mathematics
# ---------------------------------------------------------------------------

def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def input_expo(stick: float, expo: float) -> float:
    """AP_Math::input_expo source-equivalent formula."""
    stick = clamp(float(stick), -1.0, 1.0)
    if expo < 0.95:
        return (1.0 - expo) * stick / (1.0 - expo * abs(stick))
    return stick


def inverse_expo(rate_fraction: float, expo: float) -> float:
    """Analytic inverse of input_expo on [-1, 1]."""
    f = clamp(float(rate_fraction), -1.0, 1.0)
    if expo < 0.95:
        denom = 1.0 - expo + expo * abs(f)
        if abs(denom) < 1e-12:
            raise RateCommandError("inverse_expo singular denominator")
        return f / denom
    return f


def norm_input_dz(pwm: int | float, cal: RCAxisCal) -> float:
    """RC_Channel::norm_input_dz source-equivalent formula."""
    radio_in = float(pwm)
    dz_min = cal.trim_pwm - max(0, cal.deadzone)
    dz_max = cal.trim_pwm + max(0, cal.deadzone)
    reverse_mul = -1.0 if cal.reversed else 1.0

    if radio_in < dz_min and dz_min > cal.min_pwm:
        value = reverse_mul * (radio_in - dz_min) / (dz_min - cal.min_pwm)
    elif radio_in > dz_max and cal.max_pwm > dz_max:
        value = reverse_mul * (radio_in - dz_max) / (cal.max_pwm - dz_max)
    else:
        value = 0.0

    return clamp(value, -1.0, 1.0)


def stick_to_pwm_float(stick: float, cal: RCAxisCal) -> float:
    """Continuous inverse of norm_input_dz, before integer quantization."""
    x = clamp(float(stick), -1.0, 1.0)
    if cal.reversed:
        x = -x

    if abs(x) < 1e-15:
        return float(cal.trim_pwm)

    dz = max(0, cal.deadzone)
    if x > 0.0:
        start = cal.trim_pwm + dz
        return start + x * (cal.max_pwm - start)

    start = cal.trim_pwm - dz
    return start + x * (start - cal.min_pwm)


def axis_forward_rate(
    pwm: int | float,
    max_rate_deg_s: float,
    expo: float,
    cal: RCAxisCal,
) -> float:
    stick = norm_input_dz(pwm, cal)
    return math.radians(max_rate_deg_s) * input_expo(stick, expo)


def rp_forward_rate(
    roll_pwm: int,
    pitch_pwm: int,
    config: VehicleRateConfig,
) -> Tuple[float, float, float, float]:
    """
    Full ArduPilot roll/pitch forward path.

    Returns:
        represented_roll_rate, represented_pitch_rate,
        post-circular roll stick, post-circular pitch stick
    """
    xr = norm_input_dz(roll_pwm, config.roll_cal)
    xp = norm_input_dz(pitch_pwm, config.pitch_cal)

    radius = math.hypot(xr, xp)
    if radius > 1.0:
        xr /= radius
        xp /= radius

    scale = math.radians(config.rp_rate_deg_s)
    return (
        scale * input_expo(xr, config.rp_expo),
        scale * input_expo(xp, config.rp_expo),
        xr,
        xp,
    )


class IntegerAxisMap:
    """
    Exhaustive map over every valid integer PWM.

    nearest_pwm() is a global nearest-neighbour search on the integer grid for
    the single-axis forward map.
    """

    def __init__(
        self,
        max_rate_deg_s: float,
        expo: float,
        cal: RCAxisCal,
    ) -> None:
        self.max_rate_deg_s = float(max_rate_deg_s)
        self.expo = float(expo)
        self.cal = cal

        # Multiple integer PWMs can produce exactly the same represented
        # rate (especially every PWM inside the deadzone produces zero).
        # Keep the PWM closest to TRIM for each distinct represented rate.
        best_by_rate: Dict[float, int] = {}
        for pwm in range(math.ceil(cal.min_pwm), math.floor(cal.max_pwm) + 1):
            represented = axis_forward_rate(
                pwm, max_rate_deg_s, expo, cal
            )
            previous = best_by_rate.get(represented)
            if previous is None or (
                abs(pwm - cal.trim_pwm),
                pwm,
            ) < (
                abs(previous - cal.trim_pwm),
                previous,
            ):
                best_by_rate[represented] = pwm

        entries = sorted(
            ((rate, pwm) for rate, pwm in best_by_rate.items()),
            key=lambda item: item[0],
        )
        self.entries = entries
        self.rates = [entry[0] for entry in entries]

    def nearest_pwm(self, target_rate: float) -> Tuple[int, float]:
        if abs(target_rate) < 1e-15:
            return int(self.cal.trim_pwm), 0.0

        idx = bisect.bisect_left(self.rates, target_rate)
        lo = max(0, idx - 2)
        hi = min(len(self.entries), idx + 3)
        candidates = self.entries[lo:hi]
        represented, pwm = min(
            candidates,
            key=lambda item: (
                abs(item[0] - target_rate),
                abs(item[1] - self.cal.trim_pwm),
                item[1],
            ),
        )
        return pwm, represented


def pilot_thrust_for_pwm(pwm: int | float, cal: RCAxisCal, thr_mid: float) -> float:
    """Normalised thrust demand the firmware derives from a throttle PWM.

    Transcribes RC_Channel::pwm_to_range_dz -> RC_Channel::get_control_mid ->
    Mode::get_pilot_desired_throttle, integer truncations included. RC3 is a
    RANGE channel, so RC3_TRIM is deliberately unused and the deadzone sits at
    the bottom of the range, not around trim.

    The cubic's coefficient is derived from thr_mid, which is why mid stick
    lands exactly on thr_mid: T(0.5) = 0.5 - 0.375*e, and e = -(thr_mid-0.5)/0.375.
    That identity holds only while e is unsaturated, i.e. thr_mid in
    [0.125, 0.6875] -- the same window get_throttle_hover() clamps to.
    """

    raw = clamp(float(pwm), float(cal.min_pwm), float(cal.max_pwm))
    if cal.reversed:
        raw = cal.max_pwm - (raw - cal.min_pwm)

    trim_low = cal.min_pwm + cal.deadzone
    span = cal.max_pwm - trim_low
    if span <= 0:
        raise ParameterError("Throttle deadzone leaves no usable range")

    control_in = int(1000.0 * (raw - trim_low) / span) if raw > trim_low else 0
    control_in = int(clamp(control_in, 0, 1000))

    mid_stick = (1000 * ((cal.min_pwm + cal.max_pwm) // 2 - trim_low)) // span
    if mid_stick <= 0:
        mid_stick = 500

    if control_in < mid_stick:
        stick = control_in * 0.5 / mid_stick
    else:
        stick = 0.5 + (control_in - mid_stick) * 0.5 / (1000 - mid_stick)

    expo = clamp(-(float(thr_mid) - 0.5) / 0.375, -0.5, 1.0)
    return stick * (1.0 - expo) + expo * stick ** 3


def gain_corrected_specific_thrust(s: float, gain: float) -> float:
    """Scale the *deviation from hover*, never the command itself.

    A hover measurement fixes one point on the throttle-to-thrust curve, which
    is enough to absorb a change of mass and nothing else: `s = u/u_hover`
    still assumes thrust is proportional to `u`, and that only holds while the
    firmware's assumed propeller curve matches the propeller fitted. Measured
    across two Gazebo airframes differing in mass, propeller area and RC3
    calibration, the same command produced accelerations 0.80 m/s2 apart.

    One more number closes most of it: the local gain between commanded and
    delivered specific thrust. Applying it to `s - 1` rather than to `s` keeps
    the hover fixed point exact -- `s = 1` maps to 1 for every gain, so a
    mis-identified gain can never command a vehicle to leave its own hover.

    Held out honestly: identifying the gain on descending cells alone and
    applying it to climbing cells it had never seen took the fleet spread from
    0.80 to 0.25-0.28 m/s2.
    """

    g = float(gain)
    if not math.isfinite(g) or g <= 0.0:
        raise RateCommandError(f"Invalid thrust gain {gain!r}")
    return 1.0 + (float(s) - 1.0) / g


class PilotThrottleMap:
    """Exhaustive integer-PWM map for the pilot throttle channel.

    The throttle-axis counterpart of IntegerAxisMap. The forward map is a
    cubic on top of an integer-truncated range conversion, so the inverse is a
    nearest-neighbour lookup over the achievable set rather than an inversion.
    """

    def __init__(self, cal: RCAxisCal, thr_mid: float) -> None:
        self.cal = cal
        self.thr_mid = float(thr_mid)

        mid_pwm = (cal.min_pwm + cal.max_pwm) // 2
        best_by_thrust: Dict[float, int] = {}
        for pwm in range(math.ceil(cal.min_pwm), math.floor(cal.max_pwm) + 1):
            thrust = pilot_thrust_for_pwm(pwm, cal, self.thr_mid)
            previous = best_by_thrust.get(thrust)
            if previous is None or (abs(pwm - mid_pwm), pwm) < (
                abs(previous - mid_pwm),
                previous,
            ):
                best_by_thrust[thrust] = pwm

        entries = sorted(best_by_thrust.items(), key=lambda item: item[0])
        self.entries = entries
        self.thrusts = [entry[0] for entry in entries]

    def thrust_for_pwm(self, pwm: int | float) -> float:
        return pilot_thrust_for_pwm(pwm, self.cal, self.thr_mid)

    def nearest_pwm(self, target_thrust: float) -> Tuple[int, float]:
        """Integer PWM whose represented thrust is closest to the target."""

        idx = bisect.bisect_left(self.thrusts, float(target_thrust))
        lo = max(0, idx - 2)
        hi = min(len(self.entries), idx + 3)
        thrust, pwm = min(
            self.entries[lo:hi],
            key=lambda item: (abs(item[0] - float(target_thrust)), item[1]),
        )
        return pwm, thrust


class ArduPilotAcroRateAdapter:
    """
    Firmware adapter:
        physical body rate [rad/s] -> ArduPilot ACRO integer RC override.
    """

    def __init__(
        self,
        config: VehicleRateConfig,
        limits: Optional[PhysicalRateLimits] = None,
        joint_search_radius_pwm: int = 4,
    ) -> None:
        config.validate()
        self.config = config
        self.limits = limits or PhysicalRateLimits()
        self.limits.validate()
        self.joint_search_radius_pwm = max(1, int(joint_search_radius_pwm))

        self._roll_map = IntegerAxisMap(
            config.rp_rate_deg_s, config.rp_expo, config.roll_cal
        )
        self._pitch_map = IntegerAxisMap(
            config.rp_rate_deg_s, config.rp_expo, config.pitch_cal
        )
        self._yaw_map = IntegerAxisMap(
            config.yaw_rate_deg_s, config.yaw_expo, config.yaw_cal
        )

    @property
    def firmware_max_rates(self) -> PhysicalRateCommand:
        return PhysicalRateCommand(
            math.radians(self.config.rp_rate_deg_s),
            math.radians(self.config.rp_rate_deg_s),
            math.radians(self.config.yaw_rate_deg_s),
        )

    def capability_report(self) -> Dict[str, float]:
        """
        Conservative physical capability under max_rp_stick_radius.

        These values are useful when selecting one common fleet envelope.
        """
        radius = self.limits.max_rp_stick_radius
        rp_axis = (
            math.radians(self.config.rp_rate_deg_s)
            * abs(input_expo(radius, self.config.rp_expo))
        )
        return {
            "firmware_roll_max_rad_s": math.radians(self.config.rp_rate_deg_s),
            "firmware_pitch_max_rad_s": math.radians(self.config.rp_rate_deg_s),
            "firmware_yaw_max_rad_s": math.radians(self.config.yaw_rate_deg_s),
            "conservative_single_rp_rad_s_at_radius": rp_axis,
            "configured_common_p_max_rad_s": self.limits.p_max,
            "configured_common_q_max_rad_s": self.limits.q_max,
            "configured_common_r_max_rad_s": self.limits.r_max,
            "max_rp_stick_radius": radius,
        }

    def _continuous_sticks(
        self, command: PhysicalRateCommand
    ) -> Tuple[float, float, float]:
        rp_max = math.radians(self.config.rp_rate_deg_s)
        yaw_max = math.radians(self.config.yaw_rate_deg_s)

        xr = inverse_expo(command.p / rp_max, self.config.rp_expo)
        xp = inverse_expo(command.q / rp_max, self.config.rp_expo)
        xy = inverse_expo(command.r / yaw_max, self.config.yaw_expo)
        return xr, xp, xy

    def validate_command(self, command: PhysicalRateCommand) -> None:
        command.validate_finite()

        if abs(command.p) > self.limits.p_max + 1e-12:
            raise RateCommandError(
                f"|p|={abs(command.p):.4f} exceeds common p limit "
                f"{self.limits.p_max:.4f} rad/s"
            )
        if abs(command.q) > self.limits.q_max + 1e-12:
            raise RateCommandError(
                f"|q|={abs(command.q):.4f} exceeds common q limit "
                f"{self.limits.q_max:.4f} rad/s"
            )
        if abs(command.r) > self.limits.r_max + 1e-12:
            raise RateCommandError(
                f"|r|={abs(command.r):.4f} exceeds common r limit "
                f"{self.limits.r_max:.4f} rad/s"
            )

        fw = self.firmware_max_rates
        if abs(command.p) > fw.p + 1e-12 or abs(command.q) > fw.q + 1e-12:
            raise RateCommandError("Roll/pitch command exceeds FC ACRO_RP_RATE")
        if abs(command.r) > fw.r + 1e-12:
            raise RateCommandError("Yaw command exceeds FC ACRO_Y_RATE")

        xr, xp, _ = self._continuous_sticks(command)
        radius = math.hypot(xr, xp)
        if radius > self.limits.max_rp_stick_radius + 1e-12:
            raise RateCommandError(
                f"Requested p/q requires ArduPilot stick radius {radius:.4f}, "
                f"above common safe radius "
                f"{self.limits.max_rp_stick_radius:.4f}. "
                "Reduce the physical fleet rate envelope."
            )

    def map_command(
        self,
        command: PhysicalRateCommand,
        throttle_pwm: int,
    ) -> MappedRateCommand:
        """
        Map one physical body-rate command without silently saturating it.

        represented contains the exact rate implied by the selected integer
        PWMs after the complete ArduPilot forward path.
        """
        self.validate_command(command)

        xr_des, xp_des, xy_des = self._continuous_sticks(command)

        # Exact global single-axis nearest PWMs.
        roll_pwm, _ = self._roll_map.nearest_pwm(command.p)
        pitch_pwm, _ = self._pitch_map.nearest_pwm(command.q)
        yaw_pwm, represented_yaw = self._yaw_map.nearest_pwm(command.r)

        # Joint roll/pitch refinement with the full circular forward path.
        rr = self.joint_search_radius_pwm
        roll_candidates = range(
            max(self.config.roll_cal.min_pwm, roll_pwm - rr),
            min(self.config.roll_cal.max_pwm, roll_pwm + rr) + 1,
        )
        pitch_candidates = range(
            max(self.config.pitch_cal.min_pwm, pitch_pwm - rr),
            min(self.config.pitch_cal.max_pwm, pitch_pwm + rr) + 1,
        )

        roll_ideal = stick_to_pwm_float(xr_des, self.config.roll_cal)
        pitch_ideal = stick_to_pwm_float(xp_des, self.config.pitch_cal)

        best = None
        for rpwm in roll_candidates:
            for ppwm in pitch_candidates:
                p_rep, q_rep, xr_rep, xp_rep = rp_forward_rate(
                    rpwm, ppwm, self.config
                )
                rate_error = (p_rep - command.p) ** 2 + (q_rep - command.q) ** 2
                tie = 1e-15 * (
                    (rpwm - roll_ideal) ** 2 + (ppwm - pitch_ideal) ** 2
                )
                key = (rate_error + tie, rpwm, ppwm)
                if best is None or key < best[0]:
                    best = (key, rpwm, ppwm, p_rep, q_rep, xr_rep, xp_rep)

        assert best is not None
        _, roll_pwm, pitch_pwm, represented_p, represented_q, xr_rep, xp_rep = best

        # Re-evaluate yaw from the selected integer PWM for clarity.
        represented_yaw = axis_forward_rate(
            yaw_pwm,
            self.config.yaw_rate_deg_s,
            self.config.yaw_expo,
            self.config.yaw_cal,
        )
        yaw_stick = norm_input_dz(yaw_pwm, self.config.yaw_cal)

        represented = PhysicalRateCommand(
            represented_p, represented_q, represented_yaw
        )
        return MappedRateCommand(
            requested=command,
            represented=represented,
            roll_pwm=int(roll_pwm),
            pitch_pwm=int(pitch_pwm),
            yaw_pwm=int(yaw_pwm),
            throttle_pwm=int(throttle_pwm),
            roll_stick=xr_rep,
            pitch_stick=xp_rep,
            yaw_stick=yaw_stick,
            rp_stick_radius=math.hypot(xr_rep, xp_rep),
            quantization_error_p=represented_p - command.p,
            quantization_error_q=represented_q - command.q,
            quantization_error_r=represented_yaw - command.r,
        )

    def pack_channels(self, mapped: MappedRateCommand) -> List[int]:
        channels = [IGNORE_CHANNEL] * 8
        assignments = (
            (self.config.roll_ch, mapped.roll_pwm),
            (self.config.pitch_ch, mapped.pitch_pwm),
            (self.config.throttle_ch, mapped.throttle_pwm),
            (self.config.yaw_ch, mapped.yaw_pwm),
        )
        for channel_1based, value in assignments:
            channels[channel_1based - 1] = int(value)
        return channels

    def send(self, master, mapped: MappedRateCommand) -> None:
        channels = self.pack_channels(mapped)
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels,
        )

    @staticmethod
    def release(master) -> None:
        # For channels 1..8, 0 releases an override. UINT16_MAX only ignores
        # the field and does not clear an existing override.
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *([RELEASE_CHANNEL] * 8),
        )

    def neutral_command(self, throttle_pwm: int) -> MappedRateCommand:
        return self.map_command(PhysicalRateCommand(0.0, 0.0, 0.0), throttle_pwm)

    def throttle_fraction_to_pwm(self, fraction: float) -> int:
        f = clamp(float(fraction), 0.0, 1.0)
        cal = self.config.throttle_cal
        return round(cal.min_pwm + f * (cal.max_pwm - cal.min_pwm))

    def hover_mid_pwm(self) -> int:
        """PWM at mid stick, where the firmware's own get_control_mid() sits.

        The firmware evaluates its mid at (RC3_MIN + RC3_MAX)/2, so ignoring
        RC3_DZ here is correct and holds for asymmetric endpoints too.

        What this PWM commands is thr_mid_curve, i.e. ACRO_THR_MID or
        MOT_THST_HOVER -- *not* the thrust that hovers this airframe at this
        mass. It is only a hover reference on a vehicle whose MOT_THST_HOVER
        has actually been learned. Use throttle_map() with a measured hover
        thrust when the distinction matters.
        """

        return self.throttle_fraction_to_pwm(0.5)

    def throttle_map(self) -> "PilotThrottleMap":
        """Exact pilot-throttle transcription and its integer inverse."""

        cached = getattr(self, "_throttle_map", None)
        if cached is None:
            cached = PilotThrottleMap(
                self.config.throttle_cal, self.config.thr_mid_curve
            )
            self._throttle_map = cached
        return cached

    def thrust_for_throttle_pwm(self, pwm: int | float) -> float:
        return self.throttle_map().thrust_for_pwm(pwm)

    def throttle_pwm_for_thrust(self, thrust: float) -> int:
        return self.throttle_map().nearest_pwm(thrust)[0]


class TemporalRateQuantizer:
    """Error-diffused integer-PWM synthesis in physical-rate space.

    MAVLink RC override fields are integer microseconds.  At low body rates,
    one PWM step can be larger than the fleet tracking tolerance.  Selecting
    only the nearest integer therefore forces a slow feedback controller to
    hunt between two adjacent values.  This stateful quantizer carries the
    instantaneous represented-rate error into the next 50 Hz command, so the
    time-average command converges to the requested physical rate without a
    vehicle-specific calibration table.

    The zero command is always exact trim and clears all carried error.  The
    carry is bounded, and a command near the common safety envelope falls
    back to the ordinary validated nearest mapping if adding the carry would
    leave that envelope.
    """

    def __init__(
        self,
        adapter: ArduPilotAcroRateAdapter,
        *,
        residual_limit_rad_s: float = math.radians(2.0),
    ) -> None:
        if (
            not math.isfinite(residual_limit_rad_s)
            or residual_limit_rad_s <= 0.0
        ):
            raise ValueError("temporal quantizer residual limit must be positive")
        self.adapter = adapter
        self.residual_limit_rad_s = float(residual_limit_rad_s)
        self.reset()

    def reset(self) -> None:
        self.residual = PhysicalRateCommand(0.0, 0.0, 0.0)

    def map_command(
        self,
        command: PhysicalRateCommand,
        throttle_pwm: int,
    ) -> MappedRateCommand:
        self.adapter.validate_command(command)
        if max(abs(command.p), abs(command.q), abs(command.r)) < 1e-15:
            self.reset()
            return self.adapter.neutral_command(throttle_pwm)

        zero_p = abs(command.p) < 1e-15
        zero_q = abs(command.q) < 1e-15
        zero_r = abs(command.r) < 1e-15
        carry = PhysicalRateCommand(
            0.0 if zero_p else self.residual.p,
            0.0 if zero_q else self.residual.q,
            0.0 if zero_r else self.residual.r,
        )
        adjusted = PhysicalRateCommand(
            command.p + carry.p,
            command.q + carry.q,
            command.r + carry.r,
        )
        try:
            mapped = self.adapter.map_command(adjusted, throttle_pwm)
        except RateCommandError:
            # The original command was already validated.  Never expand the
            # physical envelope merely to repay a quantization residual.
            mapped = self.adapter.map_command(command, throttle_pwm)

        limit = self.residual_limit_rad_s
        self.residual = PhysicalRateCommand(
            0.0 if zero_p else clamp(
                carry.p + command.p - mapped.represented.p,
                -limit,
                limit,
            ),
            0.0 if zero_q else clamp(
                carry.q + command.q - mapped.represented.q,
                -limit,
                limit,
            ),
            0.0 if zero_r else clamp(
                carry.r + command.r - mapped.represented.r,
                -limit,
                limit,
            ),
        )
        return replace(
            mapped,
            requested=command,
            quantization_error_p=mapped.represented.p - command.p,
            quantization_error_q=mapped.represented.q - command.q,
            quantization_error_r=mapped.represented.r - command.r,
        )


# ---------------------------------------------------------------------------
# Strict MAVLink parameter access
# ---------------------------------------------------------------------------

def _param_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="ignore")
    return value.rstrip("\x00")


def fetch_parameters_strict(
    master,
    names: Sequence[str],
    *,
    attempts: int = 3,
    timeout_per_attempt: float = 3.0,
) -> Dict[str, float]:
    pending = set(names)
    result: Dict[str, float] = {}

    for _attempt in range(max(1, attempts)):
        for name in sorted(pending):
            master.mav.param_request_read_send(
                master.target_system,
                master.target_component,
                name.encode("utf-8"),
                -1,
            )

        deadline = time.monotonic() + timeout_per_attempt
        while pending and time.monotonic() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is None:
                continue
            name = _param_name(msg)
            if name in pending:
                result[name] = float(msg.param_value)
                pending.remove(name)

        if not pending:
            break

    if pending:
        raise ParameterError(
            "Missing required parameters after retries: "
            + ", ".join(sorted(pending))
        )
    return result


def fetch_optional_parameter(
    master,
    name: str,
    *,
    timeout: float = 1.5,
) -> Optional[float]:
    try:
        return fetch_parameters_strict(
            master, [name], attempts=1, timeout_per_attempt=timeout
        )[name]
    except ParameterError:
        return None


def set_parameter_verified(
    master,
    name: str,
    value: float,
    *,
    attempts: int = 3,
    timeout_per_attempt: float = 2.0,
    tolerance: float = 1e-4,
) -> bool:
    _require_pymavlink()
    for _attempt in range(max(1, attempts)):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("utf-8"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + timeout_per_attempt
        while time.monotonic() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is None or _param_name(msg) != name:
                continue
            return abs(float(msg.param_value) - float(value)) <= tolerance
    return False


def save_parameters(master, names: Sequence[str]) -> Dict[str, float]:
    return fetch_parameters_strict(master, names)


def restore_parameters(master, saved: Mapping[str, float]) -> Dict[str, bool]:
    return {
        name: set_parameter_verified(master, name, value)
        for name, value in saved.items()
    }


def load_adapter_from_vehicle(
    master,
    *,
    limits: Optional[PhysicalRateLimits] = None,
    attempts: int = 3,
    timeout_per_attempt: float = 3.0,
) -> ArduPilotAcroRateAdapter:
    acro = fetch_parameters_strict(
        master,
        ["ACRO_RP_RATE", "ACRO_RP_EXPO", "ACRO_Y_RATE", "ACRO_Y_EXPO"],
        attempts=attempts,
        timeout_per_attempt=timeout_per_attempt,
    )
    rcmap = fetch_parameters_strict(
        master,
        ["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW"],
        attempts=attempts,
        timeout_per_attempt=timeout_per_attempt,
    )

    # ModeAcro::throttle_hover() overrides thr_mid with ACRO_THR_MID when it is
    # positive; otherwise the curve uses get_throttle_hover(), which clamps
    # MOT_THST_HOVER to [0.125, 0.6875]. Both are optional: a vehicle that
    # exposes neither gets the linear curve.
    acro_thr_mid = fetch_optional_parameter(master, "ACRO_THR_MID")
    mot_thst_hover = fetch_optional_parameter(master, "MOT_THST_HOVER")
    if acro_thr_mid is not None and float(acro_thr_mid) > 0.0:
        thr_mid_curve = clamp(float(acro_thr_mid), 0.125, 0.6875)
    elif mot_thst_hover is not None:
        thr_mid_curve = clamp(float(mot_thst_hover), 0.125, 0.6875)
    else:
        thr_mid_curve = 0.5

    roll_ch = int(round(rcmap["RCMAP_ROLL"]))
    pitch_ch = int(round(rcmap["RCMAP_PITCH"]))
    throttle_ch = int(round(rcmap["RCMAP_THROTTLE"]))
    yaw_ch = int(round(rcmap["RCMAP_YAW"]))

    unique_channels = sorted({roll_ch, pitch_ch, throttle_ch, yaw_ch})
    names: List[str] = []
    for channel in unique_channels:
        names.extend(
            [
                f"RC{channel}_MIN",
                f"RC{channel}_TRIM",
                f"RC{channel}_MAX",
                f"RC{channel}_DZ",
                f"RC{channel}_REVERSED",
            ]
        )

    rc = fetch_parameters_strict(
        master,
        names,
        attempts=attempts,
        timeout_per_attempt=timeout_per_attempt,
    )

    def axis(channel: int) -> RCAxisCal:
        return RCAxisCal(
            min_pwm=int(round(rc[f"RC{channel}_MIN"])),
            trim_pwm=int(round(rc[f"RC{channel}_TRIM"])),
            max_pwm=int(round(rc[f"RC{channel}_MAX"])),
            deadzone=max(0, int(round(rc[f"RC{channel}_DZ"]))),
            reversed=bool(int(round(rc[f"RC{channel}_REVERSED"]))),
        )

    config = VehicleRateConfig(
        rp_rate_deg_s=float(acro["ACRO_RP_RATE"]),
        rp_expo=float(acro["ACRO_RP_EXPO"]),
        yaw_rate_deg_s=float(acro["ACRO_Y_RATE"]),
        yaw_expo=float(acro["ACRO_Y_EXPO"]),
        roll_cal=axis(roll_ch),
        pitch_cal=axis(pitch_ch),
        throttle_cal=axis(throttle_ch),
        yaw_cal=axis(yaw_ch),
        roll_ch=roll_ch,
        pitch_ch=pitch_ch,
        throttle_ch=throttle_ch,
        yaw_ch=yaw_ch,
        thr_mid_curve=thr_mid_curve,
    )
    return ArduPilotAcroRateAdapter(config, limits=limits)


def request_message_interval(master, message_id: int, rate_hz: float) -> None:
    _require_pymavlink()
    interval_us = int(1_000_000 / max(1.0, float(rate_hz)))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        float(message_id),
        float(interval_us),
        0,
        0,
        0,
        0,
        0,
    )


def measure_hover_thrust(
    master,
    *,
    seconds: float = 3.0,
    minimum_samples: int = 20,
    plausible: Tuple[float, float] = (0.05, 0.90),
    maximum_climb_ms: float = 0.25,
    maximum_spread: float = 0.06,
) -> Optional[float]:
    """Normalised thrust this airframe is actually using to hold its hover.

    ATTITUDE_TARGET.thrust is attitude_control->get_throttle_in()
    (ArduCopter/GCS_MAVLink_Copter.cpp:90), so in a settled hover it is the
    mixer thrust demand that carries this vehicle at this mass. That is the one
    number the pilot-throttle curve cannot supply: MOT_THST_HOVER is only the
    curve's *shape* parameter, and on a vehicle where hover learning never ran
    it sits at the firmware clamp floor of 0.125 and means nothing physical.

    Call this in a steady, level, powered hover -- GUIDED or ALT_HOLD. It reads
    zero in any manual-throttle mode while disarmed, because ModeAcro and
    ModeStabilize force pilot_desired_throttle to 0 in the SHUT_DOWN spool
    state before set_throttle_out().

    Returns None instead of a guess when the sample is too small or the value
    is implausible, so the caller can fall back to the parameter-derived seed.
    """

    _require_pymavlink()
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET, 10.0)
    request_message_interval(
        master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0
    )

    samples: List[float] = []
    climb_rate: Optional[float] = None
    rejected = 0
    deadline = time.monotonic() + max(0.5, float(seconds))
    while time.monotonic() < deadline:
        message = master.recv_match(
            type=["ATTITUDE_TARGET", "GLOBAL_POSITION_INT"],
            blocking=True,
            timeout=0.5,
        )
        if message is None:
            continue
        if message.get_type() == "GLOBAL_POSITION_INT":
            climb_rate = -float(message.vz) / 100.0
            continue
        thrust = float(message.thrust)
        if not math.isfinite(thrust):
            continue
        # A climbing vehicle is not hovering, and its throttle is not the hover
        # throttle. Without this gate a heavier airframe -- which takes longer
        # to settle after takeoff -- reports the throttle it is climbing on, and
        # everything downstream is scaled by that error. Measured on a +30%
        # airframe it read 0.4065 where the vehicle actually hovered near 0.345,
        # which is 66 PWM.
        if climb_rate is None or abs(climb_rate) > maximum_climb_ms:
            rejected += 1
            continue
        samples.append(thrust)

    if len(samples) < minimum_samples:
        return None
    hover = statistics.median(samples)
    if not (plausible[0] <= hover <= plausible[1]):
        return None
    # A sample that is mostly spread is a vehicle that was still moving.
    if len(samples) >= 8:
        spread = max(samples) - min(samples)
        if spread > maximum_spread:
            return None
    return hover


def set_mode(master, mode_name: str, timeout: float = 5.0) -> bool:
    mode_map = master.mode_mapping()
    if not mode_map or mode_name not in mode_map:
        raise ValueError(f"Mode {mode_name!r} unavailable; modes={mode_map}")
    mode_id = mode_map[mode_name]
    master.set_mode(mode_id)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is not None and int(msg.custom_mode) == int(mode_id):
            return True
    return False


def arm(master, timeout: float = 12.0) -> bool:
    _require_pymavlink()
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is not None and (
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            return True
    return False


def takeoff(
    master,
    altitude_m: float,
    timeout: float = 75.0,
    tolerance_m: float = 3.0,
) -> bool:
    """Command a GUIDED takeoff and wait until the vehicle is up.

    Acceptance is an absolute margin, not a fraction of the target. ArduPilot
    settles well below the commanded altitude -- 18.27 m was measured for a
    20.0 m command, and 10.27 m for 12.0 m -- so a percentage threshold silently
    becomes unreachable as the target drops. The old 95% rule passed at 20 m
    and could never pass at 12 m.

    The shortfall is not constant either: a 30.0 m command settles at 26.92 m,
    which misses a 3.0 m margin by eight centimetres and fails the takeoff
    with nothing to show for it. Callers that climb higher than the 20 m the
    margin was measured at must widen it -- hence the parameter rather than a
    constant.
    """

    _require_pymavlink()
    def send_takeoff() -> None:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            float(altitude_m),
        )

    send_takeoff()
    deadline = time.monotonic() + timeout
    last_send = 0.0

    while time.monotonic() < deadline:
        msg = master.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5
        )
        now = time.monotonic()
        if msg is not None and msg.relative_alt / 1000.0 >= altitude_m - tolerance_m:
            return True
        if now - last_send > 3.0:
            send_takeoff()
            last_send = now
    return False
