"""Convert HCAir body-rate commands to ArduPilot ACRO RC PWM."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import numpy as np

from fpv_gate_il.teacher.ardupilot_acro_adapter import (
    ArduPilotAcroRateAdapter,
    PhysicalRateCommand,
    PhysicalRateLimits,
    RateCommandError,
    RCAxisCal,
    TemporalRateQuantizer,
    VehicleRateConfig,
    clamp,
    load_adapter_from_vehicle,
)


def _rc_axis(raw: dict[str, Any], name: str) -> RCAxisCal:
    node = raw[name]
    return RCAxisCal(
        min_pwm=int(node["min_pwm"]),
        trim_pwm=int(node["trim_pwm"]),
        max_pwm=int(node["max_pwm"]),
        deadzone=max(0, int(node.get("deadzone", 0))),
        reversed=bool(node.get("reversed", False)),
    )


def vehicle_rate_config_from_dict(acro: dict[str, Any]) -> VehicleRateConfig:
    rc = acro["rc"]
    ch = acro["channels"]
    return VehicleRateConfig(
        rp_rate_deg_s=float(acro["rp_rate_deg_s"]),
        rp_expo=float(acro.get("rp_expo", 0.0)),
        yaw_rate_deg_s=float(acro["yaw_rate_deg_s"]),
        yaw_expo=float(acro.get("yaw_expo", 0.0)),
        roll_cal=_rc_axis(rc, "roll"),
        pitch_cal=_rc_axis(rc, "pitch"),
        throttle_cal=_rc_axis(rc, "throttle"),
        yaw_cal=_rc_axis(rc, "yaw"),
        roll_ch=int(ch["roll"]),
        pitch_ch=int(ch["pitch"]),
        throttle_ch=int(ch["throttle"]),
        yaw_ch=int(ch["yaw"]),
        thr_mid_curve=float(acro.get("thr_mid_curve", 0.5)),
    )


def physical_rate_limits_from_dict(acro: dict[str, Any]) -> PhysicalRateLimits:
    lim = acro.get("rate_limits", {})
    return PhysicalRateLimits(
        p_max=float(lim.get("p_max_rad_s", 1.20)),
        q_max=float(lim.get("q_max_rad_s", 1.20)),
        r_max=float(lim.get("r_max_rad_s", 1.00)),
        max_rp_stick_radius=float(lim.get("max_rp_stick_radius", 0.85)),
    )


def _log_adapter_config(source: str, adapter: ArduPilotAcroRateAdapter, *, note: str = "") -> None:
    c = adapter.config
    if source == "vehicle":
        label = "vehicle (MAVLink params)"
    else:
        label = "yaml (fallback)"
    suffix = f" — {note}" if note else ""
    print(f"[ACRO] Rate→PWM adapter config source: {label}{suffix}")
    print(
        f"[ACRO]   rates: RP={c.rp_rate_deg_s}°/s expo={c.rp_expo} "
        f"Y={c.yaw_rate_deg_s}°/s expo={c.yaw_expo} thr_mid={c.thr_mid_curve:.3f}"
    )
    print(
        f"[ACRO]   channels: roll={c.roll_ch} pitch={c.pitch_ch} "
        f"throttle={c.throttle_ch} yaw={c.yaw_ch}"
    )
    for axis_name, cal in (
        ("roll", c.roll_cal),
        ("pitch", c.pitch_cal),
        ("throttle", c.throttle_cal),
        ("yaw", c.yaw_cal),
    ):
        print(
            f"[ACRO]   rc_{axis_name}: min={cal.min_pwm} trim={cal.trim_pwm} "
            f"max={cal.max_pwm} dz={cal.deadzone} rev={cal.reversed}"
        )
    lim = adapter.limits
    print(
        f"[ACRO]   rate_limits (yaml policy): p={lim.p_max} q={lim.q_max} "
        f"r={lim.r_max} max_rp_radius={lim.max_rp_stick_radius}"
    )


class HCAirAcroConverter:
    """Map teacher body rates + thrust_cmd to integer RC PWM."""

    def __init__(self, cfg: dict[str, Any]) -> None:
        acro = cfg["acro"]
        plat = cfg["platform"]
        self._acro_cfg = acro
        self.hover_thrust_teacher = float(plat["hover_thrust"])
        self.hover_thrust_firmware = float(acro.get("hover_thrust_firmware", 0.33))
        self.thrust_scale = float(acro.get("thrust_scale", 0.50))

        self.config_source = "yaml"
        self.map_error_count = 0
        self.map_hold_count = 0
        self.map_neutral_count = 0
        self.last_map_error = ""
        self._vehicle_load_failed = False
        self._bind_yaml_adapter()
        self._last_mapped = None
        _log_adapter_config("yaml", self.adapter, note="initial")

    def _bind_yaml_adapter(self) -> None:
        self.adapter = ArduPilotAcroRateAdapter(
            vehicle_rate_config_from_dict(self._acro_cfg),
            limits=physical_rate_limits_from_dict(self._acro_cfg),
        )
        self._quantizer = TemporalRateQuantizer(self.adapter)

    def try_load_from_vehicle(
        self,
        master,
        *,
        attempts: int = 3,
        timeout_per_attempt: float = 3.0,
    ) -> str:
        """Load ACRO/RC params from the connected vehicle; keep yaml on failure.

        HUNTER NOTU (31 Agu 2026) — attempts/timeout DISARI ACILDI.
        Upstream sabit 3 deneme x 3.0 s kullaniyordu: yani her parametre
        grubu icin 3 saniyede BIR sorup cevap bekliyor. HUNTER'in udpout
        linkinde bu YETERSIZ. guided_sender.read_param'in yorumunda yazili
        saha dersi: "MOT_THST_HOVER bir kere okunup sonraki calistirmada
        timeout verdi" -> istegi 0.6 saniyede bir TEKRARLAMAK zorunda
        kalinmisti. Ayni link, ayni sorun.

        Ustelik burada bedel daha agir: gruplardan BIRI eksik kalirsa
        istisna atilir ve TUM konfig yaml'a duser (okunan gruplar dahil),
        `_vehicle_load_failed` yapiskan oldugu icin oturum boyunca bir daha
        DENENMEZ. Tek dusen paket = butun oturum yaml fallback'inde.
        """
        if self.config_source == "vehicle":
            return "vehicle"
        if self._vehicle_load_failed:
            return "yaml"
        if master is None:
            print("[ACRO] Vehicle param load skipped (no MAVLink), using yaml")
            return "yaml"

        print("[ACRO] Attempting rate→PWM config from vehicle (MAVLink params)...")
        try:
            limits = physical_rate_limits_from_dict(self._acro_cfg)
            adapter = load_adapter_from_vehicle(
                master,
                limits=limits,
                attempts=int(attempts),
                timeout_per_attempt=float(timeout_per_attempt),
            )
        except Exception as exc:
            self._vehicle_load_failed = True
            print(
                f"[ACRO] Vehicle param load failed "
                f"({type(exc).__name__}: {exc})"
            )
            print("[ACRO] Rate→PWM adapter: keeping yaml fallback")
            _log_adapter_config("yaml", self.adapter, note="fallback after vehicle error")
            return "yaml"

        self.adapter = adapter
        self._quantizer = TemporalRateQuantizer(adapter)
        self.config_source = "vehicle"
        _log_adapter_config("vehicle", self.adapter, note="loaded from FC")
        return "vehicle"

    @property
    def rc_channel_map(self) -> tuple[int, int, int, int]:
        c = self.adapter.config
        return (c.roll_ch, c.pitch_ch, c.throttle_ch, c.yaw_ch)

    def reset(self) -> None:
        # NOT: map_* sayaclari BILEREK sifirlanmiyor — oturum boyu birikir ki
        # bir pencerede yasanan redler sonraki pencerede de gorunsun.
        self._quantizer.reset()
        self._last_mapped = None

    def teacher_thrust_to_firmware(self, thrust_cmd: float) -> float:
        thrust = self.hover_thrust_firmware + (
            float(thrust_cmd) - self.hover_thrust_teacher
        ) * self.thrust_scale
        return float(clamp(thrust, 0.0, 1.0))

    def _clamp_command(self, body: np.ndarray) -> PhysicalRateCommand:
        lim = self.adapter.limits
        return PhysicalRateCommand(
            clamp(float(body[0]), -lim.p_max, lim.p_max),
            clamp(float(body[1]), -lim.q_max, lim.q_max),
            clamp(float(body[3]), -lim.r_max, lim.r_max),
        )

    # -----------------------------------------------------------------------
    # HUNTER EMNIYETI (31 Agu 2026) — upstream'de YOK, bilerek eklendi.
    #
    # Upstream akis:  quantizer.map_command  --RateCommandError-->
    #                 adapter.map_command
    # Ama adapter.map_command DA ayni hatayi atabiliyor: validate_command
    # icindeki dairesel stick yaricapi kontrolu (|xr,xp| > max_rp_stick_radius)
    # _clamp_command'in eksen-basi kelepcesinden GECER. roll ve pitch ayni anda
    # tavana yakinsa yaricap 0.85'i asar ve istisna yukari cikar.
    #
    # Ucusta compute_action icinden cikan bir istisna = KONTROL DONGUSU OLUR.
    # Bu yuzden son catkiya dusuyoruz: (1) son gecerli mapping, (2) o da yoksa
    # notr komut (sifir rate + istenen throttle). Her dususte sayac artar ve
    # last_mapping_debug ile CSV'ye yazilir — sessiz kalmaz.
    # -----------------------------------------------------------------------
    def _map_with_fallback(self, command: PhysicalRateCommand, throttle_pwm: int):
        try:
            return self._quantizer.map_command(command, throttle_pwm)
        except RateCommandError:
            pass
        try:
            return self.adapter.map_command(command, throttle_pwm)
        except RateCommandError as exc:
            self.map_error_count += 1
            self.last_map_error = f"{type(exc).__name__}: {exc}"
            if self.map_error_count <= 5 or self.map_error_count % 100 == 0:
                print(f"[ACRO] map_command reddi #{self.map_error_count}: {exc}")
        if self._last_mapped is not None:
            self.map_hold_count += 1
            return replace(self._last_mapped, throttle_pwm=int(throttle_pwm))
        self.map_neutral_count += 1
        return self.adapter.neutral_command(int(throttle_pwm))

    def body_rates_to_rc_pwm(self, body_action: np.ndarray) -> np.ndarray:
        """body_action: [roll, pitch, thrust_cmd, yaw] rad/s + teacher thrust."""
        body = np.asarray(body_action, dtype=np.float64)
        thrust_cmd = float(body[2])
        firmware_thrust = self.teacher_thrust_to_firmware(thrust_cmd)
        throttle_pwm = self.adapter.throttle_pwm_for_thrust(firmware_thrust)
        command = self._clamp_command(body)

        mapped = self._map_with_fallback(command, throttle_pwm)

        self._last_mapped = mapped
        return np.array(
            [
                float(mapped.roll_pwm),
                float(mapped.pitch_pwm),
                float(mapped.throttle_pwm),
                float(mapped.yaw_pwm),
            ],
            dtype=np.float32,
        )

    # -----------------------------------------------------------------------
    # HUNTER GIRIS NOKTASI (31 Agu 2026) — acro_sender.py bunu cagirir.
    #
    # Upstream body_rates_to_rc_pwm() thrust'i teacher_thrust_to_firmware()
    # ile ceviriyor (hover_thrust_firmware + dev*thrust_scale). HUNTER'da
    # dikey kanal guided_sender'in SAHADA DOGRULANMIS yolundan geciyor
    # (hover_motor + dev*thrust_gain, --scale-thrust/--thrust-dev/--thrust-cap
    # governor'lari, ve ACRO'da angle boost KAPALI oldugu icin tilt telafisi).
    # Bu yuzden burada firmware_thrust HAZIR gelir, tekrar cevrilmez.
    #
    # acro.hover_thrust_firmware / acro.thrust_scale bu yolda KULLANILMAZ;
    # sadece teacher-ici output.mode=acro_rc (offline A/B) yolunda gecerlidir.
    # -----------------------------------------------------------------------
    def rates_to_rc_pwm(self, p, q, r, firmware_thrust):
        """(p, q, r) rad/s + hazir firmware thrust (0..1) -> 4 tam sayi PWM."""
        throttle_pwm = self.adapter.throttle_pwm_for_thrust(float(firmware_thrust))
        command = self._clamp_command(
            np.array([float(p), float(q), 0.0, float(r)], dtype=np.float64)
        )
        mapped = self._map_with_fallback(command, throttle_pwm)
        self._last_mapped = mapped
        return (
            int(mapped.roll_pwm),
            int(mapped.pitch_pwm),
            int(mapped.throttle_pwm),
            int(mapped.yaw_pwm),
        )

    def last_mapping_debug(self) -> dict[str, float]:
        mapped = self._last_mapped
        if mapped is None:
            return {
                "acro_config_source": self.config_source,
                "acro_map_err": float(self.map_error_count),
                "acro_map_hold": float(self.map_hold_count),
                "acro_map_neutral": float(self.map_neutral_count),
            }
        rep = mapped.represented
        return {
            "acro_config_source": self.config_source,
            "acro_roll_pwm": float(mapped.roll_pwm),
            "acro_pitch_pwm": float(mapped.pitch_pwm),
            "acro_throttle_pwm": float(mapped.throttle_pwm),
            "acro_yaw_pwm": float(mapped.yaw_pwm),
            "acro_rep_p_rad_s": float(rep.p),
            "acro_rep_q_rad_s": float(rep.q),
            "acro_rep_r_rad_s": float(rep.r),
            "acro_quant_err_p": float(mapped.quantization_error_p),
            "acro_quant_err_q": float(mapped.quantization_error_q),
            "acro_quant_err_r": float(mapped.quantization_error_r),
            "acro_rp_stick_radius": float(mapped.rp_stick_radius),
            "acro_firmware_thrust": float(
                self.adapter.thrust_for_throttle_pwm(mapped.throttle_pwm)
            ),
            "acro_map_err": float(self.map_error_count),
            "acro_map_hold": float(self.map_hold_count),
            "acro_map_neutral": float(self.map_neutral_count),
        }
