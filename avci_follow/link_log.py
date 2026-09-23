#!/usr/bin/env python3
"""
Telemetry / latency logging for the two-radio GPS chase, built in from the start.

Three append-only CSVs per run (all under --log-dir, default ./logs):

  hedef_link_<stamp>.csv    per-message health of the TARGET radio
  chaser_link_<stamp>.csv   per-message health of the CHASER radio
  relay_lat_<stamp>.csv     per control-tick 3-leg latency budget

Per-message rows (one per received MAVLink message on that radio) carry the
message type, the gap since the previous message of the same type, and the
instantaneous rate. RADIO_STATUS / RADIO messages additionally log the SiK
link fields (rssi, remrssi, noise, remnoise, rxerrors, fixed, txbuf) — this is
how you find out whether a given radio (e.g. the B-cube v5 vs an X-Rock clone)
actually injects link telemetry, and how loaded the link is.

The relay latency CSV logs, each tick: the age of the newest target and chaser
fixes (the dominant transport latency), the planner compute time, the send
time, and the resulting engagement state — so a laggy field run can be
diagnosed offline afterwards.

Header carries run metadata as `# key=value` lines. Files flush periodically so
a mid-run crash still leaves a usable log. `enabled=False` (from --no-log) makes
every method a no-op.

Thread-safety: note_message() is called from BOTH Vehicle receive threads, so
all writes take a lock. note_relay() is called from the main loop.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path


# SiK RADIO_STATUS / RADIO field names, in the column order we log them.
_RADIO_FIELDS = ("rssi", "remrssi", "txbuf", "noise", "remnoise", "rxerrors", "fixed")

_LINK_HEADER = (
    "t_wall_iso,t_mono,type,src_sys,gap_ms,inst_hz,"
    + ",".join(_RADIO_FIELDS)
)
_RELAY_HEADER = (
    "t_wall_iso,t_mono,updated,target_age_ms,chaser_age_ms,"
    "compute_ms,send_ms,total_ms,behavior,follow_phase,"
    "v_cmd_mps,dist_m,sep_m,gate_alt_m,target_hz,chaser_hz"
)

# Command->response latency log. RTT = time from sending an ACK-eliciting
# command to receiving its COMMAND_ACK (round-trip over the chaser link, both
# ways + FC processing; clock-sync-free). spd_start = time from a speed command
# to the chaser's groundspeed first moving toward the new target (INCLUDES FC +
# physical acceleration, not pure transport).
_CMDLAT_HEADER = "t_wall_iso,t_mono,event,command,latency_ms,detail"
_CMD_NAMES = {178: "DO_CHANGE_SPEED", 512: "REQUEST_MESSAGE"}


class _CsvFile:
    def __init__(self, path: Path, meta: dict, header: str):
        self.path = path
        self.fh = open(path, "w", buffering=1)  # line-buffered
        for k, v in meta.items():
            self.fh.write(f"# {k}={v}\n")
        self.fh.write(f"# opened={time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        self.fh.write(header + "\n")
        self.rows = 0

    def write_row(self, fields: list) -> None:
        self.fh.write(",".join("" if x is None else str(x) for x in fields) + "\n")
        self.rows += 1

    def flush(self) -> None:
        try:
            self.fh.flush()
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.fh.flush()
            self.fh.close()
        except Exception:
            pass


class LinkLogger:
    def __init__(self, log_dir="logs", *, meta: dict | None = None,
                 enabled: bool = True, flush_every_s: float = 2.0):
        self.enabled = bool(enabled)
        self._lock = threading.Lock()
        self._last_flush = time.monotonic()
        self._flush_every = float(flush_every_s)
        # per (radio, msg_type) -> (last_t_mono, ema_hz)
        self._last: dict[tuple[str, str], float] = {}
        self.target_csv = None
        self.chaser_csv = None
        self.relay_csv = None
        self.cmd_lat = CmdLatencyTracker(None, enabled=False)
        if not self.enabled:
            return
        d = Path(log_dir)
        d.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        meta = dict(meta or {})
        self.target_csv = _CsvFile(d / f"hedef_link_{stamp}.csv",
                                   {**meta, "radio": "HEDEF"}, _LINK_HEADER)
        self.chaser_csv = _CsvFile(d / f"chaser_link_{stamp}.csv",
                                   {**meta, "radio": "TAKIPCI"}, _LINK_HEADER)
        self.relay_csv = _CsvFile(d / f"relay_lat_{stamp}.csv", meta, _RELAY_HEADER)
        self.cmd_lat = CmdLatencyTracker(d / f"cmd_lat_{stamp}.csv", meta, enabled=True)
        print(f"[LOG] link+latency logging -> {self.target_csv.path.parent}/"
              f"(hedef|chaser)_link_{stamp}.csv, relay_lat_{stamp}.csv, "
              f"cmd_lat_{stamp}.csv", flush=True)

    # ------------------------------------------------ per-message (RX threads)
    def note_message(self, radio: str, msg, t_mono: float | None = None) -> None:
        """Called from a Vehicle receive thread for every MAVLink message."""
        if not self.enabled:
            return
        now = time.monotonic() if t_mono is None else float(t_mono)
        try:
            mtype = msg.get_type()
        except Exception:
            return
        # RADIO_STATUS is emitted by the SiK radio itself (legacy name "RADIO").
        is_radio = mtype in ("RADIO_STATUS", "RADIO")
        with self._lock:
            csv = self.target_csv if radio == "HEDEF" else self.chaser_csv
            if csv is None:
                return
            key = (radio, mtype)
            prev = self._last.get(key)
            gap_ms = "" if prev is None else round((now - prev) * 1000.0, 1)
            inst_hz = "" if (prev is None or now <= prev) else round(1.0 / (now - prev), 2)
            self._last[key] = now
            try:
                src = msg.get_srcSystem()
            except Exception:
                src = ""
            radio_vals = [""] * len(_RADIO_FIELDS)
            if is_radio:
                for i, f in enumerate(_RADIO_FIELDS):
                    radio_vals[i] = getattr(msg, f, "")
            csv.write_row([
                time.strftime("%H:%M:%S"), round(now, 3), mtype, src, gap_ms, inst_hz,
                *radio_vals,
            ])
            self._maybe_flush(now)

    # ------------------------------------------------ per-tick (main loop)
    def note_relay(self, *, t_mono: float, updated: bool,
                   target_age_ms, chaser_age_ms, compute_ms, send_ms,
                   behavior, follow_phase, v_cmd_mps, dist_m, sep_m, gate_alt_m,
                   target_hz=None, chaser_hz=None) -> None:
        if not self.enabled or self.relay_csv is None:
            return

        def r(x, n=1):
            return "" if x is None else round(float(x), n)

        total = None
        if target_age_ms is not None and compute_ms is not None and send_ms is not None:
            total = float(target_age_ms) + float(compute_ms) + float(send_ms)
        with self._lock:
            self.relay_csv.write_row([
                time.strftime("%H:%M:%S"), round(float(t_mono), 3), int(bool(updated)),
                r(target_age_ms), r(chaser_age_ms), r(compute_ms, 2), r(send_ms, 2),
                r(total, 2), behavior or "", follow_phase or "",
                r(v_cmd_mps, 2), r(dist_m), r(sep_m), r(gate_alt_m),
                r(target_hz, 2), r(chaser_hz, 2),
            ])
            self._maybe_flush(time.monotonic())

    # ------------------------------------------------ housekeeping
    def _maybe_flush(self, now: float) -> None:
        if now - self._last_flush < self._flush_every:
            return
        self._last_flush = now
        for c in (self.target_csv, self.chaser_csv, self.relay_csv):
            if c is not None:
                c.flush()

    def close(self) -> None:
        if not self.enabled:
            return
        self.cmd_lat.close()
        with self._lock:
            for c in (self.target_csv, self.chaser_csv, self.relay_csv):
                if c is not None:
                    c.close()
            n_t = self.target_csv.rows if self.target_csv else 0
            n_c = self.chaser_csv.rows if self.chaser_csv else 0
            n_r = self.relay_csv.rows if self.relay_csv else 0
        print(f"[LOG] wrote hedef={n_t} chaser={n_c} relay={n_r} rows.", flush=True)


class CmdLatencyTracker:
    """Command->response latency for the chaser link.

    RTT: mark_command(cmd) records the send time of an ACK-eliciting command
    (DO_CHANGE_SPEED, or a periodic REQUEST_MESSAGE ping); note_ack(msg) matches
    its COMMAND_ACK and logs the round-trip. At most one send per command id is
    timed at a time; a stale outstanding one (no ACK within RTT_TIMEOUT_S) is
    logged as 'rtt_lost' — a run full of rtt_lost means the command link is
    dropping packets.

    spd_start: arm_speed(from_gs, target) + note_gs(gs) each tick logs how long
    until the chaser's groundspeed first moves >= SPD_MOVE_MPS toward a new
    speed target. This includes FC + physical acceleration (WP_ACC), so it is
    NOT pure transport latency — read it as 'how long until it visibly reacts'.

    Thread-safety: note_ack runs on the chaser receive thread; mark_command /
    arm_speed / note_gs run on the main loop. All writes take the lock.
    """

    RTT_TIMEOUT_S = 2.0
    SPD_TIMEOUT_S = 3.0
    SPD_MOVE_MPS = 0.5

    def __init__(self, path, meta=None, enabled=True):
        self.enabled = bool(enabled) and path is not None
        self._lock = threading.Lock()
        self._outstanding: dict[int, float] = {}   # command id -> send t_mono
        self._spd = None                           # (t_cmd, from_gs, target)
        self._last_flush = time.monotonic()
        self.csv = None
        self.rtt_count = 0
        self.lost_count = 0
        if self.enabled:
            self.csv = _CsvFile(path, dict(meta or {}), _CMDLAT_HEADER)

    def mark_command(self, command: int, t_mono: float) -> None:
        """Record the send of an ACK-eliciting command (main thread)."""
        if not self.enabled:
            return
        with self._lock:
            prev = self._outstanding.get(command)
            if prev is not None and (t_mono - prev) > self.RTT_TIMEOUT_S:
                # previous send never got an ACK -> lost
                self._row(t_mono, "rtt_lost", command, None,
                          f"no ack in {self.RTT_TIMEOUT_S:.0f}s")
                self.lost_count += 1
                prev = None
            if prev is None:
                self._outstanding[command] = t_mono
            # else keep waiting on the in-flight one (avoid match ambiguity)
            self._maybe_flush(t_mono)

    def note_ack(self, msg, t_mono: float) -> None:
        """Match a COMMAND_ACK to a timed send (chaser receive thread)."""
        if not self.enabled:
            return
        try:
            if msg.get_type() != "COMMAND_ACK":
                return
            cmd = int(msg.command)
        except Exception:
            return
        with self._lock:
            send_t = self._outstanding.pop(cmd, None)
            if send_t is None:
                return
            dt = t_mono - send_t
            if dt > self.RTT_TIMEOUT_S:
                self._row(t_mono, "rtt_lost", cmd, None, "ack late")
                self.lost_count += 1
            else:
                self._row(t_mono, "rtt", cmd, dt * 1000.0, "")
                self.rtt_count += 1
            self._maybe_flush(t_mono)

    def arm_speed(self, from_gs: float, target: float, t_mono: float) -> None:
        """Begin a speed-response measurement for a new speed target (main)."""
        if not self.enabled:
            return
        with self._lock:
            self._spd = (float(t_mono), float(from_gs), float(target))

    def note_gs(self, gs: float, t_mono: float) -> None:
        """Feed the chaser groundspeed each tick; logs spd_start when it moves."""
        if not self.enabled:
            return
        with self._lock:
            if self._spd is None:
                return
            t_cmd, from_gs, target = self._spd
            if (t_mono - t_cmd) > self.SPD_TIMEOUT_S:
                self._row(t_mono, "spd_timeout", 178, None,
                          f"from={from_gs:.1f} target={target:.1f}")
                self._spd = None
                return
            direction = 1.0 if target >= from_gs else -1.0
            if (float(gs) - from_gs) * direction >= self.SPD_MOVE_MPS:
                self._row(t_mono, "spd_start", 178, (t_mono - t_cmd) * 1000.0,
                          f"from={from_gs:.1f} target={target:.1f}")
                self._spd = None
            self._maybe_flush(t_mono)

    # --- internal (lock held) ---
    def _row(self, t_mono, event, command, latency_ms, detail):
        name = _CMD_NAMES.get(int(command), str(command))
        self.csv.write_row([
            time.strftime("%H:%M:%S"), round(float(t_mono), 3),
            event, name,
            "" if latency_ms is None else round(float(latency_ms), 1), detail,
        ])

    def _maybe_flush(self, now):
        if now - self._last_flush >= 2.0:
            self._last_flush = now
            if self.csv is not None:
                self.csv.flush()

    def close(self):
        if self.enabled and self.csv is not None:
            with self._lock:
                self.csv.close()
            print(f"[LOG] cmd latency: {self.rtt_count} RTT samples, "
                  f"{self.lost_count} lost.", flush=True)
