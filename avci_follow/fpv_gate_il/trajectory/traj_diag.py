"""Trajectory-phase diagnostic CSV (radar cycle / planner outputs)."""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any, TextIO

from fpv_gate_il.trajectory.types import PlannerOutput

TRAJ_DIAG_COLUMNS: tuple[str, ...] = (
    "t_s",
    "phase",
    "behavior",
    "follow_phase",
    "allow_lock",
    "realign",
    "realign_phase",
    "realign_turn",
    "realign_arrest",
    "xt_m",
    "along_m",
    "course_err_deg",
    "range_m",
    "t_400_s",
    "feasible",
    "path_cost",
    "travel_time_s",
    "n_wp",
    "v_cmd_mps",
    "drone_n_m",
    "drone_e_m",
    "drone_alt_m",
    "drone_gs_mps",
    "target_n_m",
    "target_e_m",
    "target_vn",
    "target_ve",
    "final_n_m",
    "final_e_m",
    "final_alt_m",
    "wp0_n_m",
    "wp0_e_m",
    "wp0_spd",
    "notes",
)


def default_traj_diag_path(log_dir: Path | str = "data") -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(log_dir).expanduser() / f"traj_diag_{stamp}.csv"


class TrajDiagLogger:
    """Append-only CSV of planner outputs for TRACK_PLAN."""

    def __init__(self, path: Path | str, *, meta: dict[str, Any] | None = None) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh: TextIO = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._fh,
            fieldnames=list(TRAJ_DIAG_COLUMNS),
            extrasaction="ignore",
        )
        if meta:
            for key, val in meta.items():
                self._fh.write(f"# {key}={val}\n")
        self._writer.writeheader()
        self._fh.flush()
        self._t0 = time.monotonic()
        self._rows = 0
        print(f"[TRAJ_DIAG] logging → {self.path}", flush=True)

    @property
    def rows(self) -> int:
        return self._rows

    def close(self) -> None:
        if self._fh is not None and not self._fh.closed:
            self._fh.flush()
            self._fh.close()
        print(f"[TRAJ_DIAG] wrote {self._rows} rows → {self.path}", flush=True)

    def write(
        self,
        output: PlannerOutput,
        *,
        drone_n: float,
        drone_e: float,
        drone_alt: float,
        drone_gs_mps: float,
        v_cmd_mps: float | None,
        notes: str = "",
    ) -> None:
        path = output.path
        target = output.target
        final = path.final_position if path is not None else None
        wp0 = path.waypoints[0] if path is not None and path.waypoints else None
        row = {
            "t_s": time.monotonic() - self._t0,
            "phase": output.phase.value,
            "behavior": output.behavior.value if output.behavior else "",
            "follow_phase": output.follow_phase.value,
            "allow_lock": int(bool(output.allow_visual_lock)),
            "realign": int(bool(output.realign_active)),
            "realign_phase": output.realign_phase.value if output.realign_active else "",
            "realign_turn": float(output.realign_turn_sign),
            "realign_arrest": int(bool(output.realign_arrest)),
            "xt_m": output.xt_m,
            "along_m": output.along_track_m,
            "course_err_deg": output.course_err_deg,
            "range_m": output.distance_to_target_m,
            "t_400_s": output.t_400_s if output.t_400_s is not None else float("nan"),
            "feasible": int(bool(path.feasible)) if path is not None else 0,
            "path_cost": path.cost if path is not None else float("nan"),
            "travel_time_s": path.travel_time_s if path is not None else float("nan"),
            "n_wp": len(path.waypoints) if path is not None else 0,
            "v_cmd_mps": float(v_cmd_mps) if v_cmd_mps is not None else float("nan"),
            "drone_n_m": drone_n,
            "drone_e_m": drone_e,
            "drone_alt_m": drone_alt,
            "drone_gs_mps": drone_gs_mps,
            "target_n_m": target.position.north_m if target else float("nan"),
            "target_e_m": target.position.east_m if target else float("nan"),
            "target_vn": target.velocity.vn_mps if target else float("nan"),
            "target_ve": target.velocity.ve_mps if target else float("nan"),
            "final_n_m": final.north_m if final else float("nan"),
            "final_e_m": final.east_m if final else float("nan"),
            "final_alt_m": final.alt_up_m if final else float("nan"),
            "wp0_n_m": wp0.position.north_m if wp0 else float("nan"),
            "wp0_e_m": wp0.position.east_m if wp0 else float("nan"),
            "wp0_spd": wp0.desired_speed_mps if wp0 else float("nan"),
            "notes": notes or (path.notes if path is not None else ""),
        }
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % 5 == 0:
            self._fh.flush()
