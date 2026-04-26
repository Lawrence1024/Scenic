"""Per-step CSV logger for the centerline-calibration drive scenario.

Usage from Scenic:
    from examples.racing.calibration.centerline_logger import init_logger, log_step
    init_logger('tools/frames/data/lgs_v1_centerline_drive.csv')
    monitor CenterlineDriveLogger():
        while True:
            log_step(simulation())
            wait
    require monitor CenterlineDriveLogger()

Each row captures one fellow's RD-frame readback at one sim step.
Output schema:
    sim_t, name, race_number, route, d_setpoint_m,
    x_rd, y_rd, yaw_rad, speed_mps
"""
import atexit
import csv
import math
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]

_state = {
    "file": None,
    "writer": None,
    "path": None,
    "row_count": 0,
    "warned": False,
}


def init_logger(rel_path: str) -> None:
    if _state["writer"] is not None:
        return
    out = (_REPO_ROOT / rel_path).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    f = open(out, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow([
        "sim_t", "name", "race_number", "route", "d_setpoint_m",
        "x_rd", "y_rd", "yaw_rad", "speed_mps",
    ])
    _state["file"] = f
    _state["writer"] = w
    _state["path"] = out
    _state["row_count"] = 0
    print(f"[CenterlineLogger] writing to {out}")
    atexit.register(close_logger)


def close_logger() -> None:
    f = _state["file"]
    if f is not None:
        try:
            f.flush()
            f.close()
        except Exception:
            pass
        _state["file"] = None
        _state["writer"] = None
        print(
            f"[CenterlineLogger] closed {_state['path']} "
            f"({_state['row_count']} rows)"
        )


def log_step(sim) -> None:
    w = _state["writer"]
    if w is None:
        return
    try:
        scene = getattr(sim, "scene", None)
        if scene is None:
            return
        time_step = 0.01
        try:
            params = getattr(scene, "params", None) or {}
            ts = params.get("time_step")
            if ts is not None:
                time_step = float(ts)
        except Exception:
            pass
        ct = int(getattr(sim, "currentTime", 0) or 0)
        t_s = ct * time_step

        ego = getattr(scene, "egoObject", None)
        for obj in getattr(scene, "objects", []) or []:
            if obj is ego:
                continue
            actor = getattr(obj, "dspaceActor", None)
            if actor is None:
                continue
            rd_pos = getattr(actor, "rd_position", None)
            if rd_pos is None:
                continue
            heading = 0.0
            try:
                heading = float(getattr(actor, "heading", 0.0))
            except Exception:
                pass
            speed = 0.0
            linvel = getattr(actor, "linvel", None)
            if linvel is not None:
                try:
                    speed = math.hypot(float(linvel.x), float(linvel.y))
                except Exception:
                    pass
            race_number = getattr(obj, "raceNumber", "")
            try:
                race_number = int(race_number)
            except (TypeError, ValueError):
                race_number = ""
            name = str(getattr(obj, "name", "") or "")
            route = str(getattr(obj, "_route", "") or "")
            d_setpoint = 0.0
            rst = getattr(obj, "_route_s_t", None)
            if rst is not None and len(rst) >= 2:
                try:
                    d_setpoint = float(rst[1])
                except Exception:
                    d_setpoint = 0.0
            fps = getattr(obj, "_fellow_plant_state", None)
            if isinstance(fps, dict) and fps.get("d_m") is not None:
                try:
                    d_setpoint = float(fps["d_m"])
                except Exception:
                    pass
            try:
                x_rd = float(rd_pos[0])
                y_rd = float(rd_pos[1])
            except Exception:
                continue
            w.writerow([
                f"{t_s:.3f}", name, race_number, route, f"{d_setpoint:.3f}",
                f"{x_rd:.4f}", f"{y_rd:.4f}", f"{heading:.6f}", f"{speed:.3f}",
            ])
            _state["row_count"] += 1

        if _state["file"] is not None and ct > 0 and ct % 200 == 0:
            try:
                _state["file"].flush()
            except Exception:
                pass
    except Exception as e:
        if not _state["warned"]:
            print(f"[CenterlineLogger] log_step error (suppressing further): {e}")
            _state["warned"] = True
