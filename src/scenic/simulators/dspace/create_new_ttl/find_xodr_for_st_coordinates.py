#!/usr/bin/env python3
"""Measure dSPACE fellow route geometry for route-relative (s, t) samples.

This tool places fellows in batches of at most 30 at requested (s, t) coordinates on
the dSPACE routes, reads their actual RD positions from ControlDesk, transforms them
back to XODR coordinates, and stores all outputs plus checkpoint state in a dedicated
folder under `create_new_ttl/measurements`.

Default sweep:
    - Route R2 (main loop road): s = 0..3500, step 1 m, t in {-4, 0, +4}
    - Route R1 (pitlane road):   s = 0..3500, step 1 m, t in {-4, 0, +4}

Wrap-around overlap near the end of each route is expected and intentionally preserved.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

# Script lives at src/scenic/simulators/dspace/create_new_ttl/; repo root is 5 levels up
THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent.parent.parent.parent.parent
SRC_ROOT = REPO_ROOT / "src"
if SRC_ROOT.exists():
    sys.path.insert(0, str(SRC_ROOT))

import pythoncom
from win32com.client import Dispatch

from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.controldesk.readback import (
    FELLOW_GPS_BASE,
    FELLOW_GPS_BASE_ALT,
)
from scenic.simulators.dspace.geometry.coordinate_transform import (
    apply_inverse_coordinate_transform,
    load_transform,
)
from scenic.simulators.dspace.utils import legacy as dutils

MAX_BATCH_SIZE = 30
DEFAULT_ROUTES = ("R2", "R1")
DEFAULT_S_START = 0.0
DEFAULT_S_END = 3700.0
DEFAULT_S_STEP = 1.0
DEFAULT_T_VALUES = (-4.0, 0.0, 4.0)
DEFAULT_SETTLE_STEPS = 20
DEFAULT_SETTLE_SLEEP = 0.1
DEFAULT_STARTUP_SLEEP = 2.0
OUTPUT_DIR = THIS_DIR / "measurements"
RESULTS_CSV = OUTPUT_DIR / "route_st_to_xodr_measurements.csv"
SUMMARY_TXT = OUTPUT_DIR / "route_st_to_xodr_summary.txt"
CHECKPOINT_JSON = OUTPUT_DIR / "route_st_to_xodr_checkpoint.json"
SCENARIO_BASENAME = "RouteGeometryMeasurement"
TRANSFORM_PATH = REPO_ROOT / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"

BASE_FELLOW_PATH = (
    "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/"
    "FellowMovement/FELLOW_POS_VEL/FellowTrailer"
)


@dataclass(frozen=True)
class MeasurementConfig:
    routes: tuple[str, ...]
    s_start: float
    s_end: float
    s_step: float
    t_values: tuple[float, ...]
    batch_size: int
    settle_steps: int
    settle_sleep_s: float
    startup_sleep_s: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure dSPACE route geometry by sampling fellow placements in batches."
    )
    parser.add_argument("--routes", nargs="+", default=list(DEFAULT_ROUTES), help="Route names to measure (default: R2 R1)")
    parser.add_argument("--s-start", type=float, default=DEFAULT_S_START, help="Inclusive s start in meters")
    parser.add_argument("--s-end", type=float, default=DEFAULT_S_END, help="Inclusive s end in meters")
    parser.add_argument("--s-step", type=float, default=DEFAULT_S_STEP, help="s increment in meters")
    parser.add_argument(
        "--t-values",
        nargs="+",
        type=float,
        default=list(DEFAULT_T_VALUES),
        help="One or more lateral offsets to sample at each s value (default: -4 0 4)",
    )
    parser.add_argument("--batch-size", type=int, default=MAX_BATCH_SIZE, help="Number of fellows per batch (max 30)")
    parser.add_argument("--settle-steps", type=int, default=DEFAULT_SETTLE_STEPS, help="ControlDesk single-steps after each batch start")
    parser.add_argument("--settle-sleep", type=float, default=DEFAULT_SETTLE_SLEEP, help="Sleep between settle steps (seconds)")
    parser.add_argument("--startup-sleep", type=float, default=DEFAULT_STARTUP_SLEEP, help="Sleep after maneuver start/reset (seconds)")
    parser.add_argument("--reset-checkpoint", action="store_true", help="Delete existing checkpoint/results before starting")
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> MeasurementConfig:
    routes = tuple(dict.fromkeys(route.strip() for route in args.routes if route.strip()))
    if not routes:
        raise ValueError("At least one route must be provided.")
    t_values = tuple(float(t_value) for t_value in args.t_values)
    if not t_values:
        raise ValueError("At least one t value must be provided.")
    if args.batch_size < 1 or args.batch_size > MAX_BATCH_SIZE:
        raise ValueError(f"batch-size must be between 1 and {MAX_BATCH_SIZE}.")
    if args.s_step <= 0:
        raise ValueError("s-step must be positive.")
    if args.s_end < args.s_start:
        raise ValueError("s-end must be >= s-start.")
    if args.settle_steps < 0:
        raise ValueError("settle-steps must be >= 0.")
    if args.settle_sleep < 0 or args.startup_sleep < 0:
        raise ValueError("sleep durations must be >= 0.")
    return MeasurementConfig(
        routes=routes,
        s_start=float(args.s_start),
        s_end=float(args.s_end),
        s_step=float(args.s_step),
        t_values=t_values,
        batch_size=int(args.batch_size),
        settle_steps=int(args.settle_steps),
        settle_sleep_s=float(args.settle_sleep),
        startup_sleep_s=float(args.startup_sleep),
    )


def ensure_output_dir() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def iter_s_values(start: float, end: float, step: float) -> Iterable[float]:
    steps = int(round((end - start) / step))
    for idx in range(steps + 1):
        yield round(start + idx * step, 9)


def build_samples(config: MeasurementConfig) -> list[dict]:
    samples = []
    sample_index = 0
    for route in config.routes:
        for s_value in iter_s_values(config.s_start, config.s_end, config.s_step):
            for t_value in config.t_values:
                samples.append(
                    {
                        "sample_index": sample_index,
                        "route": route,
                        "s_input_m": float(s_value),
                        "t_input_m": float(t_value),
                    }
                )
                sample_index += 1
    return samples


def reset_output_files() -> None:
    for path in (RESULTS_CSV, SUMMARY_TXT, CHECKPOINT_JSON):
        if path.exists():
            path.unlink()


def load_coordinate_transform():
    if not TRANSFORM_PATH.exists():
        raise FileNotFoundError(f"Transform file not found: {TRANSFORM_PATH}")
    transform = load_transform(str(TRANSFORM_PATH))
    if transform is None:
        raise RuntimeError(f"Failed to load coordinate transform: {TRANSFORM_PATH}")
    return transform


def connect_modeldesk():
    pythoncom.CoInitialize()
    app = Dispatch("ModelDesk.Application")
    proj = app.ActiveProject
    if proj is None:
        raise RuntimeError("No active ModelDesk project. Open a project first.")
    exp = proj.ActiveExperiment
    if exp is None:
        raise RuntimeError("No active ModelDesk experiment. Activate an experiment first.")
    return app, proj, exp


def copy_scenario(exp, scenario_name: str):
    try:
        exp.TrafficScenario.SaveAs(scenario_name, True)
        exp.ActivateTrafficScenario(scenario_name)
        return exp.TrafficScenario
    except Exception as exc:
        print(f"[WARN] Could not copy scenario as '{scenario_name}': {exc}")
        return exp.TrafficScenario


def connect_controldesk() -> ControlDeskApp:
    cd = ControlDeskApp(
        prog_id="ControlDeskNG.Application",
        outer_platform_name="Platform",
        inner_platform_name="Platform_2",
    ).connect()
    try:
        cd.go_online()
    except Exception:
        pass
    try:
        cd.start_measurement()
    except Exception:
        pass
    return cd


def activate_route(route_sel, route_name: str) -> None:
    available_names = []
    try:
        available_names = [str(item) for item in list(route_sel.AvailableElements)]
    except Exception:
        available_names = []

    chosen = None
    if route_name in available_names:
        chosen = route_name
    else:
        route_name_upper = route_name.upper()
        for candidate in available_names:
            if str(candidate).upper() == route_name_upper:
                chosen = candidate
                break
    if chosen is None and available_names:
        raise RuntimeError(f"Route '{route_name}' not found. Available routes: {available_names}")
    if chosen is None:
        raise RuntimeError(f"Route '{route_name}' not found and no available routes were reported.")
    route_sel.Activate(chosen)


def configure_measurement_seg1(segs) -> None:
    """Configure segment 1 for measurement only.

    Longitudinal stays at Constant Velocity = 0, while lateral uses Continue so the
    segment does not force a constant lateral deviation during the calibration sweep.
    """
    dutils.configure_seg1_motion(segs, v=0.0, t=0.0, source_type="Constant")
    lat1 = segs[1].Activity.LateralType
    dutils.activate_type(lat1, "Continue")


def create_fellow_at_st(ts, fellow_name: str, s_val: float, t_val: float, route_name: str):
    fellow = ts.Fellows.Add()
    try:
        fellow.Name = fellow_name
    except Exception:
        fellow.Name = f"Fellow_{ts.Fellows.Count}"

    seqs = fellow.Sequences
    dutils.clear_collection(seqs)
    seq = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(seq)

    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
    try:
        configure_measurement_seg1(segs)
    except Exception as exc:
        print(f"    [WARN] Could not configure segment 1 for {fellow_name}: {exc}")
    try:
        dutils.make_endless_transition(segs)
    except Exception as exc:
        print(f"    [WARN] Could not set endless transition for {fellow_name}: {exc}")

    try:
        route_sel = seq.Route if hasattr(seq, "Route") else seq.RouteSelection
        route_sel.UseExternal = False
        route_sel.Direction = 0
        activate_route(route_sel, route_name)
    except Exception as exc:
        raise RuntimeError(f"Could not set route '{route_name}' for {fellow_name}: {exc}") from exc

    return fellow


def clear_existing_fellows(ts) -> None:
    dutils.clear_collection(ts.Fellows)


def download_and_start(ts, exp, cd: ControlDeskApp, config: MeasurementConfig) -> None:
    ts.Save()
    ts.Download()
    time.sleep(0.5)

    maneuver_control = exp.ManeuverControl
    try:
        maneuver_control.Stop()
    except Exception:
        pass
    time.sleep(0.2)
    maneuver_control.Reset()
    time.sleep(0.2)
    maneuver_control.Start(False)
    time.sleep(config.startup_sleep_s)

    for step_idx in range(config.settle_steps):
        cd.advance_simulation_step()
        if config.settle_sleep_s:
            time.sleep(config.settle_sleep_s)
    time.sleep(0.5)


def read_batch_positions(cd: ControlDeskApp, count: int) -> list[dict]:
    x_arr = list(cd.get_var(f"{BASE_FELLOW_PATH}/x") or [])
    y_arr = list(cd.get_var(f"{BASE_FELLOW_PATH}/y") or [])
    z_arr = list(cd.get_var(f"{BASE_FELLOW_PATH}/z") or [])
    positions = []
    for index in range(count):
        if index >= len(x_arr) or index >= len(y_arr):
            raise RuntimeError(f"FellowTrailer arrays shorter than expected batch size {count} (index {index}).")
        x_val = x_arr[index]
        y_val = y_arr[index]
        z_val = z_arr[index] if index < len(z_arr) else 0.0
        positions.append(
            {
                "rd_x_m": float(x_val) if x_val is not None else 0.0,
                "rd_y_m": float(y_val) if y_val is not None else 0.0,
                "rd_z_m": float(z_val) if z_val is not None else 0.0,
            }
        )
    return positions


def read_batch_gps(cd: ControlDeskApp, count: int) -> list[dict]:
    readings = []
    lon_arr = lat_arr = hdg_arr = None
    for base in (FELLOW_GPS_BASE, FELLOW_GPS_BASE_ALT):
        try:
            lon_candidate = cd.get_var(f"{base}/Longitude_deg")
            lat_candidate = cd.get_var(f"{base}/Latitude_deg")
            hdg_candidate = cd.get_var(f"{base}/Heading_deg")
            if (
                isinstance(lon_candidate, (list, tuple))
                and isinstance(lat_candidate, (list, tuple))
                and isinstance(hdg_candidate, (list, tuple))
            ):
                lon_arr, lat_arr, hdg_arr = lon_candidate, lat_candidate, hdg_candidate
                break
        except Exception:
            continue

    for index in range(count):
        gps_row = {
            "gps_longitude_deg": None,
            "gps_latitude_deg": None,
            "gps_heading_deg": None,
        }
        if (
            isinstance(lon_arr, (list, tuple))
            and isinstance(lat_arr, (list, tuple))
            and isinstance(hdg_arr, (list, tuple))
            and index < len(lon_arr)
            and index < len(lat_arr)
            and index < len(hdg_arr)
        ):
            lon_val = lon_arr[index]
            lat_val = lat_arr[index]
            hdg_val = hdg_arr[index]
            if lon_val is not None and lat_val is not None and hdg_val is not None:
                gps_row = {
                    "gps_longitude_deg": float(lon_val),
                    "gps_latitude_deg": float(lat_val),
                    "gps_heading_deg": float(hdg_val),
                }
        readings.append(gps_row)
    return readings


def measure_batch(batch_samples: list[dict], coordinate_transform, ts, exp, cd: ControlDeskApp, config: MeasurementConfig, batch_index: int) -> list[dict]:
    print("\n" + "=" * 80)
    print(
        f"Batch {batch_index + 1}: route={batch_samples[0]['route']} "
        f"indices {batch_samples[0]['sample_index']}..{batch_samples[-1]['sample_index']} "
        f"(count={len(batch_samples)})"
    )
    print("=" * 80)

    clear_existing_fellows(ts)
    for offset, sample in enumerate(batch_samples):
        fellow_name = f"{sample['route']}_S{int(round(sample['s_input_m']))}_{offset}"
        create_fellow_at_st(ts, fellow_name, sample["s_input_m"], sample["t_input_m"], sample["route"])

    download_and_start(ts, exp, cd, config)
    positions = read_batch_positions(cd, len(batch_samples))
    gps_readings = read_batch_gps(cd, len(batch_samples))

    measured_at = time.strftime("%Y-%m-%dT%H:%M:%S")
    batch_results = []
    for offset, (sample, position, gps_reading) in enumerate(zip(batch_samples, positions, gps_readings)):
        xodr_x, xodr_y = apply_inverse_coordinate_transform(
            coordinate_transform,
            (position["rd_x_m"], position["rd_y_m"]),
        )
        result = {
            **sample,
            **position,
            **gps_reading,
            "xodr_x_m": float(xodr_x),
            "xodr_y_m": float(xodr_y),
            "xodr_z_m": float(position["rd_z_m"]),
            "batch_index": batch_index,
            "fellow_index_in_batch": offset,
            "measured_at": measured_at,
        }
        batch_results.append(result)
        print(
            f"  [{sample['route']}] s={sample['s_input_m']:.1f}, t={sample['t_input_m']:.3f} -> "
            f"RD=({result['rd_x_m']:.6f}, {result['rd_y_m']:.6f}, {result['rd_z_m']:.6f}) "
            f"XODR=({result['xodr_x_m']:.6f}, {result['xodr_y_m']:.6f}, {result['xodr_z_m']:.6f}) "
            f"GPS=({result['gps_longitude_deg']}, {result['gps_latitude_deg']}, {result['gps_heading_deg']})"
        )
    return batch_results


def save_results_csv(results: list[dict]) -> None:
    fieldnames = [
        "sample_index",
        "route",
        "s_input_m",
        "t_input_m",
        "rd_x_m",
        "rd_y_m",
        "rd_z_m",
        "gps_longitude_deg",
        "gps_latitude_deg",
        "gps_heading_deg",
        "xodr_x_m",
        "xodr_y_m",
        "xodr_z_m",
        "batch_index",
        "fellow_index_in_batch",
        "measured_at",
    ]
    with open(RESULTS_CSV, "w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)


def save_summary(results: list[dict], config: MeasurementConfig) -> None:
    lines = [
        "dSPACE route geometry measurement summary",
        "=" * 60,
        f"Routes: {', '.join(config.routes)}",
        f"s range: {config.s_start:.1f} .. {config.s_end:.1f} (step {config.s_step:.1f})",
        f"t values: {', '.join(f'{value:.3f}' for value in config.t_values)}",
        f"Batch size: {config.batch_size}",
        f"Total results: {len(results)}",
        "",
    ]
    by_route = {}
    for row in results:
        by_route.setdefault(row["route"], 0)
        by_route[row["route"]] += 1
    for route in config.routes:
        lines.append(f"{route}: {by_route.get(route, 0)} samples")
    lines.append("")
    lines.append(f"CSV: {RESULTS_CSV}")
    lines.append(f"Checkpoint: {CHECKPOINT_JSON}")
    with open(SUMMARY_TXT, "w", encoding="utf-8") as output_file:
        output_file.write("\n".join(lines) + "\n")


def save_checkpoint(results: list[dict], next_batch_index: int, total_batches: int, config: MeasurementConfig) -> None:
    payload = {
        "version": 1,
        "config": asdict(config),
        "next_batch_index": next_batch_index,
        "total_batches": total_batches,
        "results": results,
        "updated_at": time.time(),
    }
    with open(CHECKPOINT_JSON, "w", encoding="utf-8") as checkpoint_file:
        json.dump(payload, checkpoint_file, indent=2)


def _config_matches(checkpoint_config: dict, current: dict) -> bool:
    """Compare configs so list/tuple and float representation do not cause false mismatches."""
    if set(checkpoint_config.keys()) != set(current.keys()):
        return False
    for key in checkpoint_config:
        a, b = checkpoint_config[key], current[key]
        if key in ("routes", "t_values"):
            if list(a) != list(b):
                return False
        elif isinstance(a, float) and isinstance(b, float):
            if abs(a - b) > 1e-12:
                return False
        else:
            if a != b:
                return False
    return True


def load_checkpoint(config: MeasurementConfig):
    if not CHECKPOINT_JSON.exists():
        return None
    with open(CHECKPOINT_JSON, "r", encoding="utf-8") as checkpoint_file:
        payload = json.load(checkpoint_file)
    if not _config_matches(payload.get("config", {}), asdict(config)):
        raise RuntimeError(
            "Existing checkpoint configuration does not match current arguments. "
            "Use --reset-checkpoint to start over."
        )
    return payload


def clear_checkpoint() -> None:
    if CHECKPOINT_JSON.exists():
        CHECKPOINT_JSON.unlink()


def build_batches(samples: list[dict], chunk_size: int) -> list[list[dict]]:
    batches = []
    current_route = None
    current_group = []
    for sample in samples:
        if current_route is None:
            current_route = sample["route"]
        if sample["route"] != current_route or len(current_group) >= chunk_size:
            batches.append(current_group)
            current_group = []
            current_route = sample["route"]
        current_group.append(sample)
    if current_group:
        batches.append(current_group)
    return batches


def main() -> int:
    args = parse_args()
    config = build_config(args)
    ensure_output_dir()
    if args.reset_checkpoint:
        reset_output_files()

    all_samples = build_samples(config)
    batches = build_batches(all_samples, config.batch_size)
    total_batches = len(batches)

    print("=" * 80)
    print("MEASURE dSPACE ROUTE GEOMETRY (s,t -> RD -> XODR)")
    print("=" * 80)
    print(f"Routes: {', '.join(config.routes)}")
    print(
        f"s range: {config.s_start:.1f} .. {config.s_end:.1f} (step {config.s_step:.1f}), "
        f"t values={', '.join(f'{value:.1f}' for value in config.t_values)}"
    )
    print(f"Batch size: {config.batch_size} (max {MAX_BATCH_SIZE})")
    print(f"Total samples: {len(all_samples)} in {total_batches} batches")
    print(f"Outputs: {OUTPUT_DIR}")

    checkpoint = load_checkpoint(config)
    if checkpoint:
        results = checkpoint.get("results", [])
        start_batch_index = int(checkpoint.get("next_batch_index", 0))
        print(f"[CHECKPOINT] Resuming at batch {start_batch_index + 1}/{total_batches} with {len(results)} saved results.")
    else:
        results = []
        start_batch_index = 0
        print("[CHECKPOINT] No matching checkpoint found; starting from scratch.")

    coordinate_transform = load_coordinate_transform()
    scenario_name = f"{SCENARIO_BASENAME}_{time.strftime('%Y%m%d_%H%M%S')}"

    app = proj = exp = ts = cd = None
    try:
        app, proj, exp = connect_modeldesk()
        print(f"[ModelDesk] Project: {getattr(proj, 'Name', 'Unknown')}")
        print(f"[ModelDesk] Experiment: {getattr(exp, 'Name', 'Unknown')}")
        ts = copy_scenario(exp, scenario_name)
        print(f"[ModelDesk] Scenario: {getattr(ts, 'Name', scenario_name)}")

        cd = connect_controldesk()
        print("[ControlDesk] Connected and ready.")

        for batch_index in range(start_batch_index, total_batches):
            batch_samples = batches[batch_index]
            batch_results = measure_batch(batch_samples, coordinate_transform, ts, exp, cd, config, batch_index)
            results.extend(batch_results)
            save_results_csv(results)
            save_summary(results, config)
            save_checkpoint(results, batch_index + 1, total_batches, config)

        print("\n" + "=" * 80)
        print("MEASUREMENT COMPLETE")
        print("=" * 80)
        print(f"Saved {len(results)} samples to {RESULTS_CSV}")
        save_summary(results, config)
        clear_checkpoint()
        return 0
    finally:
        try:
            if cd is not None:
                try:
                    cd.stop_measurement()
                except Exception:
                    pass
                try:
                    cd.go_offline()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            pythoncom.CoUninitialize()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Measurement interrupted. Resume later with the checkpoint in place.")
        sys.exit(1)
    except Exception as exc:
        print(f"\n[FATAL ERROR] {exc}")
        raise

