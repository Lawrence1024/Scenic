#!/usr/bin/env python3
"""
Verify that Aurelion's (x, y, z) -> (s, t) mapping
returns the same longitudinal coordinate s as ModelDesk's s_forward.

Requires:
  - record_st_coordinates_v2.py (the one generating Scenic + parsing logs)
  - safe_centerline_xy_from_s() from your geometry utilities file
  - a working dSPACE Aurelion simulator accessible via the scenic CLI
"""

import subprocess
from pathlib import Path
import csv
from scenic.formats.opendrive import xodr_parser
from s_to_xy import safe_centerline_xy_from_s  # adjust import

SCRIPT_DIR = Path(__file__).parent
XODR_FILE = (SCRIPT_DIR / "../../assets/maps/dSPACE/LagunaSeca.xodr").resolve()
RECORD_SCRIPT = (SCRIPT_DIR / "record_st_coordinates_v2.py").resolve()

# Test points along the track (in meters)
TEST_S_VALUES = [0, 50, 100, 200, 400, 800, 1200, 1600]

def run_scenic_conversion(points_file: Path, log_file: Path):
    """Run Scenic + Aurelion to produce (s,t) log for the given Scenic file."""
    cmd = [
        "scenic",
        str(points_file),
        "--2d",
        "--model", "scenic.simulators.dspace.model",
        "--simulate"
    ]
    with open(log_file, "w") as f:
        subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, check=False)

def parse_aurelion_st(log_text: str):
    """Extract (s,t) pairs from simulator output log."""
    out = []
    for line in log_text.splitlines():
        if "Transformed world coordinates" in line and "to road coordinates" in line:
            try:
                parts = line.split("(s=")[1].split(",")
                s_val = float(parts[0].strip())
                t_val = float(parts[1].split("t=")[1].split(")")[0].strip())
                out.append((s_val, t_val))
            except Exception:
                pass
    return out

def verify_alignment():
    scenic_lines = [
        "param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')",
        "param time_step = 1.0/10",
        "model scenic.simulators.dspace.model",
        ""
    ]
    samples = []
    for i, s_fwd in enumerate(TEST_S_VALUES, 1):
        x, y, s_adj = safe_centerline_xy_from_s(XODR_FILE, s_fwd)
        samples.append((s_adj, x, y))
        scenic_lines.append(f"# s_forward={s_adj:.3f}")
        scenic_lines.append(f"fellow{i} = new Car at ({x:.6f}, {y:.6f}, 0.0)")
        scenic_lines.append("")

    scenic_file = SCRIPT_DIR / "verify_s_alignment.scenic"
    log_file = SCRIPT_DIR / "verify_s_alignment.log"
    with open(scenic_file, "w") as f:
        f.write("\n".join(scenic_lines))

    print(f"Running Scenic simulation on {len(samples)} points...")
    run_scenic_conversion(scenic_file, log_file)

    # Parse simulator output
    log_text = log_file.read_text()
    st_pairs = parse_aurelion_st(log_text)

    print("\n=== Results ===")
    print(f"{'Index':<6}{'s_forward':<12}{'s_aurelion':<12}{'Δs (m)':<10}{'t_aurelion':<12}")
    print("-" * 60)
    for i, ((s_fwd, x, y), (s_aur, t_aur)) in enumerate(zip(samples, st_pairs), 1):
        delta = s_aur - s_fwd
        print(f"{i:<6}{s_fwd:<12.3f}{s_aur:<12.3f}{delta:<10.4f}{t_aur:<12.3f}")

if __name__ == "__main__":
    verify_alignment()
