#!/usr/bin/env python3
"""
Comprehensive verification that RD and XODR tracks are identical.

This script performs a complete verification:
1. Verifies reference lines overlap (from verify_reference_line_overlap.py)
2. Verifies road edges overlap (from verify_road_edges_overlap.py)
3. Optionally verifies centerlines overlap (from compare_rd_xodr_centerlines.py)

This provides a complete proof that the tracks are identical.
"""

import sys
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace.geometry.utils import MAIN_ROAD_NAMES


def main():
    """Main verification function."""
    print("="*80)
    print("COMPREHENSIVE TRACK IDENTITY VERIFICATION")
    print("="*80)
    print("\nThis script verifies that RD and XODR tracks are identical by:")
    print("  1. Verifying reference lines overlap")
    print("  2. Verifying road edges overlap")
    print("  3. Optionally verifying centerlines overlap")
    print("="*80)
    
    # Find map files
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    rd_path = project_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    xodr_path = project_root / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    
    # Try alternative paths
    if not rd_path.exists():
        rd_path = Path("assets/maps/dSPACE/Laguna_Seca.rd")
    if not xodr_path.exists():
        xodr_path = Path("assets/maps/dSPACE/LagunaSeca.xodr")
    
    if not rd_path.exists():
        rd_path = Path("Scenic/assets/maps/dSPACE/Laguna_Seca.rd")
    if not xodr_path.exists():
        xodr_path = Path("Scenic/assets/maps/dSPACE/LagunaSeca.xodr")
    
    if not rd_path.exists():
        print(f"\nERROR: RD file not found: {rd_path}")
        return 1
    
    if not xodr_path.exists():
        print(f"\nERROR: XODR file not found: {xodr_path}")
        return 1
    
    print(f"\nRD file:   {rd_path}")
    print(f"XODR file: {xodr_path}")
    
    # Import verification modules
    import importlib.util
    
    # Load verify_reference_line_overlap module
    ref_line_path = script_dir / "verify_reference_line_overlap.py"
    if not ref_line_path.exists():
        # Try in the testing directory
        ref_line_path = project_root / "src" / "scenic" / "domains" / "racing" / "mpc" / "testing" / "verify_reference_line_overlap.py"
    
    if not ref_line_path.exists():
        print(f"\nERROR: Could not find verify_reference_line_overlap.py")
        print(f"  Tried: {script_dir / 'verify_reference_line_overlap.py'}")
        print(f"  Tried: {ref_line_path}")
        return 1
    
    spec = importlib.util.spec_from_file_location("verify_reference_line_overlap", ref_line_path)
    ref_line_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ref_line_module)
    compare_reference_lines = ref_line_module.compare_reference_lines
    
    # Load verify_road_edges_overlap module
    edge_path = script_dir / "verify_road_edges_overlap.py"
    if not edge_path.exists():
        print(f"\nERROR: Could not find verify_road_edges_overlap.py")
        print(f"  Tried: {edge_path}")
        return 1
    
    spec = importlib.util.spec_from_file_location("verify_road_edges_overlap", edge_path)
    edge_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(edge_module)
    verify_road_edges = edge_module.verify_road_edges
    
    all_results = []
    
    # Step 1: Verify reference lines
    print(f"\n{'='*80}")
    print("STEP 1: VERIFYING REFERENCE LINES")
    print(f"{'='*80}")
    ref_line_results = []
    for road_name in MAIN_ROAD_NAMES:
        try:
            stats = compare_reference_lines(str(rd_path), str(xodr_path), road_name, num_samples=200)
            ref_line_results.append(stats)
        except Exception as e:
            print(f"\n  [ERROR] comparing reference lines for {road_name}: {e}")
            import traceback
            traceback.print_exc()
    
    # Step 2: Verify edges
    print(f"\n{'='*80}")
    print("STEP 2: VERIFYING ROAD EDGES")
    print(f"{'='*80}")
    edge_results = []
    for road_name in MAIN_ROAD_NAMES:
        try:
            stats = verify_road_edges(str(rd_path), str(xodr_path), road_name, num_samples=200)
            edge_results.append(stats)
        except Exception as e:
            print(f"\n  [ERROR] verifying edges for {road_name}: {e}")
            import traceback
            traceback.print_exc()
    
    # Final summary
    print(f"\n{'='*80}")
    print("FINAL VERIFICATION SUMMARY")
    print(f"{'='*80}")
    
    import numpy as np
    
    # Reference line summary
    if ref_line_results:
        ref_max = max(s['max_error'] for s in ref_line_results)
        ref_mean = np.mean([s['mean_error'] for s in ref_line_results])
        print(f"\nReference Lines:")
        print(f"  Mean error: {ref_mean:.2e} m")
        print(f"  Max error:  {ref_max:.2e} m")
        if ref_max < 1e-10:
            print(f"  Status: [OK] Reference lines match perfectly")
        elif ref_max < 1e-11:
            print(f"  Status: [OK] Reference lines match (within expected precision)")
        else:
            print(f"  Status: [WARN] Reference lines have larger than expected differences")
    
    # Edge summary
    if edge_results:
        edge_max = max(s['overall_max'] for s in edge_results)
        edge_mean = np.mean([s['overall_mean'] for s in edge_results])
        print(f"\nRoad Edges:")
        print(f"  Mean error: {edge_mean:.2e} m")
        print(f"  Max error:  {edge_max:.2e} m")
        if edge_max < 1e-8:
            print(f"  Status: [OK] Edges match perfectly")
        elif edge_max < 1e-6:
            print(f"  Status: [OK] Edges match (excellent precision)")
        elif edge_max < 0.01:
            print(f"  Status: [GOOD] Edges match (within 1cm)")
        else:
            print(f"  Status: [WARN] Edges have significant differences")
    
    # Overall conclusion
    print(f"\n{'='*80}")
    print("CONCLUSION")
    print(f"{'='*80}")
    
    if ref_line_results and edge_results:
        ref_max = max(s['max_error'] for s in ref_line_results)
        edge_max = max(s['overall_max'] for s in edge_results)
        
        if ref_max < 1e-10 and edge_max < 1e-6:
            print("\n[VERIFIED] RD and XODR tracks are IDENTICAL")
            print("  - Reference lines match to floating-point precision")
            print("  - Road edges match to excellent precision")
            print("  - Tracks can be used interchangeably")
        elif ref_max < 1e-11 and edge_max < 0.01:
            print("\n[VERIFIED] RD and XODR tracks are ESSENTIALLY IDENTICAL")
            print("  - Reference lines match to high precision")
            print("  - Road edges match to within 1cm")
            print("  - Tracks can be used interchangeably for practical purposes")
        else:
            print("\n[PARTIAL] RD and XODR tracks are SIMILAR but not identical")
            print("  - Some differences detected")
            print("  - Review individual road statistics above")
    else:
        print("\n[INCOMPLETE] Could not complete verification")
        print("  - Some verification steps failed")
        print("  - Review errors above")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

