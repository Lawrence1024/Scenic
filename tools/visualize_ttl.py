#!/usr/bin/env python3
"""
Visualize TTL waypoints from CSV files.

This tool loads TTL CSV files and displays the waypoints in a 2D plot,
optionally overlaying them on the track map for alignment verification.

Usage:
    python visualize_ttl.py <ttl_csv_path> [--xodr <xodr_path>] [--dx <dx>] [--dy <dy>] [--save <output.png>]
    
Examples:
    # Visualize TTL 27 with default offsets
    python visualize_ttl.py assets/ttls/LS_ENU_TTL_CSV/needs_refine/ttl27_v5.csv
    
    # Visualize with custom offsets
    python visualize_ttl.py assets/ttls/LS_ENU_TTL_CSV/needs_refine/ttl27_v5.csv --dx -53.6 --dy -15.7
    
    # Visualize with track overlay
    python visualize_ttl.py assets/ttls/LS_ENU_TTL_CSV/needs_refine/ttl27_v5.csv --xodr assets/maps/dSPACE/LagunaSeca.xodr
    
    # Save plot to file
    python visualize_ttl.py assets/ttls/LS_ENU_TTL_CSV/needs_refine/ttl27_v5.csv --save ttl27_visualization.png
"""

import os
import sys
import csv
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path

# Add Scenic to path if needed
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))


def read_ttl_xy(csv_path: str, dx: float = 0.0, dy: float = 0.0):
    """Read ENU x,y from TTL CSV. Skips the first metadata line.
    
    Args:
        csv_path: Path to TTL CSV file
        dx: X offset to apply
        dy: Y offset to apply
        
    Returns:
        List of (x, y) tuples and metadata dict
    """
    points = []
    metadata = {}
    
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        # Read first line (metadata)
        try:
            meta_row = next(reader)
            if len(meta_row) >= 6:
                metadata = {
                    'id': meta_row[0] if len(meta_row) > 0 else None,
                    'num_points': int(meta_row[1]) if len(meta_row) > 1 else None,
                    'length': float(meta_row[2]) if len(meta_row) > 2 else None,
                    'lat': float(meta_row[3]) if len(meta_row) > 3 else None,
                    'lon': float(meta_row[4]) if len(meta_row) > 4 else None,
                    'elevation': float(meta_row[5]) if len(meta_row) > 5 else None,
                }
        except StopIteration:
            pass
        
        # Read waypoint data
        for i, row in enumerate(reader):
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0]) + dx
                y = float(row[1]) + dy
                points.append((x, y))
            except Exception:
                if i < 5:  # Only print first few errors
                    print(f"[WARN] Skipping malformed row {i+2}: {row}")
    
    return points, metadata


def load_track_reference(xodr_path: str):
    """Load track reference line from XODR file (simplified - just for visualization).
    
    This is a simplified version that extracts basic road geometry.
    For full track visualization, use the full XODR parser.
    """
    try:
        from scenic.formats.opendrive import xodr_parser
        from scenic.domains.driving.roads import Network
        
        network = Network.fromOpenDrive(xodr_path, ref_points=50)
        track_points = []
        
        # Extract points from road network
        for road in network.roads:
            for lane in road.lanes:
                # Sample points along lane centerline
                for i in range(0, len(lane.centerline), max(1, len(lane.centerline) // 100)):
                    pt = lane.centerline[i]
                    track_points.append((pt.x, pt.y))
        
        return track_points
    except Exception as e:
        print(f"[WARN] Could not load track from XODR: {e}")
        print(f"[INFO] Visualization will show TTL points only (no track overlay)")
        return None


def visualize_ttl(ttl_csv_path: str, xodr_path: str = None, dx: float = 0.0, dy: float = 0.0, 
                  save_path: str = None, show_plot: bool = True):
    """Visualize TTL waypoints.
    
    Args:
        ttl_csv_path: Path to TTL CSV file
        xodr_path: Optional path to XODR track file for overlay
        dx: X offset to apply to TTL points
        dy: Y offset to apply to TTL points
        save_path: Optional path to save the plot
        show_plot: Whether to display the plot interactively
    """
    # Load TTL points
    print(f"[INFO] Loading TTL from: {ttl_csv_path}")
    ttl_points, metadata = read_ttl_xy(ttl_csv_path, dx=dx, dy=dy)
    
    if not ttl_points:
        print(f"[ERROR] No waypoints found in {ttl_csv_path}")
        return
    
    print(f"[INFO] Loaded {len(ttl_points)} waypoints")
    if metadata:
        print(f"[INFO] TTL Metadata: ID={metadata.get('id')}, "
              f"Length={metadata.get('length', 'N/A')}m, "
              f"GPS=({metadata.get('lat')}, {metadata.get('lon')})")
    
    # Extract x and y coordinates
    x_coords = [p[0] for p in ttl_points]
    y_coords = [p[1] for p in ttl_points]
    
    # Create plot
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Load and plot track if available
    track_points = None
    if xodr_path and os.path.exists(xodr_path):
        print(f"[INFO] Loading track from: {xodr_path}")
        track_points = load_track_reference(xodr_path)
        if track_points:
            track_x = [p[0] for p in track_points]
            track_y = [p[1] for p in track_points]
            ax.plot(track_x, track_y, 'k-', linewidth=1, alpha=0.3, label='Track Reference', zorder=1)
    
    # Plot TTL waypoints
    ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7, label='TTL Path', zorder=2)
    ax.scatter(x_coords[::max(1, len(x_coords)//100)], y_coords[::max(1, len(y_coords)//100)], 
               c='red', s=10, alpha=0.6, label='TTL Waypoints (sampled)', zorder=3)
    
    # Mark start and end
    if len(ttl_points) >= 2:
        ax.scatter([x_coords[0]], [y_coords[0]], c='green', s=100, marker='o', 
                   label='Start', zorder=4, edgecolors='black', linewidths=2)
        ax.scatter([x_coords[-1]], [y_coords[-1]], c='red', s=100, marker='s', 
                   label='End', zorder=4, edgecolors='black', linewidths=2)
    
    # Set labels and title
    ax.set_xlabel('X (East, meters)', fontsize=12)
    ax.set_ylabel('Y (North, meters)', fontsize=12)
    
    csv_name = os.path.basename(ttl_csv_path)
    title = f'TTL Visualization: {csv_name}'
    if dx != 0.0 or dy != 0.0:
        title += f' (offset: dx={dx}, dy={dy})'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    # Add info text
    info_text = f"Points: {len(ttl_points)}"
    if metadata and metadata.get('length'):
        info_text += f"\nLength: {metadata['length']:.1f}m"
    if dx != 0.0 or dy != 0.0:
        info_text += f"\nOffset: ({dx}, {dy})"
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
            fontsize=9, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    # Save if requested
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[INFO] Saved visualization to: {save_path}")
    
    # Show plot
    if show_plot:
        plt.show()
    else:
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize TTL waypoints from CSV files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('ttl_csv', type=str, help='Path to TTL CSV file')
    parser.add_argument('--xodr', type=str, default=None, 
                       help='Optional path to XODR track file for overlay')
    parser.add_argument('--dx', type=float, default=0.0,
                       help='X offset to apply to TTL points (default: 0.0)')
    parser.add_argument('--dy', type=float, default=0.0,
                       help='Y offset to apply to TTL points (default: 0.0)')
    parser.add_argument('--save', type=str, default=None,
                       help='Save plot to file (e.g., output.png)')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display plot interactively (useful with --save)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.ttl_csv):
        print(f"[ERROR] TTL CSV file not found: {args.ttl_csv}")
        sys.exit(1)
    
    visualize_ttl(
        args.ttl_csv,
        xodr_path=args.xodr,
        dx=args.dx,
        dy=args.dy,
        save_path=args.save,
        show_plot=not args.no_show
    )


if __name__ == "__main__":
    main()

