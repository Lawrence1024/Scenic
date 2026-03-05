"""
GNSS <-> Scenic local (XODR) coordinate transformation.

The racing library enforces that position read-in can be in GNSS form; this module
converts GNSS (longitude_deg, latitude_deg) to Scenic local (x, y) and back.
Calibration is from a table of (lon, lat, x_local, y_local) collected at runtime.
Uses a reference point and local East-North (m), then an affine map from (E, N) to local (x, y).
Bidirectional: gnss_to_local(lon, lat) -> (x, y) and local_to_gnss(x, y) -> (lon, lat).
"""

import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# WGS84 approximate radius (m) for local flat-earth conversion
_EARTH_RADIUS_M = 6371000.0


def _gps_to_local_en(lon_deg: float, lat_deg: float, lon0_deg: float, lat0_deg: float) -> Tuple[float, float]:
    """Convert (lon, lat) in degrees to local East, North in meters (flat-earth approximation)."""
    lat0_rad = math.radians(lat0_deg)
    deg_to_rad = math.pi / 180.0
    E = (lon_deg - lon0_deg) * math.cos(lat0_rad) * _EARTH_RADIUS_M * deg_to_rad
    N = (lat_deg - lat0_deg) * _EARTH_RADIUS_M * deg_to_rad
    return (E, N)


def _local_en_to_gps(E: float, N: float, lon0_deg: float, lat0_deg: float) -> Tuple[float, float]:
    """Convert local East, North (m) to (lon, lat) in degrees."""
    lat0_rad = math.radians(lat0_deg)
    deg_to_rad = math.pi / 180.0
    lon_deg = lon0_deg + E / (math.cos(lat0_rad) * _EARTH_RADIUS_M * deg_to_rad)
    lat_deg = lat0_deg + N / (_EARTH_RADIUS_M * deg_to_rad)
    return (lon_deg, lat_deg)


class GNSSLocalTransform:
    """
    Bidirectional transform between GNSS (lon, lat) and Scenic local (x, y).
    Calibration is: (E, N) = local from GPS; [x; y] = A @ [E; N] + t.
    """

    def __init__(
        self,
        lon0_deg: float,
        lat0_deg: float,
        A: np.ndarray,
        t: np.ndarray,
    ):
        """
        Args:
            lon0_deg, lat0_deg: Reference point for local East-North (degrees).
            A: 2x2 matrix, (x, y)^T = A @ (E, N)^T + t.
            t: 2x1 translation (x0, y0).
        """
        self.lon0_deg = float(lon0_deg)
        self.lat0_deg = float(lat0_deg)
        self._A = np.asarray(A, dtype=float)
        self._t = np.asarray(t, dtype=float).reshape(2)
        self._A_inv = np.linalg.inv(self._A)

    def gnss_to_local(self, lon_deg: float, lat_deg: float) -> Tuple[float, float]:
        """Convert GNSS (longitude_deg, latitude_deg) to Scenic local (x, y) in meters."""
        E, N = _gps_to_local_en(lon_deg, lat_deg, self.lon0_deg, self.lat0_deg)
        xy = self._A @ np.array([E, N]) + self._t
        return (float(xy[0]), float(xy[1]))

    def local_to_gnss(self, x: float, y: float) -> Tuple[float, float]:
        """Convert Scenic local (x, y) in meters to GNSS (longitude_deg, latitude_deg)."""
        xy = np.array([x, y])
        en = self._A_inv @ (xy - self._t)
        return _local_en_to_gps(float(en[0]), float(en[1]), self.lon0_deg, self.lat0_deg)

    # Aliases for backward compatibility
    def gps_to_dspace(self, lon_deg: float, lat_deg: float) -> Tuple[float, float]:
        return self.gnss_to_local(lon_deg, lat_deg)

    def dspace_to_gps(self, x: float, y: float) -> Tuple[float, float]:
        return self.local_to_gnss(x, y)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize for JSON save."""
        return {
            "lon0_deg": self.lon0_deg,
            "lat0_deg": self.lat0_deg,
            "A": self._A.tolist(),
            "t": self._t.tolist(),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "GNSSLocalTransform":
        """Load from dict (e.g. JSON)."""
        return cls(
            lon0_deg=d["lon0_deg"],
            lat0_deg=d["lat0_deg"],
            A=np.array(d["A"]),
            t=np.array(d["t"]),
        )


# Backward-compatible alias
GPSDspaceTransform = GNSSLocalTransform


def load_calibration_table_csv(csv_path: Path, target: str = "xodr") -> np.ndarray:
    """
    Load gps_dspace_table.csv (or similar) into arrays.
    Returns (lon_deg, lat_deg, x_target, y_target) as (N, 4) array.
    target: "xodr" use x_dspace, y_dspace (Scenic XODR); "rd" use x_rd, y_rd (dSPACE RD) when present.
    Rows with missing lon/lat/x/y are skipped.
    """
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                lon = float(row.get("longitude_deg", row.get("lon_deg", "")) or 0)
                lat = float(row.get("latitude_deg", row.get("lat_deg", "")) or 0)
                if target == "rd" and "x_rd" in row and "y_rd" in row and row.get("x_rd", "").strip() and row.get("y_rd", "").strip():
                    x = float(row["x_rd"])
                    y = float(row["y_rd"])
                else:
                    x = float(row.get("x_dspace", "") or 0)
                    y = float(row.get("y_dspace", "") or 0)
            except (ValueError, TypeError):
                continue
            rows.append([lon, lat, x, y])
    return np.array(rows) if rows else np.empty((0, 4))


def load_gps_table_rows(csv_path: Path) -> List[Dict[str, float]]:
    """Load full table as list of dicts with numeric values (for round-trip verify)."""
    out = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                d = {}
                for k, v in row.items():
                    if v is None or not str(v).strip():
                        continue
                    try:
                        d[k] = float(v)
                    except ValueError:
                        pass
                if "longitude_deg" in d and "latitude_deg" in d and "x_dspace" in d and "y_dspace" in d:
                    out.append(d)
            except Exception:
                continue
    return out


def fit_transform_from_table(
    table: np.ndarray,
    lon0_deg: Optional[float] = None,
    lat0_deg: Optional[float] = None,
) -> GNSSLocalTransform:
    """
    Fit GNSS -> local transform from Nx4 table [lon_deg, lat_deg, x_local, y_local].
    Uses reference (lon0, lat0); if None, uses mean lon/lat of the table.
    """
    if table.size == 0 or len(table) < 3:
        raise ValueError("Need at least 3 rows for calibration")
    lon = table[:, 0]
    lat = table[:, 1]
    x_ds = table[:, 2]
    y_ds = table[:, 3]
    if lon0_deg is None:
        lon0_deg = float(np.mean(lon))
    if lat0_deg is None:
        lat0_deg = float(np.mean(lat))
    E, N = [], []
    for i in range(len(lon)):
        e, n = _gps_to_local_en(lon[i], lat[i], lon0_deg, lat0_deg)
        E.append(e)
        N.append(n)
    E = np.array(E)
    N = np.array(N)
    X = np.column_stack([E, N, np.ones(len(E))])
    Y = np.column_stack([x_ds, y_ds])
    beta, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
    A = beta[:2, :].T
    t = beta[2, :]
    return GNSSLocalTransform(lon0_deg=lon0_deg, lat0_deg=lat0_deg, A=A, t=t)


def fit_transform_from_csv(
    csv_path: Path,
    lon0_deg: Optional[float] = None,
    lat0_deg: Optional[float] = None,
    target: str = "xodr",
) -> GNSSLocalTransform:
    """Load calibration table from CSV and fit transform. target: 'xodr' (x_dspace, y_dspace) or 'rd' (x_rd, y_rd)."""
    table = load_calibration_table_csv(csv_path, target=target)
    if len(table) < 3:
        raise ValueError(f"CSV has too few valid rows for target={target}: {len(table)}")
    return fit_transform_from_table(table, lon0_deg=lon0_deg, lat0_deg=lat0_deg)


def save_calibration(transform: GNSSLocalTransform, json_path: Path) -> None:
    """Save calibration to JSON for later load."""
    json_path = Path(json_path)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(transform.to_dict(), f, indent=2)


def load_calibration(json_path: Path) -> GNSSLocalTransform:
    """Load calibration from JSON."""
    with open(json_path, encoding="utf-8") as f:
        return GNSSLocalTransform.from_dict(json.load(f))
