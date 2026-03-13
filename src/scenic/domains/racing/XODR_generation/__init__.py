"""Generate OpenDRIVE (.xodr) files from TTL centerline CSVs.

This package provides scripts to build XODR with:
- Main track and pit track as two roads from TTL centerlines
- Fixed lane widths (e.g. 5.5 m main, 3 m pit)
- Predecessor/successor links so the roads form one connected loop (no junctions)

Usage:
  From Scenic repo root:
    python -m scenic.domains.racing.XODR_generation.build_ttl_xodr [options]
  Or:
    python src/scenic/domains/racing/XODR_generation/build_ttl_xodr.py [options]

Output is written under this package (e.g. XODR_generation/generated/) by default.
See README in this folder for details.
"""

from .build_ttl_xodr import (
    build_connected_ttl_xodr,
    load_ttl_csv,
)

__all__ = [
    "build_connected_ttl_xodr",
    "load_ttl_csv",
]
