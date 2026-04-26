"""XODR <-> dSPACE RD frame calibration loader.

Phase A surgical glue. Phase B will subsume this into ``geometry/frames.py`` along with
the GPS-anchored API. For now it carries just enough to translate xy positions between
the Scenic XODR frame (used by ``param map`` + scenario ``at (...)`` placements) and
the dSPACE RD frame (used by ``Pos_x_Vehicle_CoorSys_E`` readback and the empirical
centerline ``ttl_main_road.csv``).

Background. For the OLD map (``LagunaSeca.xodr``), Scenic XODR-xy and dSPACE RD-xy
were the same frame (the XODR was auto-generated from the RD-aligned empirical TTL).
For the NEW map (``LGS_v1.xodr``), the XODR was MathWorks-generated independently;
``tools/frames/verify_xodr_rd_alignment.py`` measured a pure translation between the
two frames. The calibration JSON records that translation per XODR.

If no calibration JSON exists for the loaded XODR, the translation is ``(0, 0)`` and
both functions act as identity (preserving the OLD-map behavior).
"""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

# Cache: xodr_path -> (dx, dy) translation, or (0.0, 0.0) if no calibration.
_TRANSLATION_CACHE: dict = {}


def _calibration_path_for(xodr_path: Path) -> Path:
    """Calibration JSON sibling of an XODR: <xodr_dir>/<basename>_gps_rd_calibration.json."""
    return xodr_path.parent / f"{xodr_path.stem}_gps_rd_calibration.json"


def get_xodr_to_rd_translation(xodr_path) -> Tuple[float, float]:
    """Return (dx, dy) such that ``rd_xy = xodr_xy + (dx, dy)``.

    Looks for a sibling ``<basename>_gps_rd_calibration.json`` next to ``xodr_path``.
    Cached per path. Returns ``(0.0, 0.0)`` (identity) if no calibration is found
    or the JSON has no ``translation_xy_xodr_to_rd`` key.
    """
    if xodr_path is None:
        return (0.0, 0.0)
    p = Path(xodr_path)
    key = str(p.resolve())
    cached = _TRANSLATION_CACHE.get(key)
    if cached is not None:
        return cached

    calib_path = _calibration_path_for(p)
    if not calib_path.is_file():
        _TRANSLATION_CACHE[key] = (0.0, 0.0)
        return (0.0, 0.0)
    try:
        with open(calib_path, "r", encoding="utf-8") as f:
            d = json.load(f)
        model = d.get("model", "translation")
        if model != "translation":
            logger.warning(
                "[FrameCalibration] %s has model=%r; only 'translation' is supported in Phase A; "
                "treating as identity. Phase B will support full affine.",
                calib_path.name, model,
            )
            _TRANSLATION_CACHE[key] = (0.0, 0.0)
            return (0.0, 0.0)
        t = d.get("translation_xy_xodr_to_rd")
        if not (isinstance(t, (list, tuple)) and len(t) == 2):
            logger.warning(
                "[FrameCalibration] %s missing valid 'translation_xy_xodr_to_rd'; treating as identity.",
                calib_path.name,
            )
            _TRANSLATION_CACHE[key] = (0.0, 0.0)
            return (0.0, 0.0)
        dx, dy = float(t[0]), float(t[1])
        _TRANSLATION_CACHE[key] = (dx, dy)
        logger.info("[FrameCalibration] Loaded %s: XODR->RD translation = (%+.3f, %+.3f) m",
                    calib_path.name, dx, dy)
        return (dx, dy)
    except (OSError, json.JSONDecodeError, ValueError) as e:
        logger.warning("[FrameCalibration] Failed to load %s (%s); treating as identity.",
                       calib_path, e)
        _TRANSLATION_CACHE[key] = (0.0, 0.0)
        return (0.0, 0.0)


def xodr_to_rd(x: float, y: float, xodr_path) -> Tuple[float, float]:
    """Translate (x, y) from Scenic XODR-xy to dSPACE RD-xy."""
    dx, dy = get_xodr_to_rd_translation(xodr_path)
    return (float(x) + dx, float(y) + dy)


def rd_to_xodr(x: float, y: float, xodr_path) -> Tuple[float, float]:
    """Translate (x, y) from dSPACE RD-xy to Scenic XODR-xy."""
    dx, dy = get_xodr_to_rd_translation(xodr_path)
    return (float(x) - dx, float(y) - dy)


def get_xodr_path_from_sim(sim) -> Optional[Path]:
    """Best-effort lookup of the active XODR path for a DSpaceSimulation."""
    scene = getattr(sim, "scene", None)
    params = getattr(scene, "params", None) or {}
    p = params.get("map")
    if p is None:
        return None
    try:
        return Path(p)
    except (TypeError, ValueError):
        return None
