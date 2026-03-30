# -*- coding: utf-8 -*-
"""ModelDesk Fellow *Traffic Object* (vehicle .tro asset) via COM ``TrafficObjectType``.

See ``src/scenic/simulators/dspace/README.md`` section *Fellow vehicle asset (Traffic Object)*.
"""

from __future__ import annotations

from typing import Any, List, Optional

# Vehicle asset applied to every fellow Scenic creates in ModelDesk (UI: Traffic Object).
DEFAULT_FELLOW_TRAFFIC_OBJECT_BASENAME = "IAC_Car_AIRacing"


def resolve_try_set_element_path(avail: List[str], user: str) -> Optional[str]:
    """Map a short name (e.g. ``IAC_Car_AIRacing``) to a full ``AvailableElements`` path."""
    user = user.strip()
    if not user or not avail:
        return None
    if user in avail:
        return user
    for a in avail:
        if a.endswith("\\" + user) or a.endswith("/" + user):
            return a
        norm = a.replace("\\", "/")
        if norm.endswith("/" + user):
            return a
    base_user = user.replace(".tro", "")
    base_user = base_user.split("\\")[-1].split("/")[-1]
    for a in avail:
        a_base = a.split("\\")[-1].split("/")[-1].replace(".tro", "")
        if a_base.lower() == base_user.lower():
            return a
    ulow = user.lower()
    vehicle_hits = [a for a in avail if ulow in a.lower() and "vehicles" in a.lower()]
    if vehicle_hits:
        return vehicle_hits[0]
    for a in avail:
        if ulow in a.lower():
            return a
    return None


def _active_element_best_string(ae: Any) -> Optional[str]:
    if ae is None:
        return None
    for attr in ("Name", "RelativePath", "FullName", "Path", "StringValue"):
        if hasattr(ae, attr):
            try:
                v = getattr(ae, attr)
                if v is not None and str(v).strip():
                    return str(v).strip()
            except Exception:
                continue
    return None


def active_traffic_object_element_hint(tot: Any) -> str:
    """Human-readable hint for the current ``ActiveElement`` (e.g. library path)."""
    try:
        return _active_element_best_string(tot.ActiveElement) or ""
    except Exception:
        return ""


def current_traffic_object_avail_key(tot: Any) -> Optional[str]:
    """Return the ``AvailableElements`` entry matching ``ActiveElement``."""
    try:
        avail = list(tot.AvailableElements)
    except Exception:
        return None
    try:
        ae = tot.ActiveElement
    except Exception:
        return None
    token = _active_element_best_string(ae)
    if not token:
        return None
    return resolve_try_set_element_path(avail, token)


def apply_fellow_traffic_object(
    fellow: Any,
    asset_basename: str = DEFAULT_FELLOW_TRAFFIC_OBJECT_BASENAME,
    *,
    log_prefix: str = "[ModelDesk]",
) -> bool:
    """Set Fellow ``TrafficObjectType`` via ``Activate(full_path)``."""
    try:
        tot = getattr(fellow, "TrafficObjectType", None)
        if tot is None:
            print(f"{log_prefix} Fellow has no TrafficObjectType; skipping traffic asset.")
            return False
        if not hasattr(tot, "Activate") or not hasattr(tot, "AvailableElements"):
            print(
                f"{log_prefix} TrafficObjectType missing Activate/AvailableElements; "
                "skipping traffic asset."
            )
            return False
        avail = list(tot.AvailableElements)
        path = resolve_try_set_element_path(avail, asset_basename)
        if not path:
            print(
                f"{log_prefix} No library path for traffic asset {asset_basename!r} "
                "(see ModelDesk Traffic Object browser)."
            )
            return False
        tot.Activate(path)
        print(f"{log_prefix} Fellow traffic object -> {path!r}")
        return True
    except Exception as e:
        print(
            f"{log_prefix} Could not set fellow traffic object ({asset_basename!r}): {e}"
        )
        return False
