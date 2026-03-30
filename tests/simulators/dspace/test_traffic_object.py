"""Unit tests for ModelDesk fellow Traffic Object path resolution (no ModelDesk required)."""

from scenic.simulators.dspace.modeldesk.traffic_object import resolve_try_set_element_path

_AVAIL = [
    "Vehicles\\GER_Hatchback_A.tro",
    "Vehicles\\GER_CompactCar_A.tro",
    "Construction\\Delineator.tro",
]


def test_resolve_exact_basename():
    assert (
        resolve_try_set_element_path(_AVAIL, "GER_Hatchback_A")
        == "Vehicles\\GER_Hatchback_A.tro"
    )


def test_resolve_full_path():
    p = "Vehicles\\GER_CompactCar_A.tro"
    assert resolve_try_set_element_path(_AVAIL, p) == p


def test_resolve_suffix_insensitive_basename():
    assert (
        resolve_try_set_element_path(_AVAIL, "ger_compactcar_a")
        == "Vehicles\\GER_CompactCar_A.tro"
    )


def test_resolve_unknown_returns_none():
    assert resolve_try_set_element_path(_AVAIL, "TotallyMissing_XYZ") is None


def test_resolve_prefers_vehicle_folder_on_substring():
    """When multiple paths contain the same substring, prefer Vehicles\\."""
    avail = [
        "Construction\\Arrow_delineator.tro",
        "Vehicles\\GER_Sedan_A.tro",
    ]
    assert (
        resolve_try_set_element_path(avail, "GER_Sedan_A")
        == "Vehicles\\GER_Sedan_A.tro"
    )
