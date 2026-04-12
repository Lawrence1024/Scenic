"""Tests for evaluation-only OBB separation (racing eval_geometry)."""

import math

import pytest

from scenic.domains.racing.eval_geometry import (
    IAC_DALLARA_LENGTH_M,
    IAC_DALLARA_WIDTH_M,
    classify_eval_contact,
    eval_dspace_dist_object_1_valid,
    obb_separation_distance_m,
)


def test_iac_constants_are_inches_converted():
    assert IAC_DALLARA_LENGTH_M == pytest.approx(192 * 0.0254)
    assert IAC_DALLARA_WIDTH_M == pytest.approx(76 * 0.0254)


def test_obb_separation_parallel_same_heading():
    # Two 4×2 m boxes, heading 0, centers 10 m apart along x → gap 10 - 2 - 2 = 6
    d = obb_separation_distance_m(0, 0, 0.0, 4.0, 2.0, 10.0, 0.0, 0.0, 4.0, 2.0)
    assert d == pytest.approx(6.0)


def test_obb_overlap_is_zero():
    d = obb_separation_distance_m(0, 0, 0.0, 4.0, 2.0, 1.0, 0.0, 0.0, 4.0, 2.0)
    assert d == pytest.approx(0.0)


def test_obb_rotated_90_degrees():
    # Same geometry as parallel test but both rotated 90° (length along +y in world)
    d = obb_separation_distance_m(
        0, 0, math.pi / 2, 4.0, 2.0, 0.0, 10.0, math.pi / 2, 4.0, 2.0
    )
    assert d == pytest.approx(6.0)


def test_eval_dspace_invalid_sentinels():
    assert eval_dspace_dist_object_1_valid(None) is False
    assert eval_dspace_dist_object_1_valid(-1.0) is False
    assert eval_dspace_dist_object_1_valid(0.0) is True
    assert eval_dspace_dist_object_1_valid(1.5) is True


def test_classify_eval_contact_overlap_obb():
    risk, flags = classify_eval_contact(0.0, None)
    assert risk == "overlap"
    assert flags["overlap_obb"] is True


def test_classify_eval_contact_sensor_close():
    risk, flags = classify_eval_contact(50.0, 0.3)
    assert risk == "near"
    assert flags["near_sensor"] is True


def test_classify_eval_contact_insufficient():
    risk, _ = classify_eval_contact(None, -1.0)
    assert risk == "insufficient_data"
