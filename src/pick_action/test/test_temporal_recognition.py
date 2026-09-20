"""Tests for temporal spatial voting and target selection."""

from __future__ import annotations

import math

import pytest
from pick_action.core import ScanPoint
from pick_action.temporal_recognition import (
    StableComponent,
    TemporalGrid,
    arrangement_alignment_error,
    arrangement_line_equation,
    arrangement_spacing_metrics,
    compensate_polar_coordinates,
    compensate_target_centers,
    estimate_arrangement_axis,
    select_target_components,
    split_targets_have_plausible_parent,
)


def component(
    x_m: float,
    y_m: float,
    *,
    span: float = 0.02,
    cells: int = 4,
    occupancy: float = 0.5,
) -> StableComponent:
    return StableComponent(
        x_m=x_m,
        y_m=y_m,
        cell_count=cells,
        span_x_m=span,
        span_y_m=span,
        peak_occupancy=occupancy,
    )


def point(x_m: float, y_m: float, index: int = 0) -> ScanPoint:
    return ScanPoint(
        index=index,
        angle_rad=math.atan2(y_m, x_m),
        range_m=math.hypot(x_m, y_m),
        x_m=x_m,
        y_m=y_m,
    )


class TestTemporalGrid:
    def test_accumulates_stable_cell(self) -> None:
        grid = TemporalGrid(resolution_m=0.01)
        for _ in range(10):
            grid.add_frame([point(0.2, -0.1), point(0.205, -0.1)])
        components = grid.stable_components(
            minimum_occupancy=0.5, minimum_cells=1, connection_radius_cells=1
        )
        assert len(components) == 1
        assert components[0].peak_occupancy == pytest.approx(1.0)

    def test_ignores_transient_cell(self) -> None:
        grid = TemporalGrid(resolution_m=0.01)
        for index in range(10):
            frame = [point(0.2, -0.1)]
            if index == 0:
                frame.append(point(0.5, -0.2))
            grid.add_frame(frame)
        components = grid.stable_components(
            minimum_occupancy=0.5, minimum_cells=1, connection_radius_cells=1
        )
        assert len(components) == 1

    def test_empty_grid(self) -> None:
        assert TemporalGrid().stable_components() == []


class TestEstimateArrangementAxis:
    def test_horizontal_row(self) -> None:
        axis = estimate_arrangement_axis([component(0.0, 0.0), component(0.1, 0.0)])
        assert axis == pytest.approx((1.0, 0.0), abs=1e-6)

    def test_vertical_row_has_deterministic_sign(self) -> None:
        axis = estimate_arrangement_axis([component(0.0, 0.0), component(0.0, 0.1)])
        assert axis == pytest.approx((0.0, 1.0), abs=1e-6)

    def test_single_component_defaults_to_x(self) -> None:
        assert estimate_arrangement_axis([component(0.0, 0.0)]) == (1.0, 0.0)


class TestCompensateTargetCenters:
    def test_applies_along_and_normal_offsets(self) -> None:
        corrected = compensate_target_centers(
            [component(0.0, 0.0), component(0.1, 0.0)],
            [0.01, 0.01],
            [0.02, 0.02],
        )
        assert corrected[0].x_m == pytest.approx(0.01)
        assert corrected[0].y_m == pytest.approx(0.02)

    def test_length_mismatch_raises(self) -> None:
        with pytest.raises(ValueError):
            compensate_target_centers([component(0.0, 0.0)], [0.01], [])

    def test_empty_returns_empty(self) -> None:
        assert compensate_target_centers([], [], []) == []


class TestCompensatePolarCoordinates:
    def test_zero_coefficients_are_identity(self) -> None:
        corrected = compensate_polar_coordinates([component(0.2, -0.1)], [0.0] * 6, [0.0] * 6)
        assert corrected[0].x_m == pytest.approx(0.2)
        assert corrected[0].y_m == pytest.approx(-0.1)

    def test_wrong_coefficient_count_raises(self) -> None:
        with pytest.raises(ValueError):
            compensate_polar_coordinates([component(0.2, -0.1)], [0.0], [0.0] * 6)

    def test_non_positive_scale_raises(self) -> None:
        with pytest.raises(ValueError):
            compensate_polar_coordinates(
                [component(0.2, -0.1)], [0.0] * 6, [0.0] * 6, range_scale_m=0.0
            )


class TestSelectTargetComponents:
    def test_selects_expected_count_from_row(self) -> None:
        row = [component(0.0, 0.0), component(0.05, 0.0), component(0.10, 0.0)]
        selected = select_target_components(row, 3, maximum_component_span_m=0.05)
        assert len(selected) == 3
        assert [item.x_m for item in selected] == pytest.approx([0.0, 0.05, 0.10])

    def test_single_target_prefers_compact_component(self) -> None:
        small = component(0.0, 0.0, span=0.01)
        large = component(0.1, 0.0, span=0.05)
        selected = select_target_components([large, small], 1, maximum_component_span_m=0.06)
        assert selected == [small]

    def test_returns_empty_when_not_enough_candidates(self) -> None:
        assert select_target_components([component(0.0, 0.0)], 3) == []

    def test_rejects_oversized_component(self) -> None:
        assert select_target_components([component(0.0, 0.0, span=0.5)], 1, 0.1) == []


class TestSplitTargetsHavePlausibleParent:
    def test_accepts_targets_inside_one_parent(self) -> None:
        parent = component(0.05, 0.0, span=0.1)
        targets = [component(0.025, 0.0), component(0.075, 0.0)]
        assert split_targets_have_plausible_parent([parent], targets, 2, 0.12)

    def test_rejects_targets_outside_parent(self) -> None:
        parent = component(0.05, 0.0, span=0.1)
        targets = [component(0.5, 0.0), component(0.6, 0.0)]
        assert not split_targets_have_plausible_parent([parent], targets, 2, 0.12)

    def test_requires_matching_count(self) -> None:
        parent = component(0.05, 0.0, span=0.1)
        assert not split_targets_have_plausible_parent([parent], [component(0.05, 0.0)], 2, 0.12)


class TestArrangementMetrics:
    def test_line_equation_of_horizontal_row(self) -> None:
        line = arrangement_line_equation(
            [component(0.0, 0.0), component(0.1, 0.0), component(0.2, 0.0)]
        )
        assert line == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)

    def test_line_equation_requires_component(self) -> None:
        with pytest.raises(ValueError):
            arrangement_line_equation([])

    def test_alignment_error_zero_for_collinear_row(self) -> None:
        row = [component(0.0, 0.0), component(0.1, 0.0), component(0.2, 0.0)]
        assert arrangement_alignment_error(row) == pytest.approx(0.0)

    def test_spacing_metrics_for_even_row(self) -> None:
        row = [component(0.0, 0.0), component(0.1, 0.0), component(0.2, 0.0)]
        mean_spacing, deviation = arrangement_spacing_metrics(row)
        assert mean_spacing == pytest.approx(0.1)
        assert deviation == pytest.approx(0.0)

    def test_spacing_metrics_requires_two_components(self) -> None:
        with pytest.raises(ValueError):
            arrangement_spacing_metrics([component(0.0, 0.0)])
