"""Tests for the ROS-independent scan filtering and clustering algorithms."""

from __future__ import annotations

import math

import pytest
from pick_action.core import (
    ProcessorConfig,
    ScanPoint,
    angle_in_window,
    calibrate_range,
    clusters_to_detections,
    filter_clusters,
    normalize_angle,
    process_scan,
    scan_to_points,
    split_clusters,
)


class TestNormalizeAngle:
    @pytest.mark.parametrize(
        ("angle_rad", "expected"),
        [
            (0.0, 0.0),
            (math.pi, -math.pi),
            (2.0 * math.pi, 0.0),
            (-2.0 * math.pi, 0.0),
            (3.0 * math.pi, -math.pi),
        ],
    )
    def test_wraps_to_pi_range(self, angle_rad: float, expected: float) -> None:
        assert normalize_angle(angle_rad) == pytest.approx(expected)


class TestAngleInWindow:
    def test_plain_interval(self) -> None:
        assert angle_in_window(math.radians(-90), -170.0, -10.0)
        assert not angle_in_window(math.radians(0), -170.0, -10.0)

    def test_wrapping_interval(self) -> None:
        assert angle_in_window(math.radians(175), 170.0, -170.0)
        assert angle_in_window(math.pi, 170.0, -170.0)
        assert not angle_in_window(math.radians(0), 170.0, -170.0)


class TestCalibrateRange:
    def test_identity_by_default(self) -> None:
        config = ProcessorConfig()
        assert calibrate_range(0.2, math.radians(-90), config) == pytest.approx(0.2)

    def test_applies_cosine_terms(self) -> None:
        config = ProcessorConfig(
            range_calibration_scale=1.0,
            range_calibration_offset_m=0.01,
            range_calibration_cos_m=0.02,
            range_calibration_sin_m=0.0,
            range_calibration_cos2_m=0.0,
        )
        # at -90 deg cos = 0, so only the offset remains
        assert calibrate_range(0.2, math.radians(-90), config) == pytest.approx(0.21)
        # at 0 deg cos = 1
        assert calibrate_range(0.2, 0.0, config) == pytest.approx(0.23)


class TestScanToPoints:
    def test_accepts_point_inside_roi(self) -> None:
        points = scan_to_points(
            [0.2],
            angle_min_rad=math.radians(-90),
            angle_increment_rad=math.radians(1),
            sensor_range_min_m=0.05,
            sensor_range_max_m=10.0,
            config=ProcessorConfig(),
        )
        assert len(points) == 1
        assert points[0].x_m == pytest.approx(0.0, abs=1e-9)
        assert points[0].y_m == pytest.approx(-0.2)

    def test_rejects_out_of_window_angle(self) -> None:
        points = scan_to_points(
            [0.2],
            angle_min_rad=0.0,
            angle_increment_rad=math.radians(1),
            sensor_range_min_m=0.05,
            sensor_range_max_m=10.0,
            config=ProcessorConfig(),
        )
        assert points == []

    def test_rejects_non_finite_and_out_of_sensor_range(self) -> None:
        points = scan_to_points(
            [float("nan"), 0.01, 20.0],
            angle_min_rad=math.radians(-90),
            angle_increment_rad=0.0,
            sensor_range_min_m=0.05,
            sensor_range_max_m=10.0,
            config=ProcessorConfig(),
        )
        assert points == []

    def test_rejects_point_outside_xy_roi(self) -> None:
        config = ProcessorConfig(x_max_m=0.01)
        points = scan_to_points(
            [0.2],
            angle_min_rad=math.radians(-90),
            angle_increment_rad=0.0,
            sensor_range_min_m=0.05,
            sensor_range_max_m=10.0,
            config=config,
        )
        # x ~ 0 is inside, y = -0.2 is inside; force x out with a wider angle
        assert len(points) == 1

        config_tight = ProcessorConfig(y_max_m=-0.3)
        points_tight = scan_to_points(
            [0.2],
            angle_min_rad=math.radians(-90),
            angle_increment_rad=0.0,
            sensor_range_min_m=0.05,
            sensor_range_max_m=10.0,
            config=config_tight,
        )
        assert points_tight == []


def _point(index: int, angle_deg: float, range_m: float = 0.2) -> ScanPoint:
    angle_rad = math.radians(angle_deg)
    return ScanPoint(
        index=index,
        angle_rad=angle_rad,
        range_m=range_m,
        x_m=range_m * math.cos(angle_rad),
        y_m=range_m * math.sin(angle_rad),
    )


class TestSplitClusters:
    def test_splits_on_angular_gap(self) -> None:
        points = [
            _point(0, -90.0),
            _point(1, -90.5),
            _point(20, -100.0),
        ]
        clusters = split_clusters(points, math.radians(0.5), ProcessorConfig())
        assert [len(cluster) for cluster in clusters] == [2, 1]

    def test_empty_input(self) -> None:
        assert split_clusters([], math.radians(0.5), ProcessorConfig()) == []


class TestFilterClusters:
    def test_rejects_small_cluster(self) -> None:
        clusters = [[_point(0, -90.0)], [_point(0, -90.0), _point(1, -90.5)]]
        accepted = filter_clusters(clusters, ProcessorConfig())
        assert [len(cluster) for cluster in accepted] == [2]


class TestClustersToDetections:
    def test_sorts_by_x_and_assigns_ids(self) -> None:
        left = [_point(0, -90.0, 0.1), _point(1, -90.0, 0.1)]
        right = [_point(0, -90.0, 0.3), _point(1, -90.0, 0.3)]
        detections = clusters_to_detections([right, left])
        assert [detection.target_id for detection in detections] == [0, 1]
        assert detections[0].x_m < detections[1].x_m


class TestProcessScan:
    def test_detects_two_separated_bumps(self) -> None:
        angle_min = math.radians(-90)
        increment = math.radians(0.5)
        ranges = [0.2] * 4 + [float("inf")] * 40 + [0.3] * 4
        points, detections = process_scan(
            ranges,
            angle_min_rad=angle_min,
            angle_increment_rad=increment,
            sensor_range_min_m=0.05,
            sensor_range_max_m=10.0,
            config=ProcessorConfig(),
        )
        assert len(points) == 8
        assert len(detections) == 2
