"""Tests for the Odin/sensor pose correction and projection helpers."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from pick_action.pose_alignment import (
    correct_pose,
    correct_pose_from_odin,
    odin_to_field,
    project_target_to_gripper_line,
    robot_to_gripper_pose,
    yaw_from_quaternion,
)

from pick_action import pose_alignment


class TestYawFromQuaternion:
    def test_identity_is_zero(self) -> None:
        quaternion = SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
        assert yaw_from_quaternion(quaternion) == pytest.approx(0.0)

    def test_ninety_degrees_about_z(self) -> None:
        quaternion = SimpleNamespace(w=math.cos(math.pi / 4), x=0.0, y=0.0, z=math.sin(math.pi / 4))
        assert yaw_from_quaternion(quaternion) == pytest.approx(math.pi / 2)


class TestOdinToField:
    def test_subtracts_origin(self) -> None:
        assert odin_to_field(1.0, 2.0, 0.5, -1.0) == pytest.approx((0.5, 3.0))


class TestCorrectPose:
    def test_formula_with_zeroed_constants(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(pose_alignment, "YAW_OFFSET_RAD", 0.0)
        monkeypatch.setattr(pose_alignment, "M_SENSOR_X", 0.0)
        monkeypatch.setattr(pose_alignment, "M_SENSOR_Y", 0.0)
        monkeypatch.setattr(pose_alignment, "N_SENSOR_X", 0.0)
        monkeypatch.setattr(pose_alignment, "N_SENSOR_Y", 0.0)
        x_m, y_m, yaw_rad = correct_pose(1000.0, 2000.0, 0.0)
        assert x_m == pytest.approx(1.0)
        assert y_m == pytest.approx(2.0)
        assert yaw_rad == pytest.approx(0.0)

    def test_preserves_input_yaw(self) -> None:
        _, _, yaw_rad = correct_pose(500.0, 500.0, 0.3)
        assert yaw_rad == pytest.approx(0.3)


class TestRobotToGripperPose:
    def test_forward_offset_along_heading(self) -> None:
        pose = robot_to_gripper_pose(1.0, 2.0, 0.0, 0.5, 0.0, 0.0)
        assert pose == pytest.approx((1.5, 2.0, 0.0))

    def test_left_offset_perpendicular_to_heading(self) -> None:
        pose = robot_to_gripper_pose(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        assert pose == pytest.approx((0.0, 1.0, 0.0))

    def test_applies_yaw_offset(self) -> None:
        _, _, yaw_rad = robot_to_gripper_pose(0.0, 0.0, 0.2, 0.0, 0.0, -0.5)
        assert yaw_rad == pytest.approx(-0.3)


class TestProjectTargetToGripperLine:
    def test_projection_along_and_lateral(self) -> None:
        projection = project_target_to_gripper_line(0.0, 0.0, 0.0, 1.0, 0.5)
        assert projection.along_offset_m == pytest.approx(1.0)
        assert projection.projection_x_m == pytest.approx(1.0)
        assert projection.projection_y_m == pytest.approx(0.0)
        assert projection.lateral_error_m == pytest.approx(0.5)

    def test_negative_along_when_target_behind(self) -> None:
        projection = project_target_to_gripper_line(0.0, 0.0, 0.0, -1.0, 0.0)
        assert projection.along_offset_m == pytest.approx(-1.0)


class TestCorrectPoseFromOdin:
    def test_returns_expected_keys_and_direct_sign(self) -> None:
        result = correct_pose_from_odin(
            sensor_3_mm=1000.0,
            sensor_5_mm=1000.0,
            odin_x_m=0.0,
            odin_y_m=0.0,
            odin_yaw_rad=0.0,
            field_origin_x_m=0.0,
            field_origin_y_m=0.0,
            gripper_forward_m=0.0,
            gripper_left_m=0.0,
            gripper_yaw_offset_rad=0.0,
            target_x_m=1.0,
            target_y_m=0.0,
            direct=-1.0,
        )
        assert "gripper_forward_move_m" in result
        assert result["direct"] == pytest.approx(-1.0)
        assert result["gripper_forward_move_m"] == pytest.approx(
            -result["raw_gripper_forward_move_m"]
        )
