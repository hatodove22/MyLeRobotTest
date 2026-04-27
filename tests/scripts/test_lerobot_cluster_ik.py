#!/usr/bin/env python

import numpy as np
import pytest

from lerobot.scripts.lerobot_cluster_ik import (
    ClusterTargetMapper,
    limit_joint_step,
    parse_axis_map,
)


def test_parse_axis_map_supports_signed_axes():
    assert parse_axis_map(["z", "-x", "y"]) == [(2, 1.0), (0, -1.0), (1, 1.0)]


def test_parse_axis_map_rejects_invalid_axis():
    with pytest.raises(ValueError, match="Invalid axis_map"):
        parse_axis_map(["z", "yaw", "y"])


def test_cluster_target_mapper_anchors_first_message_to_current_ee():
    mapper = ClusterTargetMapper(
        axis_map=["z", "-x", "y"],
        scale=2.0,
        workspace_min_xyz=[-1.0, -1.0, -1.0],
        workspace_max_xyz=[1.0, 1.0, 1.0],
        max_ee_step_m=1.0,
        smoothing_alpha=1.0,
    )
    robot_origin = np.array([0.2, 0.0, 0.3])

    first = mapper.map(np.array([10.0, 20.0, 30.0]), robot_origin)
    second = mapper.map(np.array([10.1, 20.2, 30.3]), robot_origin)

    np.testing.assert_allclose(first, robot_origin)
    np.testing.assert_allclose(second, [0.8, -0.2, 0.7])


def test_cluster_target_mapper_clips_step_after_smoothing():
    mapper = ClusterTargetMapper(
        axis_map=["x", "y", "z"],
        scale=1.0,
        cluster_origin_xyz=[0.0, 0.0, 0.0],
        robot_origin_xyz=[0.0, 0.0, 0.0],
        workspace_min_xyz=[-10.0, -10.0, -10.0],
        workspace_max_xyz=[10.0, 10.0, 10.0],
        max_ee_step_m=0.1,
        smoothing_alpha=1.0,
    )

    first = mapper.map(np.array([0.0, 0.0, 0.0]), np.zeros(3))
    second = mapper.map(np.array([1.0, 0.0, 0.0]), np.zeros(3))

    np.testing.assert_allclose(first, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(second, [0.1, 0.0, 0.0])


def test_limit_joint_step_clips_each_joint_independently():
    q_target = np.array([20.0, -20.0, 2.0])
    q_reference = np.array([0.0, 0.0, 0.0])

    limited = limit_joint_step(q_target, q_reference, max_joint_step_deg=5.0)

    np.testing.assert_allclose(limited, [5.0, -5.0, 2.0])
