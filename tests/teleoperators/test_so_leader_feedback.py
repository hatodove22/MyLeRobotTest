#!/usr/bin/env python

from unittest.mock import MagicMock

from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
from lerobot.teleoperators.so_leader.so_leader import SOLeader


def test_feedback_features_disabled_by_default():
    leader = SOLeader(SOLeaderTeleopConfig(id="leader", port="COM1"))
    assert leader.feedback_features == {}


def test_send_feedback_enables_torque_once_and_rate_limits_goal_position():
    leader = SOLeader(
        SOLeaderTeleopConfig(
            id="leader",
            port="COM1",
            feedback_enabled=True,
            feedback_motors=["elbow_flex"],
            feedback_gain=0.5,
            feedback_deadband_deg=0.0,
            feedback_max_delta=5.0,
            feedback_rate_limit=1.0,
        )
    )
    leader.bus = MagicMock()
    leader.bus.is_connected = True
    leader.bus.sync_read.return_value = {"elbow_flex": 10.0}

    leader.send_feedback({"elbow_flex.pos": 20.0})

    leader.bus.enable_torque.assert_called_once_with(["elbow_flex"])
    leader.bus.sync_write.assert_called_once_with("Goal_Position", {"elbow_flex": 11.0})

    leader.bus.sync_write.reset_mock()
    leader.send_feedback({"elbow_flex.pos": 20.0})

    leader.bus.enable_torque.assert_called_once_with(["elbow_flex"])
    leader.bus.sync_write.assert_called_once_with("Goal_Position", {"elbow_flex": 12.0})
