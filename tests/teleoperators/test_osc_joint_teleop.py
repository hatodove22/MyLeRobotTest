#!/usr/bin/env python

import socket
import time

import pytest

pytest.importorskip("pythonosc", reason="python-osc is required for OSC teleoperator tests")

from pythonosc.udp_client import SimpleUDPClient

from lerobot.teleoperators.osc_joint_teleop import OSC_JOINT_NAMES, OscJointTeleop
from lerobot.teleoperators.osc_joint_teleop.config_osc_joint_teleop import OscJointTeleopConfig


def _get_free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _wait_for(predicate, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for OSC message to be processed.")


def test_osc_joint_teleop_defaults_to_zero_action():
    teleop = OscJointTeleop(OscJointTeleopConfig(id="osc_test", recv_port=_get_free_udp_port()))
    teleop.connect()
    try:
        assert teleop.get_action() == {f"{joint}.pos": 0.0 for joint in OSC_JOINT_NAMES}
    finally:
        teleop.disconnect()


def test_osc_joint_teleop_updates_single_joint():
    port = _get_free_udp_port()
    teleop = OscJointTeleop(OscJointTeleopConfig(id="osc_test", recv_port=port))
    teleop.connect()
    try:
        client = SimpleUDPClient("127.0.0.1", port)
        client.send_message("/lerobot/joint/elbow_flex", 12.5)

        def has_value() -> bool:
            return teleop.get_action()["elbow_flex.pos"] == pytest.approx(12.5)

        _wait_for(has_value)
        assert teleop.get_action()["elbow_flex.pos"] == pytest.approx(12.5)
    finally:
        teleop.disconnect()


def test_osc_joint_teleop_updates_all_joints():
    port = _get_free_udp_port()
    teleop = OscJointTeleop(OscJointTeleopConfig(id="osc_test", recv_port=port))
    teleop.connect()
    try:
        client = SimpleUDPClient("127.0.0.1", port)
        values = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        client.send_message("/lerobot/joints", values)

        def has_values() -> bool:
            action = teleop.get_action()
            return all(action[f"{joint}.pos"] == pytest.approx(value) for joint, value in zip(OSC_JOINT_NAMES, values, strict=True))

        _wait_for(has_values)
        assert teleop.get_action() == {
            f"{joint}.pos": pytest.approx(value) for joint, value in zip(OSC_JOINT_NAMES, values, strict=True)
        }
    finally:
        teleop.disconnect()
