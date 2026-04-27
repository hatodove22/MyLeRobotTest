#!/usr/bin/env python

import logging
import threading
import time
from functools import cached_property
from typing import Any

from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.import_utils import _pythonosc_available, require_package

from ..teleoperator import Teleoperator
from .config_osc_joint_teleop import OscJointTeleopConfig

if _pythonosc_available:
    from pythonosc.dispatcher import Dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
    from pythonosc.udp_client import SimpleUDPClient
else:
    Dispatcher = None
    ThreadingOSCUDPServer = None
    SimpleUDPClient = None

logger = logging.getLogger(__name__)

OSC_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class OscJointTeleop(Teleoperator):
    config_class = OscJointTeleopConfig
    name = "osc_joint_teleop"

    def __init__(self, config: OscJointTeleopConfig):
        require_package("python-osc", extra="hardware", import_name="pythonosc")
        super().__init__(config)
        self.config = config
        self._dispatcher: Dispatcher | None = None
        self._server: ThreadingOSCUDPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._client: SimpleUDPClient | None = None
        self._is_connected = False
        self._last_message_time: float | None = None
        self._lock = threading.Lock()
        self._action: dict[str, float] = {joint: 0.0 for joint in OSC_JOINT_NAMES}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in OSC_JOINT_NAMES}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in OSC_JOINT_NAMES}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def _mark_message(self) -> None:
        self._last_message_time = time.monotonic()

    def _update_joint(self, joint: str, value: float) -> None:
        with self._lock:
            self._action[joint] = float(value)
            self._mark_message()

    def _joint_handler(self, address: str, *args: Any) -> None:
        if not args:
            return
        joint = address.rsplit("/", maxsplit=1)[-1]
        if joint not in self._action:
            return
        self._update_joint(joint, float(args[0]))

    def _all_joints_handler(self, address: str, *args: Any) -> None:
        if len(args) < len(OSC_JOINT_NAMES):
            return

        with self._lock:
            for joint, value in zip(OSC_JOINT_NAMES, args[: len(OSC_JOINT_NAMES)], strict=True):
                self._action[joint] = float(value)
            self._mark_message()

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self._dispatcher = Dispatcher()
        self._dispatcher.map("/lerobot/joints", self._all_joints_handler)
        for joint in OSC_JOINT_NAMES:
            self._dispatcher.map(f"/lerobot/joint/{joint}", self._joint_handler)

        self._server = ThreadingOSCUDPServer((self.config.host, self.config.recv_port), self._dispatcher)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        if self.config.send_host is not None and self.config.send_port is not None:
            self._client = SimpleUDPClient(self.config.send_host, self.config.send_port)

        self._is_connected = True
        logger.info("OSC joint teleop listening on udp://%s:%s", self.config.host, self.config.recv_port)

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        if self.config.stale_timeout_s is not None and self._last_message_time is not None:
            age = time.monotonic() - self._last_message_time
            if age > self.config.stale_timeout_s:
                logger.debug("OSC joint teleop is stale by %.3fs; returning the last known action.", age)

        with self._lock:
            return {f"{joint}.pos": value for joint, value in self._action.items()}

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if self._client is None:
            return

        for key, value in feedback.items():
            if not isinstance(value, (int, float)):
                continue
            if key.endswith(".pos"):
                joint = key.removesuffix(".pos")
                self._client.send_message(f"/lerobot/feedback/{joint}", float(value))
            elif key.endswith(".load"):
                joint = key.removesuffix(".load")
                self._client.send_message(f"/lerobot/feedback/{joint}/load", float(value))
            elif key.endswith(".current"):
                joint = key.removesuffix(".current")
                self._client.send_message(f"/lerobot/feedback/{joint}/current", float(value))

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._server_thread is not None:
            self._server_thread.join(timeout=1.0)
        self._server = None
        self._server_thread = None
        self._dispatcher = None
        self._client = None
        self._is_connected = False
