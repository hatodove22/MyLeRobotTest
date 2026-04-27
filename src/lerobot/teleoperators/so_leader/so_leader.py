# !/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_so_leader import SOLeaderTeleopConfig

logger = logging.getLogger(__name__)


class SOLeader(Teleoperator):
    """Generic SO leader base for SO-100/101/10X teleoperators."""

    config_class = SOLeaderTeleopConfig
    name = "so_leader"

    def __init__(self, config: SOLeaderTeleopConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self._feedback_goal_deltas: dict[str, float] = {}
        self._feedback_torque_enabled = False
        self._feedback_load_boost = 0.25

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        if not self.config.feedback_enabled:
            return {}

        features = {f"{motor}.pos": float for motor in self.config.feedback_motors}
        if self.config.feedback_use_load:
            features.update({f"{motor}.load": float for motor in self.config.feedback_motors})
        return features

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        full_turn_motor = "wrist_roll"
        unknown_range_motors = [motor for motor in self.bus.motors if motor != full_turn_motor]
        print(
            f"Move all joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        range_mins[full_turn_motor] = 0
        range_maxes[full_turn_motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self._disable_feedback()
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def _resolve_feedback_param(self, param: float | dict[str, float], motor: str) -> float:
        if isinstance(param, dict):
            return float(param.get(motor, 0.0))
        return float(param)

    def _enable_feedback(self) -> None:
        if self._feedback_torque_enabled or not self.config.feedback_enabled:
            return
        self.bus.enable_torque(self.config.feedback_motors)
        self._feedback_torque_enabled = True

    def _disable_feedback(self) -> None:
        if not self._feedback_torque_enabled:
            self._feedback_goal_deltas.clear()
            return

        try:
            self.bus.disable_torque(self.config.feedback_motors)
        finally:
            self._feedback_goal_deltas.clear()
            self._feedback_torque_enabled = False

    def send_feedback(self, feedback: dict[str, float]) -> None:
        if not self.config.feedback_enabled:
            self._disable_feedback()
            return

        follower_targets = {
            motor: float(feedback[f"{motor}.pos"])
            for motor in self.config.feedback_motors
            if f"{motor}.pos" in feedback
        }
        if not follower_targets:
            self._disable_feedback()
            return

        self._enable_feedback()
        leader_positions = self.bus.sync_read("Present_Position", self.config.feedback_motors)
        goal_positions: dict[str, float] = {}
        for motor, follower_position in follower_targets.items():
            leader_position = float(leader_positions[motor])
            error = follower_position - leader_position
            deadband = self._resolve_feedback_param(self.config.feedback_deadband_deg, motor)
            if abs(error) <= deadband:
                desired_delta = 0.0
            else:
                desired_delta = error * self._resolve_feedback_param(self.config.feedback_gain, motor)
                if self.config.feedback_use_load:
                    load = abs(float(feedback.get(f"{motor}.load", 0.0)))
                    desired_delta *= 1.0 + min(load / 1000.0, 1.0) * self._feedback_load_boost

            max_delta = self._resolve_feedback_param(self.config.feedback_max_delta, motor)
            desired_delta = max(-max_delta, min(max_delta, desired_delta))

            previous_delta = self._feedback_goal_deltas.get(motor, 0.0)
            rate_limit = self._resolve_feedback_param(self.config.feedback_rate_limit, motor)
            delta_step = desired_delta - previous_delta
            if abs(delta_step) > rate_limit:
                delta_step = rate_limit if delta_step > 0 else -rate_limit

            new_delta = previous_delta + delta_step
            self._feedback_goal_deltas[motor] = new_delta
            goal_positions[motor] = leader_position + new_delta

        if goal_positions:
            self.bus.sync_write("Goal_Position", goal_positions)

    @check_if_not_connected
    def disconnect(self) -> None:
        self._disable_feedback()
        self.bus.disconnect()
        logger.info(f"{self} disconnected.")


SO100Leader = SOLeader
SO101Leader = SOLeader
