#!/usr/bin/env python

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

"""
Receive a Cluster end-effector target over OSC, solve IK, and drive an SO follower arm.

Example:

```shell
lerobot-cluster-ik \
  --robot.type=so101_follower \
  --robot.port=COM4 \
  --robot.id=my_so101 \
  --robot.max_relative_target=10 \
  --urdf_path=C:/Users/me/SO-ARM100/Simulation/SO101/so101_new_calib.urdf \
  --target_frame_name=gripper_frame_link \
  --host=127.0.0.1 \
  --recv_port=9000
```

Cluster is expected to send `/ik/target` with three float values: x, y, z.
By default the first received Cluster coordinate is used as the Cluster origin,
and the robot's current end-effector position is used as the robot origin.
"""

import logging
import math
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Any

import numpy as np

from lerobot.configs import parser
from lerobot.model import RobotKinematics
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_so_follower,
    make_robot_from_config,
    so_follower,
)
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.import_utils import _pythonosc_available, register_third_party_plugins, require_package
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, move_cursor_up

if _pythonosc_available:
    from pythonosc.dispatcher import Dispatcher
    from pythonosc.osc_server import ThreadingOSCUDPServer
else:
    Dispatcher = None
    ThreadingOSCUDPServer = None

logger = logging.getLogger(__name__)

DEFAULT_AXIS_MAP = ["z", "-x", "y"]


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def rotation_matrix_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm == 0.0:
        return np.eye(3, dtype=float)
    x, y, z = axis / axis_norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=float,
    )


def make_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rpy_to_matrix(rpy)
    transform[:3, 3] = xyz
    return transform


def parse_xyz_attr(element: ET.Element | None, attr_name: str, default: str = "0 0 0") -> np.ndarray:
    value = default if element is None else element.attrib.get(attr_name, default)
    parts = [float(part) for part in value.split()]
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values for {attr_name}, got {value!r}")
    return np.asarray(parts, dtype=float)


@dataclass
class URDFJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


class URDFKinematics:
    """Small position IK fallback for SO arms when placo is unavailable on Windows."""

    def __init__(
        self,
        urdf_path: str,
        target_frame_name: str = "gripper_frame_link",
        joint_names: list[str] | None = None,
    ):
        self.urdf_path = Path(urdf_path)
        self.target_frame_name = target_frame_name
        self.motor_names = [] if joint_names is None else list(joint_names)
        self.joints = self._load_chain()
        self.active_joint_names = [
            joint.name
            for joint in self.joints
            if joint.joint_type in {"revolute", "continuous"} and (not self.motor_names or joint.name in self.motor_names)
        ]
        self._active_indexes = [
            self.motor_names.index(name) for name in self.active_joint_names if name in self.motor_names
        ]

        if not self.active_joint_names:
            raise ValueError(f"No active joints found in URDF chain to {target_frame_name}.")

    def _load_chain(self) -> list[URDFJoint]:
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        child_to_joint: dict[str, URDFJoint] = {}

        for joint_element in root.findall("joint"):
            parent_element = joint_element.find("parent")
            child_element = joint_element.find("child")
            if parent_element is None or child_element is None:
                continue

            origin_element = joint_element.find("origin")
            limit_element = joint_element.find("limit")
            lower = None if limit_element is None or "lower" not in limit_element.attrib else float(limit_element.attrib["lower"])
            upper = None if limit_element is None or "upper" not in limit_element.attrib else float(limit_element.attrib["upper"])
            joint = URDFJoint(
                name=joint_element.attrib["name"],
                joint_type=joint_element.attrib.get("type", "fixed"),
                parent=parent_element.attrib["link"],
                child=child_element.attrib["link"],
                origin_xyz=parse_xyz_attr(origin_element, "xyz"),
                origin_rpy=parse_xyz_attr(origin_element, "rpy"),
                axis=parse_xyz_attr(joint_element.find("axis"), "xyz", default="0 0 1"),
                lower=lower,
                upper=upper,
            )
            child_to_joint[joint.child] = joint

        chain: list[URDFJoint] = []
        link = self.target_frame_name
        while link in child_to_joint:
            joint = child_to_joint[link]
            chain.append(joint)
            link = joint.parent
        if not chain:
            raise ValueError(f"Could not find target frame {self.target_frame_name!r} in {self.urdf_path}.")
        chain.reverse()
        return chain

    def _active_q_rad_from_full_deg(self, joint_pos_deg: np.ndarray) -> np.ndarray:
        if self.motor_names:
            return np.deg2rad(joint_pos_deg[self._active_indexes])
        return np.deg2rad(joint_pos_deg[: len(self.active_joint_names)])

    def _full_deg_from_active_q_rad(self, current_joint_pos: np.ndarray, active_q_rad: np.ndarray) -> np.ndarray:
        result = np.asarray(current_joint_pos, dtype=float).copy()
        if self.motor_names:
            for q_index, motor_index in enumerate(self._active_indexes):
                result[motor_index] = math.degrees(float(active_q_rad[q_index]))
            return result

        result[: len(active_q_rad)] = np.rad2deg(active_q_rad)
        return result

    def _clip_active_q(self, active_q_rad: np.ndarray) -> np.ndarray:
        clipped = active_q_rad.copy()
        active_i = 0
        for joint in self.joints:
            if joint.name not in self.active_joint_names:
                continue
            if joint.lower is not None:
                clipped[active_i] = max(clipped[active_i], joint.lower)
            if joint.upper is not None:
                clipped[active_i] = min(clipped[active_i], joint.upper)
            active_i += 1
        return clipped

    def _fk_active(self, active_q_rad: np.ndarray) -> np.ndarray:
        transform = np.eye(4, dtype=float)
        active_i = 0
        for joint in self.joints:
            transform = transform @ make_transform(joint.origin_xyz, joint.origin_rpy)
            if joint.name in self.active_joint_names:
                joint_rotation = np.eye(4, dtype=float)
                joint_rotation[:3, :3] = rotation_matrix_from_axis_angle(joint.axis, float(active_q_rad[active_i]))
                transform = transform @ joint_rotation
                active_i += 1
        return transform

    def forward_kinematics(self, joint_pos_deg: np.ndarray) -> np.ndarray:
        return self._fk_active(self._active_q_rad_from_full_deg(joint_pos_deg))

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,
        desired_ee_pose: np.ndarray,
        position_weight: float = 1.0,
        orientation_weight: float = 0.01,
    ) -> np.ndarray:
        del orientation_weight
        q = self._active_q_rad_from_full_deg(current_joint_pos)
        target_pos = np.asarray(desired_ee_pose[:3, 3], dtype=float)
        damping = 1e-3
        eps = 1e-4

        for _ in range(80):
            current_pos = self._fk_active(q)[:3, 3]
            error = (target_pos - current_pos) * position_weight
            if float(np.linalg.norm(error)) < 1e-4:
                break

            jacobian = np.zeros((3, len(q)), dtype=float)
            for joint_i in range(len(q)):
                q_eps = q.copy()
                q_eps[joint_i] += eps
                jacobian[:, joint_i] = (self._fk_active(q_eps)[:3, 3] - current_pos) / eps

            lhs = jacobian @ jacobian.T + (damping**2) * np.eye(3, dtype=float)
            dq = jacobian.T @ np.linalg.solve(lhs, error)
            q = self._clip_active_q(q + np.clip(dq, -0.08, 0.08))

        return self._full_deg_from_active_q_rad(current_joint_pos, q)


@dataclass
class ClusterIKConfig:
    robot: RobotConfig

    # Path to the SO101/SO100 URDF used by placo. The SO-ARM100 repository contains:
    # Simulation/SO101/so101_new_calib.urdf
    urdf_path: str
    target_frame_name: str = "gripper_frame_link"

    host: str = "127.0.0.1"
    recv_port: int = 9000
    osc_address: str = "/ik/target"

    fps: int = 30
    control_time_s: float | None = None
    stale_timeout_s: float = 0.5

    # Cluster/Unity is typically x=right, y=up, z=forward.
    # SO arm URDFs commonly use x=forward, y=left, z=up, so default maps:
    # robot_x=cluster_z, robot_y=-cluster_x, robot_z=cluster_y.
    axis_map: list[str] = field(default_factory=lambda: list(DEFAULT_AXIS_MAP))
    scale: float = 1.0
    cluster_origin_xyz: list[float] | None = None
    robot_origin_xyz: list[float] | None = None

    workspace_min_xyz: list[float] = field(default_factory=lambda: [-0.5, -0.5, -0.1])
    workspace_max_xyz: list[float] = field(default_factory=lambda: [0.5, 0.5, 0.7])
    max_ee_step_m: float = 0.02
    smoothing_alpha: float = 0.35
    position_weight: float = 1.0
    orientation_weight: float = 0.01

    # None keeps the current gripper position.
    gripper_pos: float | None = None
    max_joint_step_deg: float = 8.0
    display_data: bool = True


def _as_xyz(values: list[float] | tuple[float, float, float] | np.ndarray, name: str) -> np.ndarray:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values, got {values}.")
    arr = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain only finite values, got {values}.")
    return arr


def parse_axis_map(axis_map: list[str]) -> list[tuple[int, float]]:
    if len(axis_map) != 3:
        raise ValueError(f"axis_map must contain exactly 3 entries, got {axis_map}.")

    axis_to_index = {"x": 0, "y": 1, "z": 2}
    parsed: list[tuple[int, float]] = []
    for entry in axis_map:
        sign = -1.0 if entry.startswith("-") else 1.0
        axis = entry[1:] if entry.startswith("-") else entry
        if axis not in axis_to_index:
            raise ValueError(f"Invalid axis_map entry '{entry}'. Use x, y, z or -x, -y, -z.")
        parsed.append((axis_to_index[axis], sign))
    return parsed


@dataclass
class ClusterTargetMapper:
    axis_map: list[str]
    scale: float
    workspace_min_xyz: list[float]
    workspace_max_xyz: list[float]
    max_ee_step_m: float
    smoothing_alpha: float
    cluster_origin_xyz: list[float] | None = None
    robot_origin_xyz: list[float] | None = None

    _parsed_axis_map: list[tuple[int, float]] = field(init=False, repr=False)
    _cluster_origin: np.ndarray | None = field(default=None, init=False, repr=False)
    _robot_origin: np.ndarray | None = field(default=None, init=False, repr=False)
    _last_target: np.ndarray | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        self._parsed_axis_map = parse_axis_map(self.axis_map)
        self._workspace_min = _as_xyz(self.workspace_min_xyz, "workspace_min_xyz")
        self._workspace_max = _as_xyz(self.workspace_max_xyz, "workspace_max_xyz")
        if np.any(self._workspace_min > self._workspace_max):
            raise ValueError("workspace_min_xyz must be <= workspace_max_xyz on every axis.")
        if self.max_ee_step_m <= 0:
            raise ValueError("max_ee_step_m must be positive.")
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in the range (0, 1].")
        if self.cluster_origin_xyz is not None:
            self._cluster_origin = _as_xyz(self.cluster_origin_xyz, "cluster_origin_xyz")
        if self.robot_origin_xyz is not None:
            self._robot_origin = _as_xyz(self.robot_origin_xyz, "robot_origin_xyz")

    @property
    def cluster_origin(self) -> np.ndarray | None:
        return None if self._cluster_origin is None else self._cluster_origin.copy()

    @property
    def robot_origin(self) -> np.ndarray | None:
        return None if self._robot_origin is None else self._robot_origin.copy()

    def map(self, cluster_xyz: np.ndarray, default_robot_origin: np.ndarray) -> np.ndarray:
        cluster_xyz = _as_xyz(cluster_xyz, "cluster_xyz")
        default_robot_origin = _as_xyz(default_robot_origin, "default_robot_origin")

        if self._cluster_origin is None:
            self._cluster_origin = cluster_xyz.copy()
        if self._robot_origin is None:
            self._robot_origin = default_robot_origin.copy()

        cluster_delta = cluster_xyz - self._cluster_origin
        robot_delta = np.asarray(
            [sign * cluster_delta[index] for index, sign in self._parsed_axis_map],
            dtype=float,
        )
        raw_target = self._robot_origin + robot_delta * float(self.scale)
        bounded_target = np.clip(raw_target, self._workspace_min, self._workspace_max)

        if self._last_target is None:
            self._last_target = bounded_target
            return bounded_target.copy()

        smoothed = self._last_target + self.smoothing_alpha * (bounded_target - self._last_target)
        step = smoothed - self._last_target
        step_norm = float(np.linalg.norm(step))
        if step_norm > self.max_ee_step_m:
            smoothed = self._last_target + step * (self.max_ee_step_m / step_norm)

        self._last_target = smoothed
        return smoothed.copy()


class LatestOSCTargetReceiver:
    def __init__(self, host: str, port: int, address: str):
        require_package("python-osc", extra="hardware", import_name="pythonosc")
        self.host = host
        self.port = port
        self.address = address
        self._dispatcher: Dispatcher | None = None
        self._server: ThreadingOSCUDPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: np.ndarray | None = None
        self._last_message_time: float | None = None

    def _target_handler(self, address: str, *args: Any) -> None:
        if len(args) < 3:
            logger.warning("Ignoring OSC %s with fewer than 3 values: %s", address, args)
            return

        try:
            target = np.asarray([float(args[0]), float(args[1]), float(args[2])], dtype=float)
        except (TypeError, ValueError):
            logger.warning("Ignoring OSC %s with non-numeric values: %s", address, args)
            return

        if not np.all(np.isfinite(target)):
            logger.warning("Ignoring OSC %s with non-finite values: %s", address, args)
            return

        with self._lock:
            self._latest = target
            self._last_message_time = time.monotonic()

    def connect(self) -> None:
        self._dispatcher = Dispatcher()
        self._dispatcher.map(self.address, self._target_handler)
        self._server = ThreadingOSCUDPServer((self.host, self.port), self._dispatcher)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        logger.info("Listening for Cluster OSC target on udp://%s:%s%s", self.host, self.port, self.address)

    def latest(self) -> tuple[np.ndarray | None, float | None]:
        with self._lock:
            target = None if self._latest is None else self._latest.copy()
            last_message_time = self._last_message_time
        age = None if last_message_time is None else time.monotonic() - last_message_time
        return target, age

    def disconnect(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._server_thread is not None:
            self._server_thread.join(timeout=1.0)
        self._server = None
        self._server_thread = None
        self._dispatcher = None


def get_motor_names(robot: Robot) -> list[str]:
    if hasattr(robot, "bus") and hasattr(robot.bus, "motors"):
        return list(robot.bus.motors.keys())
    return [key.removesuffix(".pos") for key in robot.action_features if key.endswith(".pos")]


def observation_to_joints(observation: RobotObservation, motor_names: list[str]) -> np.ndarray:
    missing = [name for name in motor_names if f"{name}.pos" not in observation]
    if missing:
        raise KeyError(f"Observation is missing motor positions: {missing}")
    return np.asarray([float(observation[f"{name}.pos"]) for name in motor_names], dtype=float)


def make_action(
    q_target: np.ndarray,
    observation: RobotObservation,
    motor_names: list[str],
    gripper_pos: float | None,
) -> RobotAction:
    action: RobotAction = {}
    for i, name in enumerate(motor_names):
        if name == "gripper":
            value = float(observation["gripper.pos"] if gripper_pos is None else gripper_pos)
        else:
            value = float(q_target[i])
        action[f"{name}.pos"] = value
    return action


def limit_joint_step(q_target: np.ndarray, q_reference: np.ndarray, max_joint_step_deg: float) -> np.ndarray:
    if max_joint_step_deg <= 0:
        raise ValueError("max_joint_step_deg must be positive.")
    delta = np.clip(q_target - q_reference, -max_joint_step_deg, max_joint_step_deg)
    return q_reference + delta


def cluster_ik_loop(
    robot: Robot,
    receiver: LatestOSCTargetReceiver,
    kinematics: RobotKinematics,
    mapper: ClusterTargetMapper,
    cfg: ClusterIKConfig,
) -> None:
    motor_names = get_motor_names(robot)
    if "gripper" not in motor_names:
        logger.warning("No gripper motor found; gripper_pos will be ignored.")

    obs = robot.get_observation()
    q_curr = observation_to_joints(obs, motor_names)
    reference_pose = kinematics.forward_kinematics(q_curr)
    default_robot_origin = reference_pose[:3, 3].copy()
    start = time.perf_counter()
    wait_line_printed = False

    while True:
        loop_start = time.perf_counter()
        cluster_target, age = receiver.latest()

        if cluster_target is None:
            if not wait_line_printed:
                print(f"Waiting for OSC {cfg.osc_address} on udp://{cfg.host}:{cfg.recv_port} ...")
                wait_line_printed = True
            precise_sleep(max(1.0 / cfg.fps - (time.perf_counter() - loop_start), 0.0))
            if cfg.control_time_s is not None and time.perf_counter() - start >= cfg.control_time_s:
                return
            continue

        if wait_line_printed:
            print("Received first OSC target.")
            wait_line_printed = False

        if age is not None and age > cfg.stale_timeout_s:
            logger.debug("Cluster OSC target is stale by %.3fs; holding last robot goal.", age)
            precise_sleep(max(1.0 / cfg.fps - (time.perf_counter() - loop_start), 0.0))
            continue

        obs = robot.get_observation()
        q_observed = observation_to_joints(obs, motor_names)
        target_pos = mapper.map(cluster_target, default_robot_origin)

        desired_pose = reference_pose.copy()
        desired_pose[:3, 3] = target_pos

        q_target = kinematics.inverse_kinematics(
            q_curr,
            desired_pose,
            position_weight=cfg.position_weight,
            orientation_weight=cfg.orientation_weight,
        )
        q_target = limit_joint_step(q_target, q_observed, cfg.max_joint_step_deg)
        action = make_action(q_target, obs, motor_names, cfg.gripper_pos)
        sent_action = robot.send_action(action)
        q_curr = np.asarray([sent_action[f"{name}.pos"] for name in motor_names], dtype=float)

        if cfg.display_data:
            print(
                "cluster_xyz="
                f"{np.round(cluster_target, 4).tolist()} "
                f"robot_ee_xyz={np.round(target_pos, 4).tolist()} "
                f"age_ms={0.0 if age is None else age * 1e3:.1f}"
            )
            move_cursor_up(1)

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(1.0 / cfg.fps - dt_s, 0.0))

        if cfg.control_time_s is not None and time.perf_counter() - start >= cfg.control_time_s:
            return


@parser.wrap()
def cluster_ik(cfg: ClusterIKConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    require_package("python-osc", extra="hardware", import_name="pythonosc")

    if cfg.fps <= 0:
        raise ValueError("fps must be positive.")
    if getattr(cfg.robot, "use_degrees", True) is not True:
        raise ValueError("Cluster IK requires --robot.use_degrees=true because RobotKinematics uses degrees.")
    if not math.isfinite(cfg.scale) or cfg.scale <= 0:
        raise ValueError("scale must be a positive finite value.")

    robot = make_robot_from_config(cfg.robot)
    receiver = LatestOSCTargetReceiver(cfg.host, cfg.recv_port, cfg.osc_address)
    mapper = ClusterTargetMapper(
        axis_map=cfg.axis_map,
        scale=cfg.scale,
        cluster_origin_xyz=cfg.cluster_origin_xyz,
        robot_origin_xyz=cfg.robot_origin_xyz,
        workspace_min_xyz=cfg.workspace_min_xyz,
        workspace_max_xyz=cfg.workspace_max_xyz,
        max_ee_step_m=cfg.max_ee_step_m,
        smoothing_alpha=cfg.smoothing_alpha,
    )

    print(f"Connecting robot: {cfg.robot.type}")
    robot.connect()
    try:
        receiver.connect()
        try:
            kinematics = RobotKinematics(
                urdf_path=cfg.urdf_path,
                target_frame_name=cfg.target_frame_name,
                joint_names=get_motor_names(robot),
            )
        except ImportError as exc:
            logger.warning("Falling back to built-in URDF IK because RobotKinematics is unavailable: %s", exc)
            kinematics = URDFKinematics(
                urdf_path=cfg.urdf_path,
                target_frame_name=cfg.target_frame_name,
                joint_names=get_motor_names(robot),
            )
        cluster_ik_loop(robot, receiver, kinematics, mapper, cfg)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.disconnect()
        robot.disconnect()


def main() -> None:
    register_third_party_plugins()
    cluster_ik()


if __name__ == "__main__":
    main()
