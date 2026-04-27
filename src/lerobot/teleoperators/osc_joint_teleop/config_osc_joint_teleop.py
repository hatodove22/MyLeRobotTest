#!/usr/bin/env python

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("osc_joint_teleop")
@dataclass
class OscJointTeleopConfig(TeleoperatorConfig):
    host: str = "127.0.0.1"
    recv_port: int = 9000
    send_host: str | None = None
    send_port: int | None = None
    use_degrees: bool = True
    stale_timeout_s: float | None = None
