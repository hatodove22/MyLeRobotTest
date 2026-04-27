#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@dataclass
class SOLeaderConfig:
    """Base configuration class for SO Leader teleoperators."""

    # Port to connect to the arm
    port: str

    # Whether to use degrees for angles
    use_degrees: bool = True

    # Enable gentle pseudo-force feedback on selected joints.
    feedback_enabled: bool = False

    # Joints that receive pseudo-force feedback.
    feedback_motors: list[str] = field(default_factory=lambda: ["shoulder_lift", "elbow_flex", "wrist_flex"])

    # Scale of follower-leader position error converted into leader resistance.
    feedback_gain: float | dict[str, float] = 0.2

    # Ignore small position errors to keep the leader free near the target.
    feedback_deadband_deg: float | dict[str, float] = 2.0

    # Clamp the per-cycle virtual offset applied to the leader joints.
    feedback_max_delta: float | dict[str, float] = 4.0

    # Rate limit the virtual offset so resistance ramps in smoothly.
    feedback_rate_limit: float | dict[str, float] = 0.5

    # Optionally scale feedback slightly with follower load.
    feedback_use_load: bool = False


@TeleoperatorConfig.register_subclass("so101_leader")
@TeleoperatorConfig.register_subclass("so100_leader")
@dataclass
class SOLeaderTeleopConfig(TeleoperatorConfig, SOLeaderConfig):
    pass


SO100LeaderConfig = SOLeaderTeleopConfig
SO101LeaderConfig = SOLeaderTeleopConfig
