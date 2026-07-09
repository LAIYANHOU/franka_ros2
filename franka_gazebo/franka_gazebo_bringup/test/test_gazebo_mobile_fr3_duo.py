# Copyright (c) 2026 Franka Robotics GmbH
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

"""Gazebo integration test for the Mobile FR3 Duo launch."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gazebo_test_utils import (  # noqa: E402
    GazeboShutdownTestBase, GazeboTestBase, make_gazebo_test_description,
)

import launch_testing  # noqa: E402


EXPECTED_ARM_JOINTS = {
    f'{arm_prefix}_fr3v2_joint{joint_index}'
    for arm_prefix in ('left', 'right')
    for joint_index in range(1, 8)
}


def generate_test_description():
    """Launch the Mobile FR3 Duo Gazebo example in headless mode."""
    return make_gazebo_test_description('gazebo_mobile_fr3_duo_example.launch.py')


class TestGazeboMobileFr3Duo(GazeboTestBase):
    """Verify the Mobile FR3 Duo launch publishes both arm joint states."""

    NODE_NAME = 'gazebo_mobile_fr3_duo_test'
    EXPECTED_JOINTS = EXPECTED_ARM_JOINTS

    def test_publishes_dual_arm_joint_states(self):
        """Check that both mobile FR3 arms appear in /joint_states."""
        joint_state = self.wait_for_joint_states(min_joints=14)
        self.assertTrue(
            EXPECTED_ARM_JOINTS.issubset(set(joint_state.name)),
            f'Expected Mobile FR3 Duo arm joints not found. Got: {joint_state.name}',
        )


@launch_testing.post_shutdown_test()
class TestGazeboMobileFr3DuoShutdown(GazeboShutdownTestBase):
    """Verify the Mobile FR3 Duo launch exits without unexpected errors."""

    IGNORED_ERROR_PATTERNS = [
        'service call timed out',
        'swerve_drive_controller',
    ]
