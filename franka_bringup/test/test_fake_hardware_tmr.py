#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""Fake-hardware integration test for the TMR v0.2 mobile base."""

from pathlib import Path
import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

import launch_testing
import launch_testing.actions

import rclpy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState

STARTUP_TIMEOUT = 10.0
JOINT_STATE_TIMEOUT = 15.0

# TMR drive joints (4 joints: front/rear steering + front/rear driving)
EXPECTED_TMR_JOINTS = {
    'tmrv0_2_joint_0',
    'tmrv0_2_joint_1',
    'tmrv0_2_joint_2',
    'tmrv0_2_joint_3',
}


def generate_test_description():
    """Launch TMR with fake hardware and joint_state_broadcaster."""
    tmr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'tmrv0_2.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_config_file': str(
                Path(__file__).resolve().parent / 'config'
                / 'test_fake_hardware_tmr.config.yaml'
            ),
            'controller_name': 'swerve_drive_controller',
        }.items(),
    )

    return (
        LaunchDescription(
            [
                tmr_launch,
                TimerAction(
                    period=STARTUP_TIMEOUT,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ]
        ),
        {'tmr_launch': tmr_launch},
    )


class TestFakeHardwareTmr(unittest.TestCase):
    """Verify TMR mobile base launches and publishes joint states."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create a test node."""
        self.node = rclpy.create_node('fake_hardware_tmr_test')

    def tearDown(self):
        """Destroy the test node."""
        self.node.destroy_node()

    def wait_for_joint_states(self, *, min_joints, timeout_sec):
        """Wait for /joint_states with at least min_joints."""
        received = []

        def callback(msg):
            if len(msg.name) >= min_joints:
                received.append(msg)

        sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            callback,
            qos_profile_sensor_data,
        )

        try:
            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.2)
                if received:
                    return received[-1]
        finally:
            self.node.destroy_subscription(sub)

        self.fail(
            f'Did not receive /joint_states with >= {min_joints} joints '
            f'within {timeout_sec}s'
        )

    def test_tmr_publishes_drive_joints(self):
        """Verify TMR drive joints appear in /joint_states."""
        joint_state = self.wait_for_joint_states(
            min_joints=4,
            timeout_sec=JOINT_STATE_TIMEOUT,
        )
        self.assertTrue(
            EXPECTED_TMR_JOINTS.issubset(set(joint_state.name)),
            f'Expected TMR drive joints not found. '
            f'Expected: {EXPECTED_TMR_JOINTS}, Got: {set(joint_state.name)}',
        )


@launch_testing.post_shutdown_test()
class TestFakeHardwareTmrShutdown(unittest.TestCase):
    """Verify TMR launch exits without unexpected errors."""

    IGNORED_PATTERNS = [
        'swerve_drive_controller',
        'service call timed out',
        'pal_statistics',
        'joint_state_publisher',
    ]

    def test_has_no_error(self, proc_output):
        """Check no unexpected ERROR messages in output."""
        error_lines = []
        for event in proc_output:
            text = event.text if hasattr(event, 'text') else str(event)
            if isinstance(text, bytes):
                text = text.decode('utf-8', errors='replace')
            for line in text.splitlines():
                if '[ERROR]' not in line:
                    continue
                if any(p in line.lower() for p in self.IGNORED_PATTERNS):
                    continue
                error_lines.append(line)

        self.assertEqual(
            error_lines, [],
            f'Found unexpected [ERROR] log messages: {error_lines}',
        )
