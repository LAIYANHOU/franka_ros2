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

"""Shared utilities for Gazebo integration tests."""

import subprocess
import time
import unittest

from launch import (
    actions,
    launch_description_sources,
    LaunchDescription,
    substitutions,
)

import launch_ros.substitutions

import launch_testing
import launch_testing.actions

import rclpy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


def ensure_gz_sim_not_running():
    """Kill any remaining Gazebo and related ROS processes between tests."""
    subprocess.run(['pkill', '-2', '-f', '^gz sim'], check=False)
    time.sleep(2)
    subprocess.run(['pkill', '-9', '-f', '^gz sim'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'ruby.*gz'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'controller_manager'], check=False)
    subprocess.run(
        ['pkill', '-9', '-f', 'robot_state_publisher'], check=False
    )
    time.sleep(2)


def output_text(output_event):
    """Decode a captured process IO event to text."""
    text = getattr(output_event, 'text', None)
    if text is None:
        return str(output_event)
    if isinstance(text, bytes):
        return text.decode('utf-8', errors='replace')
    return str(text)


def make_gazebo_test_description(launch_file, *, startup_duration=12.0):
    """Create a standard Gazebo test LaunchDescription.

    Args:
        launch_file: Name of the launch file in franka_gazebo_bringup.
        startup_duration: Seconds to wait before marking ready.

    Returns:
        Tuple of (LaunchDescription, context dict) for launch_testing.
    """
    ensure_gz_sim_not_running()

    launch_description = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        'franka_gazebo_bringup'
                    ),
                    'launch',
                    launch_file,
                ]
            )
        ),
        launch_arguments={
            'gz_args': '-r -s --headless-rendering',
            'rviz': 'false',
        }.items(),
    )

    return (
        LaunchDescription(
            [
                launch_description,
                actions.TimerAction(
                    period=startup_duration,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ]
        ),
        {'launch_description': launch_description},
    )


class GazeboTestBase(unittest.TestCase):
    """Base class for Gazebo behavioral integration tests.

    Subclasses must define:
        NODE_NAME: str — unique ROS node name for the test.
        EXPECTED_JOINTS: set — joint names to verify in /joint_states.
        JOINT_STATE_TIMEOUT: float — seconds to wait for joint states.
    """

    NODE_NAME = 'gazebo_test'
    EXPECTED_JOINTS = set()
    JOINT_STATE_TIMEOUT = 60.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        ensure_gz_sim_not_running()

    def setUp(self):
        self.node = rclpy.create_node(self.NODE_NAME)

    def tearDown(self):
        self.node.destroy_node()

    def wait_for_joint_states(self, *, min_joints, timeout_sec=None):
        """Wait for /joint_states with at least min_joints."""
        if timeout_sec is None:
            timeout_sec = self.JOINT_STATE_TIMEOUT
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


class GazeboShutdownTestBase(unittest.TestCase):
    """Base class for Gazebo post-shutdown error checks.

    Subclasses may override IGNORED_ERROR_PATTERNS.
    """

    IGNORED_ERROR_PATTERNS = ['service call timed out']

    def setUp(self):
        ensure_gz_sim_not_running()

    def test_has_no_error(self, proc_output):
        """Check no unexpected [ERROR] messages appear in launch output."""
        error_lines = []
        for output_event in proc_output:
            for line in output_text(output_event).splitlines():
                if '[ERROR]' not in line:
                    continue
                if any(
                    p in line.lower()
                    for p in self.IGNORED_ERROR_PATTERNS
                ):
                    continue
                error_lines.append(line)

        self.assertEqual(
            error_lines,
            [],
            f'Found unexpected [ERROR] log messages: {error_lines}',
        )
