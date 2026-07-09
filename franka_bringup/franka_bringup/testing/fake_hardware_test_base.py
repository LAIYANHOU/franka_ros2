# Copyright 2026 Franka Robotics GmbH
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

"""Base class and utilities for fake-hardware integration tests."""
# Provides FakeHardwareTestBase: handles ROS lifecycle, node management,
# ControllerServiceClient, joint_state waiting, and error assertion.
# See franka_bringup/test/README.md for full architecture documentation.

import time
import unittest

from franka_bringup.testing.controller_service_client import ControllerServiceClient

import rclpy
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


JOINT_STATE_BROADCASTER = 'joint_state_broadcaster'

# Default timeouts (tuned for CI environments)
SERVICE_DISCOVERY_TIMEOUT = 20.0
CONTROLLER_STATE_TIMEOUT = 30.0
JOINT_STATE_TIMEOUT = 10.0


class FakeHardwareTestBase(unittest.TestCase):
    """Base class for fake-hardware integration tests."""

    # Subclasses should override NODE_NAME for unique ROS node naming.

    NODE_NAME = 'fake_hardware_test'

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(self.NODE_NAME)
        self.controller_client = ControllerServiceClient(self.node)

    def tearDown(self):
        self.controller_client.destroy()
        self.node.destroy_node()

    def wait_for_stack_ready(self):
        """Wait for controller_manager services and joint_state_broadcaster."""
        self.assertTrue(
            self.controller_client.wait_for_services(
                timeout_sec=SERVICE_DISCOVERY_TIMEOUT
            ),
            'controller_manager services did not become available',
        )
        self.assertTrue(
            self.controller_client.wait_for_controller_state(
                JOINT_STATE_BROADCASTER, ['active'],
                timeout_sec=CONTROLLER_STATE_TIMEOUT,
            ),
            'joint_state_broadcaster did not become active',
        )

    def wait_for_joint_states(self, *, min_joints=1, timeout_sec=JOINT_STATE_TIMEOUT):
        """Wait for /joint_states with at least `min_joints` joints published."""
        received = []

        def callback(msg):
            received.append(msg)

        sub = self.node.create_subscription(
            JointState, '/joint_states', callback, qos_profile_sensor_data,
        )

        try:
            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.2)
                if received and len(received[-1].name) >= min_joints:
                    return received[-1]
        finally:
            self.node.destroy_subscription(sub)

        self.fail(
            f'Did not receive /joint_states with >= {min_joints} joints '
            f'within {timeout_sec}s'
        )

    def assert_no_errors_in_output(self, proc_output, *, ignore_patterns=None):
        """Assert no [ERROR] messages appear in launch output."""
        # ignore_patterns: list of substrings to exempt from error checking
        ignore_patterns = ignore_patterns or []
        output_lines = []
        for event in proc_output:
            text = (
                event.text.decode('utf-8', errors='replace')
                if isinstance(event.text, bytes)
                else event.text
            )
            output_lines.append(text)

        all_output = '\n'.join(output_lines)
        error_lines = [
            line for line in all_output.split('\n')
            if '[ERROR]' in line
            and not any(pattern in line for pattern in ignore_patterns)
        ]
        self.assertEqual(
            error_lines, [],
            f'Found unexpected [ERROR] log messages: {error_lines}',
        )
