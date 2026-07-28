// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "franka_msgs/msg/franka_robot_state.hpp"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster_parameters.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

namespace franka_robot_state_broadcaster {

/**
 * Publishes the full Franka robot state and a set of convenience topics.
 *
 * Publishing the ROS message might cost a few dozen microseconds, which is too much to spend
 * between the arrival of a robot state and the departure of the matching command. The
 * controller therefore defaults to running asynchronously: the controller manager only
 * signals it, and both the message construction and the publishing happen on the
 * controller's own thread. Another comparable approach would be to move the publishing directly
 * from a separate controller directly to franka_hardware_interface + realtime publishers, which
 * spawn their own thread for publishing.
 */
class FrankaRobotStateBroadcaster : public controller_interface::ControllerInterface {
 public:
  explicit FrankaRobotStateBroadcaster(
      std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state = nullptr);

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::string state_interface_name_{"robot_state"};
  std::shared_ptr<rclcpp::Publisher<franka_msgs::msg::FrankaRobotState>> franka_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>
      current_pose_stamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>
      last_desired_pose_stamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>>
      desired_end_effector_twist_stamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>
      external_wrench_in_base_frame_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>
      external_wrench_in_stiffness_frame_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
      external_joint_torques_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> measured_joint_states_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> desired_joint_states_publisher_;

  const std::string kCurrentPoseTopic = "~/current_pose";
  const std::string kLastDesiredPoseTopic = "~/last_desired_pose";
  const std::string kDesiredEETwist = "~/desired_end_effector_twist";
  const std::string kMeasuredJointStates = "~/measured_joint_states";
  const std::string kExternalWrenchInStiffnessFrame = "~/external_wrench_in_stiffness_frame";
  const std::string kExternalWrenchInBaseFrame = "~/external_wrench_in_base_frame";
  const std::string kExternalJointTorques = "~/external_joint_torques";
  const std::string kDesiredJointStates = "~/desired_joint_states";

  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  // Filled in place every cycle and published from the same thread, so a single message
  // suffices and stays resident in cache.
  franka_msgs::msg::FrankaRobotState state_msg_;

  // Convenience topics publish every N-th cycle, where N is derived in on_configure()
  // from the convenience_publish_rate parameter.
  int convenience_publish_skip_{1};
  int convenience_counter_{0};
};
}  // namespace franka_robot_state_broadcaster
