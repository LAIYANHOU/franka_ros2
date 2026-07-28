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

#include <algorithm>
#include <memory>
#include <string>

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp>

namespace franka_robot_state_broadcaster {

FrankaRobotStateBroadcaster::FrankaRobotStateBroadcaster(
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state)
    : franka_robot_state_(std::move(franka_robot_state)) {}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = franka_robot_state_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params_ = param_listener_->get_params();
  auto this_node = get_node();
  auto robot_description = get_robot_description();
  if (robot_description.empty()) {
    RCLCPP_ERROR(this_node->get_logger(), "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }

  if (!franka_robot_state_) {
    std::string full_prefix = params_.arm_prefix.empty() ? "" : params_.arm_prefix + "_";
    std::string hw_interface_name = full_prefix + params_.robot_type + "/" + state_interface_name_;

    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        hw_interface_name, robot_description);
  }

  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  current_pose_stamped_publisher_ = this_node->create_publisher<geometry_msgs::msg::PoseStamped>(
      kCurrentPoseTopic, state_qos);
  last_desired_pose_stamped_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::PoseStamped>(kLastDesiredPoseTopic,
                                                                   state_qos);
  desired_end_effector_twist_stamped_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::TwistStamped>(kDesiredEETwist, state_qos);
  measured_joint_states_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kMeasuredJointStates, state_qos);
  external_wrench_in_stiffness_frame_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
          kExternalWrenchInStiffnessFrame, state_qos);
  external_wrench_in_base_frame_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::WrenchStamped>(kExternalWrenchInBaseFrame,
                                                                     state_qos);
  external_joint_torques_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kExternalJointTorques, state_qos);
  desired_joint_states_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kDesiredJointStates, state_qos);

  try {
    franka_state_publisher_ = this_node->create_publisher<franka_msgs::msg::FrankaRobotState>(
        "~/" + state_interface_name_, state_qos);

    franka_robot_state_->initialize_robot_state_msg(state_msg_);
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }

  const int update_rate = static_cast<int>(get_update_rate());
  if (update_rate <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Update rate is %d Hz, so no publish rate can be derived from it.", update_rate);
    return CallbackReturn::ERROR;
  }

  const int requested_rate =
      std::min(static_cast<int>(params_.convenience_publish_rate), update_rate);
  convenience_publish_skip_ = std::max(1, update_rate / requested_rate);
  const int effective_rate = update_rate / convenience_publish_skip_;
  if (effective_rate != requested_rate) {
    RCLCPP_WARN(get_node()->get_logger(),
                "convenience_publish_rate %d Hz does not evenly divide update rate %d Hz. "
                "Effective rate: %d Hz.",
                requested_rate, update_rate, effective_rate);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Convenience topics at %d Hz, full state at %d Hz",
              effective_rate, update_rate);

  if (!is_async()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Running on the controller manager's real-time loop. Building the state message "
                "there delays every command sent to the robot and raises the rate at which the "
                "robot refuses them. Set 'is_async: true' for this controller.");
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Could not claim the robot state interface. Check that 'robot_state' is listed "
                 "among this controller's state interfaces and that the hardware exports it.");
    return CallbackReturn::ERROR;
  }
  if (!franka_robot_state_->initialize_state_buffer()) {
    return CallbackReturn::ERROR;
  }
  convenience_counter_ = 0;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->reset_state_buffer();
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaRobotStateBroadcaster::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  state_msg_.header.stamp = time;

  if (!franka_robot_state_->get_values_as_message(state_msg_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get franka state via franka state interface.");
    return controller_interface::return_type::ERROR;
  }

  // Full state always publishes at the update rate (1kHz).
  franka_state_publisher_->publish(state_msg_);

  if (++convenience_counter_ >= convenience_publish_skip_) {
    convenience_counter_ = 0;
    current_pose_stamped_publisher_->publish(state_msg_.o_t_ee);
    last_desired_pose_stamped_publisher_->publish(state_msg_.o_t_ee_d);
    desired_end_effector_twist_stamped_publisher_->publish(state_msg_.o_dp_ee_d);
    external_wrench_in_base_frame_publisher_->publish(state_msg_.o_f_ext_hat_k);
    external_wrench_in_stiffness_frame_publisher_->publish(state_msg_.k_f_ext_hat_k);
    measured_joint_states_publisher_->publish(state_msg_.measured_joint_state);
    external_joint_torques_publisher_->publish(state_msg_.tau_ext_hat_filtered);
    desired_joint_states_publisher_->publish(state_msg_.desired_joint_state);
  }

  return controller_interface::return_type::OK;
}

}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)
