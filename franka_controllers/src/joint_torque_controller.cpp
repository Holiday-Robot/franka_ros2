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

#include <franka_controllers/default_robot_behavior_utils.hpp>
#include <franka_controllers/joint_torque_controller.hpp>
#include <franka_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

namespace franka_controllers {

JointTorqueController::JointTorqueController()
    : hday_robot_description_(hday_robot_name, hday_robot_type),
      hday_robot_(hday_robot_description_) {
  hday_target_frames_ = hday_robot_.getEndeffectorLinks();
}

controller_interface::InterfaceConfiguration
JointTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration JointTorqueController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::return_type JointTorqueController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  auto joint_command_msg_ptr = joint_command_msg_external_point_ptr_.readFromRT();
  if (!joint_command_msg_ptr || !(*joint_command_msg_ptr)) {
    // Buffer is empty or contains nullptr
  } else {
    // Buffer contains a valid message
    auto joint_command_msg = *joint_command_msg_ptr;
    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(joint_command_msg->effort[i]);
    }
  }

  updateJointStates();
  publish_joint_state(joint_state);

  for (size_t i = 0; i < joint_state.position.size(); i++) {
    hday_cur_joints_["joint" + std::to_string(i + 1)] = joint_state.position[i];
  }
  auto fk_results = hday_robot_.forwardKinematics(hday_cur_joints_, hday_target_frames_, "link0");
  cur_ef_pose_ = fk_results[hday_target_frames_[0]].pose.matrix();
  cur_ef_position_ = cur_ef_pose_.block<3, 1>(0, 3);
  cur_ef_orientation_ = Eigen::Quaterniond(cur_ef_pose_.block<3, 3>(0, 0));

  publish_ef_pose(cur_ef_position_, cur_ef_orientation_);

  return controller_interface::return_type::OK;
}

CallbackReturn JointTorqueController::on_init() {
  try {
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(1000ms);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  // subscribe
  joint_command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/hday/rt_franka/joint_torque_command", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTorqueController::joint_command_callback, this, std::placeholders::_1));

  // publish
  joint_state_base_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "/hday/rt_franka/joint_state", rclcpp::SystemDefaultsQoS());
  joint_state_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
          joint_state_base_publisher_);

  ef_pose_base_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/hday/rt_franka/ef_pose", rclcpp::SystemDefaultsQoS());
  ef_pose_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
          ef_pose_base_publisher_);

  joint_state_publisher_->lock();
  joint_state_publisher_->msg_.name.resize(num_joints);
  joint_state_publisher_->msg_.position.resize(num_joints);
  joint_state_publisher_->msg_.velocity.resize(num_joints);
  joint_state_publisher_->msg_.effort.resize(num_joints);
  joint_state_publisher_->unlock();

  joint_state.header.stamp = get_node()->now();
  joint_state.name.resize(num_joints);
  joint_state.position.resize(num_joints);
  joint_state.velocity.resize(num_joints);
  joint_state.effort.resize(num_joints);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  joint_command_msg_external_point_ptr_.writeFromNonRT(
      std::shared_ptr<sensor_msgs::msg::JointState>());
  updateJointStates();
  return CallbackReturn::SUCCESS;
}

void JointTorqueController::joint_command_callback(
    const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
  if (!validate_effort_msg(*msg)) {
    return;
  }
  joint_command_msg_external_point_ptr_.writeFromNonRT(msg);
};

bool JointTorqueController::validate_effort_msg(const sensor_msgs::msg::JointState& command) const {
  if (command.name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming command.");
    return false;
  }

  if (command.name.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint name size mismatch");
    return false;
  }

  if (command.effort.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Effort command size mismatch");
    return false;
  }

  return true;
}

void JointTorqueController::updateJointStates() {
  joint_state.header.stamp = get_node()->now();
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(3 * i);
    const auto& velocity_interface = state_interfaces_.at(3 * i + 1);
    const auto& effort_interface = state_interfaces_.at(3 * i + 2);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    assert(effort_interface.get_interface_name() == "effort");

    joint_state.position[i] = position_interface.get_value();
    joint_state.velocity[i] = velocity_interface.get_value();
    joint_state.effort[i] = effort_interface.get_value();
  }
}

void JointTorqueController::publish_joint_state(const sensor_msgs::msg::JointState& joint_state) {
  if (joint_state_publisher_ && joint_state_publisher_->trylock()) {
    joint_state_publisher_->msg_.header.stamp = joint_state.header.stamp;
    // for (int i = 0; i < num_joints; ++i) {
    //   joint_state_publisher_->msg_.name[i] = arm_id_ + "_joint" + std::to_string(i + 1);
    // }
    for (int i = 0; i < num_joints; ++i) {
      joint_state_publisher_->msg_.name[i] = "joint" + std::to_string(i + 1);
    }
    joint_state_publisher_->msg_.position = joint_state.position;
    joint_state_publisher_->msg_.velocity = joint_state.velocity;
    joint_state_publisher_->msg_.effort = joint_state.effort;
  }
  joint_state_publisher_->unlockAndPublish();
}

void JointTorqueController::publish_ef_pose(const Eigen::Vector3d ef_position,
                                            Eigen::Quaterniond ef_orientation) {
  if (ef_pose_publisher_ && ef_pose_publisher_->trylock()) {
    ef_pose_publisher_->msg_.header.stamp = joint_state.header.stamp;
    ef_pose_publisher_->msg_.pose.position.x = ef_position(0);
    ef_pose_publisher_->msg_.pose.position.y = ef_position(1);
    ef_pose_publisher_->msg_.pose.position.z = ef_position(2);
    ef_pose_publisher_->msg_.pose.orientation.x = ef_orientation.x();
    ef_pose_publisher_->msg_.pose.orientation.y = ef_orientation.y();
    ef_pose_publisher_->msg_.pose.orientation.z = ef_orientation.z();
    ef_pose_publisher_->msg_.pose.orientation.w = ef_orientation.w();
  }
  ef_pose_publisher_->unlockAndPublish();
}

}  // namespace franka_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_controllers::JointTorqueController,
                       controller_interface::ControllerInterface)
