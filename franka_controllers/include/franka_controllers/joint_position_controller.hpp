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

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "common/types/joint_state.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "robot_model/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_controllers {

class JointPositionController : public controller_interface::ControllerInterface {
 public:
  JointPositionController();
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  std::string robot_description_;
  const int num_joints = 7;
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};

  bool initialization_flag_{true};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_base_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
      joint_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ef_pose_base_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
      ef_pose_publisher_;

  void joint_command_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
  void publish_joint_state(const sensor_msgs::msg::JointState& joint_state);
  void publish_ef_pose(const Eigen::Vector3d ef_position, Eigen::Quaterniond ef_orientation);
  bool validate_position_msg(const sensor_msgs::msg::JointState& command) const;
  void updateJointStates();

  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>
      joint_command_msg_external_point_ptr_;

  sensor_msgs::msg::JointState joint_state;

  std::string hday_robot_name = "fr3";
  std::string hday_robot_type = "default";
  hday::robot_parser::Description hday_robot_description_;
  hday::robot_model::RobotModel hday_robot_;
  std::vector<std::string> hday_target_frames_;
  hday::JointState hday_cur_joints_;
  Eigen::MatrixXd cur_ef_pose_;
  Eigen::Vector3d cur_ef_position_;
  Eigen::Quaterniond cur_ef_orientation_;
};

}  // namespace franka_controllers
