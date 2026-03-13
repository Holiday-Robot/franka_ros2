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

#include <franka_controllers/joint_position_controller.hpp>
#include <franka_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 8, Eigen::Dynamic> Matrix8Xd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

using namespace std::chrono;
namespace franka_controllers {

JointPositionController::JointPositionController()
    : hday_robot_description_(hday_robot_name, hday_robot_type),
      hday_robot_(hday_robot_description_) {
  hday_target_frames_ = hday_robot_.getEndeffectorLinks();

  Eigen::VectorXd pos_lb, pos_ub, vel_limit, acc_limit;
  hday_robot_.getPositionLowerBounds(pos_lb);
  hday_robot_.getPositionUpperBounds(pos_ub);
  hday_robot_.getVelocityBounds(vel_limit);
  hday_robot_.getAccelerationBounds(acc_limit);
  vel_limit = 2.62 * Eigen::VectorXd::Ones(vel_limit.size());
  acc_limit = 8.0 * Eigen::VectorXd::Ones(acc_limit.size());
  trajectory_planner_ = std::make_shared<hday::trajectory_planner::TrajectoryOptimization>(
      pos_lb - 0.2 * Eigen::VectorXd::Ones(pos_lb.size()),
      pos_ub + 0.2 * Eigen::VectorXd::Ones(pos_lb.size()), vel_limit, acc_limit);

  otg_ = std::make_shared<ruckig::Ruckig<7>>(0.001);
  input_ = std::make_shared<ruckig::InputParameter<7>>();
  input_->current_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input_->current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input_->current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input_->target_position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input_->target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input_->target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double vel_scale = 0.5;
  input_->max_velocity = {2.62 * vel_scale, 2.62 * vel_scale, 2.62 * vel_scale, 2.62 * vel_scale,
                          2.62 * vel_scale, 2.62 * vel_scale, 2.62 * vel_scale};
  input_->min_velocity = {2.62 * -vel_scale, 2.62 * -vel_scale, 2.62 * -vel_scale,
                          2.62 * -vel_scale, 2.62 * -vel_scale, 2.62 * -vel_scale,
                          2.62 * -vel_scale};
  double acc_scale = 0.8;
  input_->max_acceleration = {10.0 * acc_scale, 10.0 * acc_scale, 10.0 * acc_scale,
                              10.0 * acc_scale, 10.0 * acc_scale, 10.0 * acc_scale,
                              10.0 * acc_scale};
  input_->min_acceleration = {10.0 * -acc_scale, 10.0 * -acc_scale, 10.0 * -acc_scale,
                              10.0 * -acc_scale, 10.0 * -acc_scale, 10.0 * -acc_scale,
                              10.0 * -acc_scale};
  double jerk_scale = 0.1;
  input_->max_jerk = {5000.0 * jerk_scale, 5000.0 * jerk_scale, 5000.0 * jerk_scale,
                      5000.0 * jerk_scale, 5000.0 * jerk_scale, 5000.0 * jerk_scale,
                      5000.0 * jerk_scale};
  output_ = std::make_shared<ruckig::OutputParameter<7>>();

  q_ref_ = Eigen::VectorXd::Zero(7);
  dq_ref_ = Eigen::VectorXd::Zero(7);
  d2q_ref_ = Eigen::VectorXd::Zero(7);
  Kp_ = 0.2 * Eigen::MatrixXd::Identity(7, 7);
  Kv_ = 0.1 * Eigen::MatrixXd::Identity(7, 7);
  q_otg_ = Eigen::VectorXd::Zero(7);
  dq_otg_ = Eigen::VectorXd::Zero(7);
  d2q_otg_ = Eigen::VectorXd::Zero(7);
}

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/initial_joint_position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    // config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    // config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

// compute 7th order polynomial coefficients
Matrix8Xd JointPositionController::compute7thOrderCoeffs(const Eigen::VectorXd& q0,
                                                         const Eigen::VectorXd& qf,
                                                         double T) {
  double T2 = T * T;
  double T3 = T2 * T;
  double T4 = T3 * T;
  double T5 = T4 * T;
  double T6 = T5 * T;
  double T7 = T6 * T;
  Matrix8d A;
  // clang-format off
  A << 1, 0, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0, 0,
       0, 0, 2, 0, 0, 0, 0, 0,
       0, 0, 0, 6, 0, 0, 0, 0,
       1, T, T2, T3, T4, T5, T6, T7,
       0, 1, 2 * T, 3 * T3, 4 * T3, 5 * T4, 6 * T5, 7 * T6,
       0, 0, 2, 6 * T, 12 * T2, 20 * T3, 30 * T4, 42 * T5,
       0, 0, 0, 6, 24 * T, 60 * T2, 120 * T3, 210 * T4;
  // clang-format on

  // double Tinv = 1.0 / T;
  // double T2inv = Tinv * Tinv;
  // double T3inv = T2inv * Tinv;
  // double T4inv = T3inv * Tinv;
  // double T5inv = T4inv * Tinv;
  // double T6inv = T5inv * Tinv;
  // double T7inv = T6inv * Tinv;

  // Matrix8d Ainv;
  // Ainv.setZero();
  // Ainv(0, 0) = 1.0;
  // Ainv(1, 1) = 1.0;
  // Ainv(2, 2) = 1.0 / 2.0;
  // Ainv(3, 3) = 1.0 / 6.0;

  // Ainv(4, 0) = -35.0 * T4inv;
  // Ainv(4, 1) = -20.0 * T3inv;
  // Ainv(4, 2) = -5.0 * T2inv;
  // Ainv(4, 3) = -2.0 / 3.0 * Tinv;
  // Ainv(4, 4) = 35.0 * T4inv;
  // Ainv(4, 5) = -15.0 * T3inv;
  // Ainv(4, 6) = 5.0 / 2.0 * T2inv;
  // Ainv(4, 7) = -1.0 / 6.0 * Tinv;

  // Ainv(5, 0) = 84.0 * T5inv;
  // Ainv(5, 1) = 45.0 * T4inv;
  // Ainv(5, 2) = 10.0 * T3inv;
  // Ainv(5, 3) = 1.0 * T2inv;
  // Ainv(5, 4) = -84.0 * T5inv;
  // Ainv(5, 5) = 39.0 * T4inv;
  // Ainv(5, 6) = -7.0 * T3inv;
  // Ainv(5, 7) = 1.0 / 2.0 * T2inv;

  // Ainv(6, 0) = -70.0 * T6inv;
  // Ainv(6, 1) = -36.0 * T5inv;
  // Ainv(6, 2) = -15.0 / 2.0 * T4inv;
  // Ainv(6, 3) = -2.0 / 3.0 * T3inv;
  // Ainv(6, 4) = 70.0 * T6inv;
  // Ainv(6, 5) = -34.0 * T5inv;
  // Ainv(6, 6) = 13.0 / 2.0 * T4inv;
  // Ainv(6, 7) = -1.0 / 2.0 * T3inv;

  // Ainv(7, 0) = 20.0 * T7inv;
  // Ainv(7, 1) = 10.0 * T6inv;
  // Ainv(7, 2) = 2.0 * T5inv;
  // Ainv(7, 3) = 1.0 / 6.0 * T4inv;
  // Ainv(7, 4) = -20.0 * T7inv;
  // Ainv(7, 5) = 10.0 * T6inv;
  // Ainv(7, 6) = -2.0 * T5inv;
  // Ainv(7, 7) = 1.0 / 6.0 * T4inv;

  int D = q0.size();
  Matrix8Xd b(8, D);
  b.row(0) = q0.transpose();
  b.row(1).setZero();
  b.row(2).setZero();
  b.row(3).setZero();
  b.row(4) = qf.transpose();
  b.row(5).setZero();
  b.row(6).setZero();
  b.row(7).setZero();

  Matrix8Xd coeffs = A.fullPivLu().solve(b);
  // Matrix8Xd coeffs = Ainv * b;
  return coeffs;
}

void JointPositionController::evaluateTrajectory(const Matrix8Xd& coeffs,
                                                 double t,
                                                 Eigen::VectorXd& pos,
                                                 Eigen::VectorXd& vel,
                                                 Eigen::VectorXd& acc,
                                                 Eigen::VectorXd& jerk) {
  int D = coeffs.cols();

  for (int i = 0; i < D; ++i) {
    Eigen::VectorXd c = coeffs.col(i);
    pos(i) = c(0) + c(1) * t + c(2) * std::pow(t, 2) + c(3) * std::pow(t, 3) +
             c(4) * std::pow(t, 4) + c(5) * std::pow(t, 5) + c(6) * std::pow(t, 6) +
             c(7) * std::pow(t, 7);
    vel(i) = c(1) + 2 * c(2) * t + 3 * c(3) * std::pow(t, 2) + 4 * c(4) * std::pow(t, 3) +
             5 * c(5) * std::pow(t, 4) + 6 * c(6) * std::pow(t, 5) + 7 * c(7) * std::pow(t, 6);
    acc(i) = 2 * c(2) + 6 * c(3) * t + 12 * c(4) * std::pow(t, 2) + 20 * c(5) * std::pow(t, 3) +
             30 * c(6) * std::pow(t, 4) + 42 * c(7) * std::pow(t, 5);
    jerk(i) = 6 * c(3) + 24 * c(4) * t + 60 * c(5) * std::pow(t, 2) + 120 * c(6) * std::pow(t, 3) +
              210 * c(7) * std::pow(t, 4);
  }
}

void JointPositionController::evaluateTrajectory_pos(const Matrix8Xd& coeffs,
                                                     double t,
                                                     Eigen::VectorXd& pos) {
  int D = coeffs.cols();

  for (int i = 0; i < D; ++i) {
    Eigen::VectorXd c = coeffs.col(i);
    pos(i) = c(0) + c(1) * t + c(2) * std::pow(t, 2) + c(3) * std::pow(t, 3) +
             c(4) * std::pow(t, 4) + c(5) * std::pow(t, 5) + c(6) * std::pow(t, 6) +
             c(7) * std::pow(t, 7);
  }
}
double JointPositionController::computeViolation(const Matrix8Xd& coeffs,
                                                 double T,
                                                 double v_max,
                                                 double a_max,
                                                 double j_max,
                                                 double dt) {
  int steps = static_cast<int>(T / dt) + 1;
  double max_violation = 0.0;

  for (int i = 0; i < steps; ++i) {
    // if (i == 0) {
    //   v_max = 2.62;
    // } else if (i == 1) {
    //   v_max = 2.62;
    // } else if (i == 2) {
    //   v_max = 2.62;
    // } else if (i == 3) {
    //   v_max = 2.62;
    // } else if (i == 4) {
    //   v_max = 5.26;
    // } else if (i == 5) {
    //   v_max = 4.18;
    // } else if (i == 6) {
    //   v_max = 5.26;
    // }
    double t = i * dt;
    Eigen::VectorXd pos_tmp, vel_tmp, acc_tmp, jerk_tmp;
    pos_tmp.setZero(num_joints);
    vel_tmp.setZero(num_joints);
    acc_tmp.setZero(num_joints);
    jerk_tmp.setZero(num_joints);
    evaluateTrajectory(coeffs, t, pos_tmp, vel_tmp, acc_tmp, jerk_tmp);
    max_violation = std::max(max_violation, vel_tmp.cwiseAbs().maxCoeff() / v_max);
    max_violation = std::max(max_violation, acc_tmp.cwiseAbs().maxCoeff() / a_max);
    max_violation = std::max(max_violation, jerk_tmp.cwiseAbs().maxCoeff() / j_max);
  }

  return max_violation;
}

double JointPositionController::findMinimumT(const Eigen::VectorXd& q0,
                                             const Eigen::VectorXd& qf,
                                             double v_max,
                                             double a_max,
                                             double j_max,
                                             double dt,
                                             double margin) {
  double T_low = 0.01, T_high = 10.0, T_mid;
  for (int iter = 0; iter < 100; ++iter) {
    T_mid = 0.5 * (T_low + T_high);
    auto coeffs = compute7thOrderCoeffs(q0, qf, T_mid);
    double violation = computeViolation(coeffs, T_mid, v_max, a_max, j_max, dt);
    if (violation < margin)
      T_high = T_mid;
    else
      T_low = T_mid;
    if (T_high - T_low < 1e-4)
      break;
  }
  return T_high;
}

controller_interface::return_type JointPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[2 * i].get_value();
      command_interfaces_[i].set_value(initial_q_.at(i));
    }
    initialization_flag_ = false;
  }

  // auto joint_command_msg_ptr = joint_command_msg_external_point_ptr_.readFromRT();
  // if (!joint_command_msg_ptr || !(*joint_command_msg_ptr)) {
  //   // Buffer is empty or contains nullptr
  // } else {
  //   // Buffer contains a valid message
  //   // auto joint_command_msg = *joint_command_msg_ptr;

  //   if (command_updated || send_mode) {
  //     command_updated = false;
  //     send_mode = true;

  //     if (elapsed_time <= T_opt) {
  //       evaluateTrajectory_pos(coeffs, elapsed_time, pos);
  //       for (int i = 0; i < num_joints; ++i) {
  //         command_interfaces_[i].set_value(pos(i));
  //       }
  //       elapsed_time += 0.001;
  //       std::cout << elapsed_time << std::endl;
  //       // std::cout << pos.transpose() << std::endl;
  //     } else {
  //       send_mode = false;
  //       elapsed_time = 0.0;
  //     }
  //   }
  // }

  if (command_updated || send_mode) {
    command_updated = false;
    send_mode = true;

    if (elapsed_time <= T_opt) {
      // std::cout << elapsed_time << std::endl;
      // std::cout << pos.transpose() << std::endl;

      // 7-th order polynomial
      evaluateTrajectory_pos(coeffs, elapsed_time, pos);

      // // trajectory optimization
      // auto point = trajectory_planner_->getTrajectory().getTrajectoryPointAt(elapsed_time);
      // pos = point.pos;

      // // ruckig
      // otg_->update(*input_, *output_);
      // // Eigen::VectorXd q_otg(7), dq_otg(7), d2q_otg(7);
      // for (unsigned i = 0; i < num_joints; i++) {
      //   pos(i) = output_->new_position[i];
      //   q_otg_(i) = output_->new_position[i];
      //   dq_otg_(i) = output_->new_velocity[i];
      //   d2q_otg_(i) = output_->new_acceleration[i];
      // }
      // // d2q_ref_ = -Kp_ * (q_ref_ - q_otg) - Kv_ * (dq_ref_ - dq_otg) + d2q_otg;
      // output_->pass_to_input(*input_);

      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(pos(i));
      }
      elapsed_time += 0.001;
    } else {
      d2q_otg_.setZero();
      dq_otg_.setZero();
      send_mode = false;
      elapsed_time = 0.0;
    }
  }

  // if (!ini) {
  //   d2q_ref_ = -Kp_ * (q_ref_ - q_otg_) - Kv_ * (dq_ref_ - dq_otg_) + d2q_otg_;
  //   for (unsigned i = 0; i < num_joints; i++) {
  //     if (d2q_ref_(i) < input_->min_acceleration.value()[i])
  //       d2q_ref_(i) = input_->min_acceleration.value()[i];
  //     else if (d2q_ref_(i) > input_->max_acceleration[i])
  //       d2q_ref_(i) = input_->max_acceleration[i];
  //   }
  //   dq_ref_ += 0.001 * d2q_ref_;
  //   for (unsigned i = 0; i < num_joints; i++) {
  //     if (dq_ref_(i) < input_->min_velocity.value()[i])
  //       dq_ref_(i) = input_->min_velocity.value()[i];
  //     else if (dq_ref_(i) > input_->max_velocity[i])
  //       dq_ref_(i) = input_->max_velocity[i];
  //   }
  //   q_ref_ += 0.001 * dq_ref_;
  //   for (int i = 0; i < num_joints; ++i) {
  //     command_interfaces_[i].set_value(q_ref_(i));
  //   }

  //   std::cout << "---------\n";
  //   std::cout << "pos: ";
  //   for (unsigned i = 0; i < num_joints; i++)
  //     std::cout << q_ref_[i] << "\t";
  //   std::cout << "\n";
  //   std::cout << "vel: ";
  //   for (unsigned i = 0; i < num_joints; i++)
  //     std::cout << dq_ref_[i] << "\t";
  //   std::cout << "\n";
  //   std::cout << "acc: ";
  //   for (unsigned i = 0; i < num_joints; i++)
  //     std::cout << d2q_ref_[i] << "\t";
  //   std::cout << "\n";
  // }

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

CallbackReturn JointPositionController::on_init() {
  try {
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_configure(
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

  // subscribe
  joint_command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "/hday/rt_franka/joint_position_command", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointPositionController::joint_command_callback, this, std::placeholders::_1));

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

  q0.setZero(num_joints);
  qf.setZero(num_joints);
  pos.setZero(num_joints);
  vel.setZero(num_joints);
  acc.setZero(num_joints);
  jerk.setZero(num_joints);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // joint_command_msg_external_point_ptr_.writeFromNonRT(
  //     std::shared_ptr<sensor_msgs::msg::JointState>());
  updateJointStates();
  initialization_flag_ = true;
  return CallbackReturn::SUCCESS;
}

void JointPositionController::joint_command_callback(
    const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
  std::cout << "A: " << send_mode << std::endl;
  if (send_mode) {
    std::cout << "============" << std::endl;
    std::cout << "Ignored" << std::endl;
    std::cout << "Ignored" << std::endl;
    std::cout << "Ignored" << std::endl;
    std::cout << "Ignored" << std::endl;
    std::cout << "Ignored" << std::endl;
    std::cout << "Ignored" << std::endl;
    std::cout << "Ignored" << std::endl;
  }

  if (!send_mode) {
    if (!validate_position_msg(*msg)) {
      return;
    }
    joint_command_msg_external_point_ptr_.writeFromNonRT(msg);
    for (int i = 0; i < num_joints; ++i) {
      if (ini) {
        q0(i) = initial_q_.at(i);
        input_->current_position[i] = q0(i);  // update only at the initial
        q_ref_ = q0;
      } else {
        q0(i) = pos(i);
      }
    }
    for (int i = 0; i < num_joints; ++i) {
      qf(i) = msg->position[i];
    }

    auto start = high_resolution_clock::now();
    // double frac = 0.9;
    // double v_max = 2.62, a_max = 10.0 * frac, j_max = 5000.0 * frac;

    // 7-th order polynomial
    double v_max = 2.62, a_max = 8.0, j_max = 120.0;
    double dt = 0.001, margin = 0.98;
    T_opt = findMinimumT(q0, qf, v_max, a_max, j_max, dt, margin);
    T_opt = std::ceil(T_opt * 1000.0) / 1000.0;
    coeffs = compute7thOrderCoeffs(q0, qf, T_opt);
    command_updated = true;
    send_mode = true;
    ini = false;

    // // trajectory optimization
    // hday::path_planner::discrete::WaypointGraph::Vertex source, target;
    // source.point = q0;
    // source.derivatives.resize(q0.size(), 3);
    // source.setDerivatives(0, Eigen::VectorXd::Zero(q0.size()));  // v0 = 0
    // source.setDerivatives(1, Eigen::VectorXd::Zero(q0.size()));  // a0 = 0
    // target.point = qf;
    // target.derivatives.resize(qf.size(), 3);
    // target.setDerivatives(0, Eigen::VectorXd::Zero(qf.size()));  // vf = 0
    // target.setDerivatives(1, Eigen::VectorXd::Zero(qf.size()));  // af = 0

    // auto path = std::make_shared<hday::path_planner::discrete::WaypointGraph::Edge>();
    // path->from = std::make_shared<hday::path_planner::discrete::WaypointGraph::Vertex>(source);
    // path->to = std::make_shared<hday::path_planner::discrete::WaypointGraph::Vertex>(target);
    // path->velocity_scale = 1.0;
    // path->acceleration_scale = 0.8;
    // if (trajectory_planner_->plan({path})) {
    // T_opt = trajectory_planner_->getTrajectory().duration();
    // }

    // // ruckig
    // for (unsigned i = 0; i < num_joints; i++)
    //   input_->target_position[i] = qf(i);  // update target position at requested

    // ruckig::Trajectory<7> trajectory;
    // auto result = otg_->calculate(*input_, trajectory);
    // if (result != ruckig::Result::ErrorInvalidInput) {
    //   command_updated = true;
    //   send_mode = true;
    //   ini = false;
    //   T_opt = trajectory.get_duration();
    // }

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end - start).count();
    std::cout << T_opt << " " << duration * 0.001 << std::endl;
    std::cout << q0.transpose() << std::endl;
    std::cout << qf.transpose() << std::endl;
  }
};

bool JointPositionController::validate_position_msg(
    const sensor_msgs::msg::JointState& command) const {
  if (command.name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming command.");
    return false;
  }

  if (command.name.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint name size mismatch");
    return false;
  }

  if (command.position.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Position command size mismatch");
    return false;
  }

  return true;
}

void JointPositionController::updateJointStates() {
  joint_state.header.stamp = get_node()->now();
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i + 1);
    // const auto& velocity_interface = state_interfaces_.at(4 * i + 2);
    // const auto& effort_interface = state_interfaces_.at(4 * i + 3);

    assert(position_interface.get_interface_name() == "position");
    // assert(velocity_interface.get_interface_name() == "velocity");
    // assert(effort_interface.get_interface_name() == "effort");

    joint_state.position[i] = position_interface.get_value();
    // joint_state.velocity[i] = velocity_interface.get_value();
    // joint_state.effort[i] = effort_interface.get_value();
  }
}

void JointPositionController::publish_joint_state(const sensor_msgs::msg::JointState& joint_state) {
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

void JointPositionController::publish_ef_pose(const Eigen::Vector3d ef_position,
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
PLUGINLIB_EXPORT_CLASS(franka_controllers::JointPositionController,
                       controller_interface::ControllerInterface)
