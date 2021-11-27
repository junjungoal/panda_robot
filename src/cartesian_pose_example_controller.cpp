// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// #include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <panda_robot/cartesian_pose_example_controller.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace panda_robot_controllers {

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  pose_cartesian_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (pose_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  try {
    pose_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        pose_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  // try {
  //   auto state_handle = state_interface->getHandle(arm_id + "_robot");
  //
  //   std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  //   for (size_t i = 0; i < q_start.size(); i++) {
  //     if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
  //       ROS_ERROR_STREAM(
  //           "CartesianPoseExampleController: Robot is not in the expected starting position for "
  //           "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //           "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //       return false;
  //     }
  //   }
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "CartesianPoseExampleController: Exception getting state handle: " << e.what());
  //   return false;
  // }


  this->was_in_contact = false;
  this->time_since_last_target_pose = ros::Time::now();
  this->target_pose_subscriber =
      node_handle.subscribe("/franka_control/target_pose", 1, &CartesianPoseExampleController::target_pose_callback, this);
  // this->current_pose_publisher = node_handle.advertise<geometry_msgs::Twist>("/franka_control/current_target_pose", 1);
  return true;
}

void CartesianPoseExampleController::target_pose_callback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->target_pose = *msg;

    if (this->target_pose.position.x != this->target_pose.position.x ||
        this->target_pose.position.y != this->target_pose.position.y ||
        this->target_pose.position.z != this->target_pose.position.z ||
        this->target_pose.orientation.x != this->target_pose.orientation.x ||
        this->target_pose.orientation.y != this->target_pose.orientation.y ||
        this->target_pose.orientation.z != this->target_pose.orientation.z ||
        this->target_pose.orientation.w != this->target_pose.orientation.w) {

        ROS_ERROR("Can't have NaN in the target pose");
        exit(-1);
    }
    this->time_since_last_target_pose = ros::Time::now();
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  this->current_pose = pose_cartesian_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianPoseExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  auto state_handle = this->state_interface->getHandle(arm_id + "_robot");
  
  bool is_in_contact_x = state_handle.getRobotState().cartesian_contact[0] > 0;
  bool is_in_contact_y = state_handle.getRobotState().cartesian_contact[1] > 0;
  bool is_in_contact_z = state_handle.getRobotState().cartesian_contact[2] > 0;

  bool is_in_contact = is_in_contact_x || is_in_contact_y || is_in_contact_z;

  std::array<double, 16> new_pose = this->current_pose;
  new_pose[12] = this->target_pose.position.x;
  new_pose[13] = this->target_pose.position.y;
  new_pose[14] = this->target_pose.position.z;
  // std::array<double, 16> new_pose = this->current_pose;

  pose_cartesian_handle_->setCommand(new_pose);

  was_in_contact = is_in_contact;
}

void CartesianPoseExampleController::stopping(const ros::Time& /*time*/) {
  this->current_pose = pose_cartesian_handle_->getRobotState().O_T_EE_d;
  pose_cartesian_handle_->setCommand(this->current_pose);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(panda_robot_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)
