// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>

namespace panda_robot_controllers {

class CartesianPoseExampleController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;
  void target_pose_callback(const geometry_msgs::Pose::ConstPtr& msg);

 private:
  franka_hw::FrankaPoseCartesianInterface* pose_cartesian_interface_;
  franka_hw::FrankaStateInterface* state_interface;
  std::string arm_id;

  bool was_in_contact;
  // std::array<double, 6> ee_twist_cause_of_contact;
  ros::Time time_since_last_target_pose;
  
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> pose_cartesian_handle_;
  ros::Duration elapsed_time_;
  
  geometry_msgs::Pose target_pose;
  // geometry_msgs::Pose current_pose;
  std::array<double, 16> current_pose;
  
  ros::Subscriber target_pose_subscriber;

  // ros::Publisher current_pose_publisher;
  
};

}  
