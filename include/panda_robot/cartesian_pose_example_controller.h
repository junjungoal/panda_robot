// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace panda_robot_controllers {

class CartesianPoseExampleController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void target_pose_callback(const std_msgs::Float32MultiArray::ConstPtr& msgs);

 private:
  std::string arm_id;
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  franka_hw::FrankaStateInterface* state_interface;
  ros::Duration elapsed_time_;
  geometry_msgs::Pose target_pose;
  std::array<double, 16> current_pose {};

  std::array<double, 16> raw_pose_cmd = {{0.0, 0.0, 0.0}};
  std::array<double, 16> filtered_raw_pose_cmd = {{0.0, 0.0, 0.0}};
  std::array<double, 16> filtered_target_cmd = {{0.0, 0.0, 0.0}};
  bool read_message;
  float alpha1;
  float alpha2;
  float decay_rate;



  ros::Subscriber target_pose_subscriber;
};

}  // namespace franka_example_controllers
