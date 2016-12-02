/**
 * Based on: https://github.com/davetcoleman/ros_control_boilerplate
 */

#ifndef KULBABU_ROS_CONTROL__KULBABU_HARDWARE_INTERFACE_H
#define KULBABU_ROS_CONTROL__KULBABU_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace kulbabu_hardware {

/// \brief Hardware interface for a robot
class KulbabuHardwareInterface: public hardware_interface::RobotHW {
 public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   * \param joint_mode - method to control joints by: 0 - position, 1 - velocity, or 2 - effort
   */
  KulbabuHardwareInterface(ros::NodeHandle& nh, int joint_mode);

  /// \brief Initialize the hardware interface
  virtual void init();

  /// \brief Read the state from the robot hardware.
  virtual void read(ros::Duration elapsed_time);

  /// \brief write the command to the robot hardware.
  virtual void write(ros::Duration elapsed_time);

 protected:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle                              nh_;

  // Interfaces
  hardware_interface::JointStateInterface      joint_state_interface_;
  //hardware_interface::PositionJointInterface   position_joint_interface_;
  hardware_interface::VelocityJointInterface   velocity_joint_interface_;
  //hardware_interface::EffortJointInterface     effort_joint_interface_;

  // Shared memory
  int                                          steps_per_m_;

  std::vector<int>                             pin_dirs_;
  std::vector<int>                             pin_steps_;

  std::vector<std::string>                     joint_names_;
  //std::vector<double>                          joint_position_;
  std::vector<double>                          joint_velocity_;
  //std::vector<double>                          joint_effort_;
  //std::vector<double>                          joint_position_command_;
  std::vector<double>                          joint_velocity_command_;
  //std::vector<double>                          joint_effort_command_;
  std::size_t                                  num_joints_;
  // `joint_mode_`: 0 - position, 1 - velocity, or 2 - effort
  //std::size_t                                  joint_mode_;
};  // class

}  // namespace kulbabu_hardware

#endif  // KULBABU_ROS_CONTROL__KULBABU_HARDWARE_INTERFACE_H
