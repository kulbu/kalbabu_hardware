/**
 * Based on: https://github.com/davetcoleman/ros_control_boilerplate
 */

#include <kulbabu_hardware/kulbabu_hardware_control_loop.h>

namespace kulbabu_hardware {

KulbabuHardwareControlLoop::KulbabuHardwareControlLoop(
  ros::NodeHandle& nh,
  boost::shared_ptr<kulbabu_hardware::KulbabuHardwareInterface> hardware_interface)
  : nh_(nh),
  hardware_interface_(hardware_interface) {
  // Create the controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(
    hardware_interface_.get(),
    nh_));

  // Get period - default to 100 hz
  nh_.param("hardware_control_loop/loop_hz", loop_hz_, 100.0);
  ROS_DEBUG_STREAM_NAMED("constructor",
    "Using loop freqency of " << loop_hz_ << " hz");

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &last_time_);

  // Start timer
  ros::Duration update_freq = ros::Duration(1/loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq,
    &KulbabuHardwareControlLoop::update, this);
}

void KulbabuHardwareControlLoop::update(const ros::TimerEvent& e) {
  // Get change in time
  clock_gettime(CLOCK_MONOTONIC, &current_time_);
  elapsed_time_ = ros::Duration(
    current_time_.tv_sec - last_time_.tv_sec +
    (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
  last_time_ = current_time_;
  // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "kulbabu_hardware_main",
  //  "Sampled update loop with elapsed time " << elapsed_time_.toSec());

  // Input
  hardware_interface_->read(elapsed_time_);

  // Control
  controller_manager_->update(ros::Time::now(), elapsed_time_);

  // Output
  hardware_interface_->write(elapsed_time_);
}

}  // namespace kulbabu_hardware
