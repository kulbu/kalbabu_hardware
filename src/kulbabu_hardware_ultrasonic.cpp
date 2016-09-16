#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kulbabu_hardware_ultrasonic");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // TODO: Do something.

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
