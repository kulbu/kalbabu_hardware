/**
 * Based on: https://github.com/davetcoleman/kulbabu_hardware
 */
#include <kulbabu_hardware/kulbabu_hardware_control_loop.h>
#include <kulbabu_hardware/kulbabu_hardware_interface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kulbabu_hardware_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<kulbabu_hardware::KulbabuHardwareInterface> kulbabu_hardware_interface
    (new kulbabu_hardware::KulbabuHardwareInterface(nh));
  kulbabu_hardware_interface->init();

  // Start the control loop
  kulbabu_hardware::KulbabuHardwareControlLoop control_loop(nh, kulbabu_hardware_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
