#include <kulbabu_hardware/kulbabu_hardware_motors.h>

namespace kulbabu_hardware
{
KulbabuHardwareMotors::KulbabuHardwareMotors(ros::NodeHandle &nh)
  : name_("kulbabu_hardware_motors")
  , nh_(nh)
{
  // Load rosparams
  ROS_INFO_STREAM_NAMED(name_, "Loading encoder calibration");
  nh.getParam("hardware_interface/encoder_max", encoder_max_);
  if (!encoder_max_) {
    ROS_FATAL_STREAM_NAMED(name_,
      "No encoder_max found on parameter server"
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }
}

void KulbabuHardwareMotors::setCommand(uint8_t index, double cmd_perc)
{
  /*
  uint8_t cmd_dir  = 0;
  if (cmd_perc > 0) {
    cmd_dir = 255;
  } else if (cmd_perc < 0) {
    cmd_dir = 126;
  }
  */

  //buf[(index*2)] = cmd_dir; // Direction
  //buf[(index*2)+1] = abs(cmd_perc) * 255; // Velocity
}

void KulbabuHardwareMotors::doCommand()
{
  // TODO: Write then read to/from I2C.
}

double KulbabuHardwareMotors::getEncoderVelocity(uint8_t index)
{

  return 126/255;
}

}  // namespace
