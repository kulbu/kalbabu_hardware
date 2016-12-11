#ifndef KULBABU_ROS_CONTROL__KULBABU_HARDWARE_MOTOR_H
#define KULBABU_ROS_CONTROL__KULBABU_HARDWARE_MOTOR_H

// C++
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>

namespace kulbabu_hardware
{

/// \brief Hardware interface for a robot
class KulbabuHardwareMotors
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  KulbabuHardwareMotors(ros::NodeHandle &nh);

  /** \brief Destructor */
  virtual ~KulbabuHardwareMotors() {}

  /** \brief Set the buffer for sending to the robot hardware. */
  virtual void setCommand(uint8_t index, double cmd_perc);

  /** \brief Send buffer to the robot hardware. */
  virtual void doCommand();

  /** \brief Read the encoder count from the robot hardware. */
  virtual double getEncoderVelocity(uint8_t index);

protected:

  // Short name of this class
  std::string name_;

  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  int encoder_max_;

};  // class

}  // namespace

#endif // KULBABU_ROS_CONTROL__KULBABU_HARDWARE_MOTOR_H
