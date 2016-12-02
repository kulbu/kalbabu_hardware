/**
 * Based on: https://github.com/davetcoleman/ros_control_boilerplate
 */

#include <kulbabu_hardware/kulbabu_hardware_interface.h>
#include <fcntl.h>

#define SYSFS_GPIO_DIR  "/sys/class/gpio"
#define SYSFS_PWM_DIR   "/sys/devices/platform/pwm-ctrl"
#define MAX_BUF 256

// FIXME: Encapsulate this IO code.
int pwm_enable(unsigned int pwm, bool enable) {
  int fd;
  char buf[MAX_BUF];

  // First check current value, only update if changed.
  char curr[11];
  int status;
  snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/enable%d", pwm);
  fd = open(buf, sizeof(buf), O_RDONLY);
  if (fd < 0) {
    ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
      "pwm/enable/read");
    return fd;
  }
  read(fd, curr, sizeof(curr));
  close(fd);

  // "On" always ends with an "n". Quick and dirty.
  status = (curr[strlen(curr)-2] == 'n');

  if (status != enable) {
    ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
       "Chan: " << pwm << " Enable: " << enable);

    snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/enable%d", pwm);
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
        "pwm/enable/write");
      return fd;
    }

    if (enable)
      write(fd, "1", 2);
    else
      write(fd, "0", 2);

    close(fd);
  }
  return 0;
}

int pwm_freq(unsigned int pwm, unsigned int freq) {
  int fd;
  char buf[MAX_BUF];

  // First check current value, only update if changed.
  char curr[11];
  int status;
  snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/freq%d", pwm);
  fd = open(buf, sizeof(buf), O_RDONLY);
  if (fd < 0) {
    ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
      "pwm/freq/read");
    return fd;
  }
  read(fd, curr, sizeof(curr));
  close(fd);

  if (atoi(curr) != freq && freq >= 1) {
    ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
       "Chan: " << pwm << " Freq: " << freq);

    snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/freq%d", pwm);
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
        "pwm/freq/write");
      return fd;
    }

    int len = snprintf(buf, sizeof(buf), "%d", freq);
    write(fd, buf, len);
    close(fd);
  }
  return 0;
}

int pwm_duty(unsigned int pwm, unsigned int duty) {
  int fd;
  char buf[MAX_BUF];

  ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
     "Chan: " << pwm << " Duty: " << duty);

  snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/duty%d", pwm);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
      "pwm/duty/write");
    return fd;
  }

  int len = snprintf(buf, sizeof(buf), "%d", duty);
  write(fd, buf, len);
  close(fd);

  return 0;
}

/*
int gpio_own(char *file) {
  uid_t uid = getuid();
  uid_t gid = getgid();

  if (chown (file, uid, gid) != 0) {
    if (errno == ENOENT) {
      ROS_WARN_STREAM_NAMED("kulbabu_hardware_interface",
        "gpio/own File does not exist");
    } else {
      ROS_FATAL_STREAM_NAMED("kulbabu_hardware_interface",
        "gpio/own Unable to change ownership "
        << file << " Error: " << strerror (errno));
      exit (1) ;
    }
  }

}

int gpio_export(unsigned int gpio) {
  int fd;
  char buf[MAX_BUF];

  ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
     "Export: " << gpio);

  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd < 0) {
    ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
      "gpio/export");
    return fd;
  }

  int len = snprintf(buf, sizeof(buf), "%d", gpio);
  write(fd, buf, len);
  close(fd);

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  gpio_own(buf);

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
  gpio_own(buf);

 return 0;
}

int gpio_dir(unsigned int gpio, bool output) {
  int fd;
  char buf[MAX_BUF];

  ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
     "Dir: " << gpio << " Output: " << output);

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
      "gpio/direction");
    return fd;
  }

  if (output)
    write(fd, "out", 2);
  else
    write(fd, "in", 2);

  close(fd);

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
  gpio_own(buf);

  return 0;
}
*/

int gpio_set(unsigned int gpio, unsigned int value) {
  int fd;
  char buf[MAX_BUF];

  ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
     "GPIO: " << gpio << " Value: " << value);

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
      "gpio/set-value");
    return fd;
  }

  if (value)
    write(fd, "1", 2);
  else
    write(fd, "0", 2);

  close(fd);
  return 0;
}

namespace kulbabu_hardware {

KulbabuHardwareInterface::KulbabuHardwareInterface(
  ros::NodeHandle& nh,
  int joint_mode)
  : nh_(nh),
  joint_mode_(joint_mode) {
  // Initialize shared memory and interfaces here
  init();  // this implementation loads from rosparam

  ROS_INFO_NAMED("kulbabu_hardware_interface",
    "Loaded kulbabu_hardware_interface.");
}

void KulbabuHardwareInterface::init() {
  ROS_INFO_STREAM_NAMED("kulbabu_hardware_interface",
    "Reading rosparams from namespace: " << nh_.getNamespace());

  nh_.getParam("hardware_interface/steps_per_m", steps_per_m_);
  if (!steps_per_m_ > 0) {
    ROS_FATAL_STREAM_NAMED("kulbabu_hardware_interface",
      "No step per metre found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  // Get joint names
  nh_.getParam("hardware_interface/joints", joint_names_);
  if (joint_names_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbabu_hardware_interface",
      "No joints found on parameter server"
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }
  num_joints_ = joint_names_.size();

  // Get direction GPIO pins; 0 = forward, 1 = reverse.
  nh_.getParam("hardware_interface/dirs", pin_dirs_);
  if (pin_dirs_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbabu_hardware_interface",
      "No GPIO direction pins found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  // Get PWM sysfs indexes.
  nh_.getParam("hardware_interface/steps", pin_steps_);
  if (pin_steps_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbabu_hardware_interface",
      "No GPIO step pins found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialize controller
  for (std::size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
      "Loading joint name: " << joint_names_[i]);

    // Export GPIO pin for output.
    // FIXME: Permission denied? Manual export `gpio export 88 out`
    // gpio_export(pin_dirs_[i]);
    // gpio_dir(pin_dirs_[i], 1);

    // Start with PWM turned off and 50/50 duty cycle.
    pwm_enable(pin_steps_[i], 0);
    pwm_duty(pin_steps_[i], 512);

    // Set direction pins to forward by default.
    gpio_set(pin_dirs_[i], 0);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
      joint_names_[i],
      &joint_position_[i],
      &joint_velocity_[i],
      &joint_effort_[i]));

    switch (joint_mode_) {
      case 0:
        // Create position joint interface
        position_joint_interface_
          .registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(
              joint_names_[i]),
              &joint_position_command_[i]));
        break;
      case 1:
        // Create velocity joint interface
        velocity_joint_interface_
          .registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(
              joint_names_[i]),
              &joint_velocity_command_[i]));
        break;
      case 2:
        // Create effort joint interface
        effort_joint_interface_
          .registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(
              joint_names_[i]),
              &joint_effort_command_[i]));
        break;
    }
  }

  registerInterface(&joint_state_interface_);  // From RobotHW base class.
  registerInterface(&position_joint_interface_);  // From RobotHW base class.
  registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
  registerInterface(&effort_joint_interface_);  // From RobotHW base class.
}

void KulbabuHardwareInterface::read(ros::Duration elapsed_time) {
  // TODO: Optical encoders.

  // Read the joint states from your hardware here
  // e.g.
  // for (std::size_t i = 0; i < num_joints_; ++i)
  // {
  //   joint_position_[i] = robot_api_.getJointPosition(i);
  //   joint_velocity_[i] = robot_api_.getJointVelocity(i);
  //   joint_effort_[i] = robot_api_.getJointEffort(i);
  // }
}

void KulbabuHardwareInterface::write(ros::Duration elapsed_time) {
  int freq = 0;
  int dir = 0;

  // Send commands in different modes
  for (std::size_t i = 0; i < num_joints_; ++i) {
    // Skip joints with no pins defined
    if (pin_dirs_[i] == -1 || pin_steps_[i] == -1) continue;

    switch (joint_mode_) {
      case 1:  // hardware_interface::MODE_VELOCITY:
        // Calc PWM frequency.
        freq = joint_velocity_command_[i] * steps_per_m_;

       // DEBUG: instantly adding velocity to state
        joint_velocity_[i] = joint_velocity_command_[i];

        // Reverse flips direction.
        if (freq < 0) {
          freq = abs(freq);
          dir = 1;
        } else {
          dir = 0;
        }

        // Disable PWM when stopped.
        if (freq < 1) {
          freq = 0;
          pwm_enable(pin_steps_[i], 0);
        } else {
          pwm_enable(pin_steps_[i], 1);
        }

        // Set direction pin.
        gpio_set(pin_dirs_[i], dir);

        // Set PWM frequency.
        pwm_freq(pin_steps_[i], freq);

        ROS_DEBUG_STREAM_NAMED("kulbabu_hardware_interface",
          "\ni: "     << i <<
          "\ncmd: "   << joint_velocity_command_[i] <<
          "\ndist: "  << steps_per_m_ <<
          "\nfreq: "  << freq <<
          "\ndir: "   << dir <<
          "\njoint: " << joint_names_[i] <<
          "\ndir: "   << pin_dirs_[i] <<
          "\nstep: "  << pin_steps_[i]);
        break;

      default:
        ROS_ERROR_STREAM_NAMED("kulbabu_hardware_interface",
          "Joint mode not implemented");
        break;
    }
  }
}


}  // namespace kulbabu_hardware
