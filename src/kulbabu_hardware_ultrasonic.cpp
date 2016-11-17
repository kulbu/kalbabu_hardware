#include <ros/ros.h>
#include <sensor_msgs/Range>
#include <linux/i2c-dev.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kulbabu_hardware_ultrasonic");
  ros::NodeHandle nh;

  ros::Publisher range_pub = n.advertise<sensor_msgs::Range>("range", 50);

  // Setup i2c
  int file;
  int adapter_nr = 1; /* probably dynamically determined */
  char filename[20];

  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  file = open(filename, O_RDWR);
  if (file < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    ROS_ERROR_STREAM( "I2C failed opening connection");
    exit(1);
  }

  // TODO: Configurable.
  int addr = 0x10; /* The I2C address */

  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    ROS_ERROR_STREAM( "I2C failed addressing connection");
    exit(1);
  }

  //ros::spin();
  ros::Time current_time

  // TODO: Configurable rate.
  ros::Rate r(10.0);
  while (n.ok()) {
    current_time = ros::Time::now();

    uint8_t res;

    // Read data from i2c
    res = i2c_smbus_read_word_data(file, reg);
    if (res < 0) {
      /* ERROR HANDLING: i2c transaction failed */
      ROS_WARN_STREAM( "I2C failed " << res );
    } else {
      /* res contains the read word */
      ROS_INFO_STREAM( "I2C read " << res );

      // https://github.com/ros/common_msgs/blob/indigo-devel/sensor_msgs/msg/Range.msg
      sensor_msgs::Range range_msg;
      range_msg.header.stamp = current_time;

      range_msg.radiation_type = 0; // Ultrasonic
      range_msg.field_of_view = 0.261799388; // 15deg in rad (HCSR04)
      range_msg.max_range = 4; // 4m
      range_msg.min_range = 2/100; // 2cm

      for (uint8_t x=0;x<8;x++) {
        sprintf(range_msg.header.frame_id, "range%d", x)
        range_msg.range = res[x];
        range_pub.publish(range_msg);
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  // Wait until shutdown signal recieved
  //ros::waitForShutdown();

  return 0;
}
