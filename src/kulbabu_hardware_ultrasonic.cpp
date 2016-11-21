//#include <glib.h>
//#include <glib/gprintf.h>
//#include <errno.h>
//#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <linux/i2c-dev.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kulbabu_hardware_ultrasonic");
  ros::NodeHandle nh;

  ros::Publisher range_pubs[8]; 

  for (uint8_t x=0;x<8;x++) {	
    char name[10];
    sprintf(name, "range%d", x);
    range_pubs[x] = nh.advertise<sensor_msgs::Range>(name, 50);
  }

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
  uint8_t addr = 0x10; /* The I2C address */

  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    ROS_ERROR_STREAM( "I2C failed addressing connection");
    exit(1);
  }

  //ros::spin();

  // TODO: Configurable rate.
  ros::Rate r(10.0);
  while (nh.ok()) {
    ros::Time current_time = ros::Time::now();

    //uint8_t res;
    // Read data from i2c
    //res = i2c_smbus_read_word_data(file, addr);
    //if (res < 0) {


    uint8_t buf[8];
    buf[0] = addr;

    if (read(file, buf, sizeof(buf)) != sizeof(buf)) {
      /* ERROR HANDLING: i2c transaction failed */
      ROS_WARN_STREAM( "I2C failed " << buf );
    } else {
      // https://github.com/ros/common_msgs/blob/indigo-devel/sensor_msgs/msg/Range.msg
      sensor_msgs::Range range_msg;
      range_msg.header.stamp = current_time;

      range_msg.radiation_type = 0; // Ultrasonic // FIXME: sensor_msgs::Range::ULTRASOUND
      range_msg.field_of_view = 0.261799388; // 15deg in rad (HCSR04)
      range_msg.max_range = 4; // 4m
      range_msg.min_range = 2/100; // 2cm

      for (uint8_t x=0;x<8;x++) {
        char name[10];
        sprintf(name, "range%d", x);
        range_msg.header.frame_id = name;
        range_msg.range = (float)buf[x];
        ROS_INFO( "I2C read %s %d", name, buf[x] );
        range_pubs[x].publish(range_msg);
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  // Wait until shutdown signal recieved
  //ros::waitForShutdown();

  return 0;
}
