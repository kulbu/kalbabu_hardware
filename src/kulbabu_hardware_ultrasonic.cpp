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

  nh.param<std::string>("topic_name", topic_name, "range");
  nh.param<std::int>("i2c_adapter", i2c_adapter, 1);
  nh.param<std::int>("i2c_address", i2c_address, 0x10);
  nh.param<std::double>("field_of_view", field_of_view, 0.261799388); // 15deg in rad (HCSR04)
  nh.param<std::double>("max_range", max_range, 4.00);
  nh.param<std::double>("min_range", min_range, 0.02);
  nh.param<std::double>("publish_frequency", publish_frequency, 10.0);

  ros::Publisher range_pubs[8];

  for (uint8_t x=0;x<8;x++) {
    char name[10];
    sprintf(name, "%s%d", topic_name, x);
    range_pubs[x] = nh.advertise<sensor_msgs::Range>(name, 50);
  }

  // Setup i2c
  int file;
  char filename[20];

  snprintf(filename, 19, "/dev/i2c-%d", i2c_adapter);
  file = open(filename, O_RDWR);
  if (file < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    ROS_ERROR_STREAM( "I2C failed opening connection");
    exit(1);
  }

  if (ioctl(file, I2C_SLAVE, i2c_address) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    ROS_ERROR_STREAM( "I2C failed addressing connection");
    exit(1);
  }

  ros::Rate r(publish_frequency);
  while (nh.ok()) {
    ros::Time current_time = ros::Time::now();

    //uint8_t res;
    // Read data from i2c
    //res = i2c_smbus_read_word_data(file, addr);
    //if (res < 0) {

    uint8_t buf[8];
    buf[0] = i2c_address;

    if (read(file, buf, sizeof(buf)) != sizeof(buf)) {
      /* ERROR HANDLING: i2c transaction failed */
      ROS_WARN_STREAM( "I2C failed " << buf );
    } else {
      // https://github.com/ros/common_msgs/blob/indigo-devel/sensor_msgs/msg/Range.msg
      sensor_msgs::Range range_msg;
      range_msg.header.stamp = current_time;

      range_msg.radiation_type = 0; // Ultrasonic // FIXME: sensor_msgs::Range::ULTRASOUND
      range_msg.field_of_view = field_of_view;
      range_msg.max_range = max_range;
      range_msg.min_range = min_range;

      for (uint8_t x=0;x<8;x++) {
        char name[10];
        sprintf(name, "%s%d", topic_name, x);
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
