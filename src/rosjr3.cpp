
// JR3 sensor driver includes
#include <fcntl.h>
#include <jr3pci-ioctl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

#define FILTER0 0
#define FILTER1 1
#define FILTER2 2
#define FILTER3 3
#define FILTER4 4
#define FILTER5 5
#define FILTER6 6
#define SENSOR0 0
#define SENSOR1 1

int main(int argc, char **argv) {
  six_axis_array fm;
  force_array fs;
  int ret, i, fd;

  ros::init(argc, argv, "rosjr3");

  ros::NodeHandle n;

  ros::Publisher frctrq_pub =
      n.advertise<geometry_msgs::WrenchStamped>("jr3ft", 10);

  // --- Load and apply parameters ---
  int rate_hz = 1000;
  n.getParam("rosjr3/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);

  std::vector<int> full_scales = {100, 100, 200, 5, 5, 5};
  if (n.getParam("rosjr3/full_scales", full_scales)) {
    for (i = 0; i < 3; i++) {
      fs.f[i] = full_scales[i];
      fs.m[i] = full_scales[i + 3];
    }
  }

  int num_sensor = 0;
  n.getParam("rosjr3/num_sensor", num_sensor);
  ROS_INFO("num_sensor = %d", num_sensor);

  int num_filter = 0;
  n.getParam("rosjr3/num_filter", num_filter);
  ROS_INFO("num_filter = %d", num_filter);

  // --- Try to open the device ---
  if ((fd = open("/dev/jr3", O_RDWR)) < 0) {
    ROS_ERROR("Can't open device. No way to read force!");
    ROS_INFO("Shutting down rosjr3 node.");
    ros::shutdown();
    return -1;
  }

  // --- Remove the offsets ---
  switch (num_sensor) {
  case SENSOR0:
    ret = ioctl(fd, IOCTL0_JR3_ZEROOFFS);
    break;
  case SENSOR1:
    ret = ioctl(fd, IOCTL1_JR3_ZEROOFFS);
    break;
  }

  while (ros::ok()) {
    // --- Read the current forces/torques (raw 14 bit ADC values) ---
    switch (num_sensor) {
    case SENSOR0:
      switch (num_filter) {
      case FILTER0:
        ret = ioctl(fd, IOCTL0_JR3_FILTER0, &fm);
        break;
      case FILTER1:
        ret = ioctl(fd, IOCTL0_JR3_FILTER1, &fm);
        break;
      case FILTER2:
        ret = ioctl(fd, IOCTL0_JR3_FILTER2, &fm);
        break;
      case FILTER3:
        ret = ioctl(fd, IOCTL0_JR3_FILTER3, &fm);
        break;
      case FILTER4:
        ret = ioctl(fd, IOCTL0_JR3_FILTER4, &fm);
        break;
      case FILTER5:
        ret = ioctl(fd, IOCTL0_JR3_FILTER5, &fm);
        break;
      case FILTER6:
        ret = ioctl(fd, IOCTL0_JR3_FILTER6, &fm);
        break;
      }
      break;
    case SENSOR1:
      switch (num_filter) {
      case FILTER0:
        ret = ioctl(fd, IOCTL1_JR3_FILTER0, &fm);
        break;
      case FILTER1:
        ret = ioctl(fd, IOCTL1_JR3_FILTER1, &fm);
        break;
      case FILTER2:
        ret = ioctl(fd, IOCTL1_JR3_FILTER2, &fm);
        break;
      case FILTER3:
        ret = ioctl(fd, IOCTL1_JR3_FILTER3, &fm);
        break;
      case FILTER4:
        ret = ioctl(fd, IOCTL1_JR3_FILTER4, &fm);
        break;
      case FILTER5:
        ret = ioctl(fd, IOCTL1_JR3_FILTER5, &fm);
        break;
      case FILTER6:
        ret = ioctl(fd, IOCTL1_JR3_FILTER6, &fm);
        break;
      }
      break;
    }

    if (ret != -1) {
      // --- Construct the message ---
      geometry_msgs::WrenchStamped msg_wrench;
      msg_wrench.header.stamp = ros::Time::now();
      msg_wrench.header.frame_id = "base_link";
      msg_wrench.wrench.force.x = (float)fm.f[0] * fs.f[0] / 16384;
      msg_wrench.wrench.force.y = (float)fm.f[1] * fs.f[1] / 16384;
      msg_wrench.wrench.force.z = (float)fm.f[2] * fs.f[2] / 16384;
      msg_wrench.wrench.torque.x = (float)fm.m[0] * fs.m[0] / 16384;
      msg_wrench.wrench.torque.y = (float)fm.m[1] * fs.m[1] / 16384;
      msg_wrench.wrench.torque.z = (float)fm.m[2] * fs.m[2] / 16384;

      // --- Publish the message ---
      frctrq_pub.publish(msg_wrench);

      ros::spinOnce();
      loop_rate.sleep();
    } else {
      // An error occurred while reading data and we need to stop
      ROS_ERROR("JR3 driver returned an error.");
      ROS_INFO("Shutting down rosjr3 node.");
      ros::shutdown();
      return -1;
    }
  }
  // TODO This does not seem to be actually called ... figure out why
  ROS_INFO("Shutting down rosjr3 node.");
  close(fd);
}
