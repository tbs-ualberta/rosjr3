
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

#define FILTER0 2
#define FILTER1 3
#define FILTER2 4
#define FILTER3 5
#define FILTER4 6
#define FILTER5 7
#define FILTER6 8

int main(int argc, char **argv) {
  six_axis_array fm0, fm1;
  force_array fs;
  int ret, i, fd;

  ros::init(argc, argv, "rosjr3");

  ros::NodeHandle n;

  ros::Publisher frctrq0_pub =
      n.advertise<geometry_msgs::WrenchStamped>("jr3ft0", 10);
  ros::Publisher frctrq1_pub =
      n.advertise<geometry_msgs::WrenchStamped>("jr3ft1", 10);

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
  ret = ioctl(fd, IOCTL0_JR3_ZEROOFFS);
  ret = ioctl(fd, IOCTL1_JR3_ZEROOFFS);

  while (ros::ok()) {
    // --- Read the current forces/torques (raw 14 bit ADC values) ---
    switch (num_filter) {
    case FILTER0:
      ret = ioctl(fd, IOCTL0_JR3_FILTER0, &fm0);
      ret = ioctl(fd, IOCTL1_JR3_FILTER0, &fm1);
      break;
    case FILTER1:
      ret = ioctl(fd, IOCTL0_JR3_FILTER1, &fm0);
      ret = ioctl(fd, IOCTL1_JR3_FILTER1, &fm1);
      break;
    case FILTER2:
      ret = ioctl(fd, IOCTL0_JR3_FILTER2, &fm0);
      ret = ioctl(fd, IOCTL1_JR3_FILTER2, &fm1);
      break;
    case FILTER3:
      ret = ioctl(fd, IOCTL0_JR3_FILTER3, &fm0);
      ret = ioctl(fd, IOCTL1_JR3_FILTER3, &fm1);
      break;
    case FILTER4:
      ret = ioctl(fd, IOCTL0_JR3_FILTER4, &fm0);
      ret = ioctl(fd, IOCTL1_JR3_FILTER4, &fm1);
      break;
    case FILTER5:
      ret = ioctl(fd, IOCTL0_JR3_FILTER5, &fm0);
      ret = ioctl(fd, IOCTL1_JR3_FILTER5, &fm1);
      break;
    case FILTER6:
      ret = ioctl(fd, IOCTL0_JR3_FILTER6, &fm0);
      ret = ioctl(fd, IOCTL1_JR3_FILTER6, &fm1);
      break;
    }

    if (ret != -1) {
      // --- Construct the messages ---
      geometry_msgs::WrenchStamped msg_wrench0;
      geometry_msgs::WrenchStamped msg_wrench1;
      ros::Time time_now = ros::Time::now();
      msg_wrench0.header.stamp = time_now;
      msg_wrench0.header.frame_id = "base_link";
      msg_wrench0.wrench.force.x = (float)fm0.f[0] * fs.f[0] / 16384;
      msg_wrench0.wrench.force.y = (float)fm0.f[1] * fs.f[1] / 16384;
      msg_wrench0.wrench.force.z = (float)fm0.f[2] * fs.f[2] / 16384;
      msg_wrench0.wrench.torque.x = (float)fm0.m[0] * fs.m[0] / 16384;
      msg_wrench0.wrench.torque.y = (float)fm0.m[1] * fs.m[1] / 16384;
      msg_wrench0.wrench.torque.z = (float)fm0.m[2] * fs.m[2] / 16384;
      msg_wrench1.header.stamp = time_now;
      msg_wrench1.header.frame_id = "base_link";
      msg_wrench1.wrench.force.x = (float)fm1.f[0] * fs.f[0] / 16384;
      msg_wrench1.wrench.force.y = (float)fm1.f[1] * fs.f[1] / 16384;
      msg_wrench1.wrench.force.z = (float)fm1.f[2] * fs.f[2] / 16384;
      msg_wrench1.wrench.torque.x = (float)fm1.m[0] * fs.m[0] / 16384;
      msg_wrench1.wrench.torque.y = (float)fm1.m[1] * fs.m[1] / 16384;
      msg_wrench1.wrench.torque.z = (float)fm1.m[2] * fs.m[2] / 16384;

      // --- Publish the messages ---
      frctrq0_pub.publish(msg_wrench0);
      frctrq1_pub.publish(msg_wrench1);

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
