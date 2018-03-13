
// JR3 sensor driver includes
#include <fcntl.h>
#include <jr3pci-ioctl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// UDP includes
#include <data_transmission.h>
#include <string.h>

// ROS includes
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"

#define SENSOR0 0
#define SENSOR1 1
#define FILTER0 2
#define FILTER1 3
#define FILTER2 4
#define FILTER3 5
#define FILTER4 6
#define FILTER5 7
#define FILTER6 8

// Globals
data_transmission _transmission;

int main(int argc, char **argv) {
  six_axis_array fm;
  force_array fs;
  int ret, i, fd;

  ros::init(argc, argv, "rosjr3");

  ros::NodeHandle n;

  ros::Publisher frctrq_pub =
      n.advertise<geometry_msgs::WrenchStamped>("jr3ft", 1);

  // --- Load and apply parameters ---
  int rate_hz = 1000;
  n.getParam("rosjr3_udp/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);

  // Sensor parameters
  std::vector<int> full_scales = {100, 100, 200, 5, 5, 5};
  if (n.getParam("rosjr3_udp/full_scales", full_scales)) {
    for (i = 0; i < 3; i++) {
      fs.f[i] = full_scales[i];
      fs.m[i] = full_scales[i + 3];
    }
  }
  int num_sensor = 0;
  n.getParam("rosjr3_udp/num_sensor", num_sensor);
  ROS_INFO("num_sensor = %d", num_sensor);
  int num_filter = 0;
  n.getParam("rosjr3_udp/num_filter", num_filter);
  ROS_INFO("num_filter = %d", num_filter);

  // Transmission parameters
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("rosjr3_udp/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7777;
  if (!n.getParam("rosjr3_udp/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  string ip_remote_st = "0.0.0.0";
  if (!n.getParam("rosjr3_udp/ip_rem", ip_remote_st)) {
    ROS_ERROR("Parameter \"ip_rem\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_remote_si = 7777;
  if (!n.getParam("rosjr3_udp/port_rem", port_remote_si)) {
    ROS_ERROR("Parameter \"port_rem\" not provided. Exiting.");
    ros::shutdown();
  }

  // --- Try to open the device ---
  if ((fd = open("/dev/jr3", O_RDWR)) < 0) {
    ROS_ERROR("Can't open device. No way to read force!");
    ROS_INFO("Shutting down rosjr3 node.");
    ros::shutdown();
    return -1;
  }

  // --- Setup the transmission ---
  char ip_local_scp[1024];
  char ip_remote_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  strcpy(ip_remote_scp, ip_remote_st.c_str());
  _transmission.init_transmission(ip_local_scp, port_local_si, ip_remote_scp,
                                  port_remote_si);

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
      double msg_array[6] = {
          msg_wrench.wrench.force.x,  msg_wrench.wrench.force.y,
          msg_wrench.wrench.force.z,  msg_wrench.wrench.torque.x,
          msg_wrench.wrench.torque.y, msg_wrench.wrench.torque.z,
      };

      // --- Publish and transmit (UDP) the message ---
      frctrq_pub.publish(msg_wrench);
      _transmission.send(msg_array, 6);

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
