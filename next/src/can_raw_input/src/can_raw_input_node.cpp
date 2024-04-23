#include <ostream>
#include <stdio.h>

#include <can_raw/CanFrame.h>
#include <ros/ros.h>

#include "can_interface.hpp"
#include "ros/publisher.h"

using can_raw::CanFrame;

int main(int argc, char **argv) {
  try {
    ros::init(argc, argv, "can_raw_input_node");
    ros::NodeHandle nh;

    auto pub = nh.advertise<CanFrame>("canbus_input", 16);

    SocketCAN can("can0");
    while (true) {
      auto frame = can.receive();

      CanFrame frame_ros;
      frame_ros.id = frame.can_id;
      for (int i = 0; i < frame.can_dlc; i++)
        frame_ros.data[i] = frame.data[i];

      pub.publish(frame_ros);
    }
  } catch (std::string error) {
    std::cerr << "can_raw_input error: " << error << '\n';
    return EXIT_FAILURE;
  }
}
