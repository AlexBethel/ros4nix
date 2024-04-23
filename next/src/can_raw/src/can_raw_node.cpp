#include <ostream>
#include <stdio.h>

#include <can_raw/CanFrame.h>
#include <ros/ros.h>

#include "can_interface.hpp"

class Node {
  ros::NodeHandle nh;
  ros::Subscriber subscriber;
  SocketCAN socket;

public:
  Node() {
    subscriber = nh.subscribe("/canbus", 16, &Node::handle_message, this);
    socket = SocketCAN("can0");
  }

  void handle_message(const can_raw::CanFrame &frame) {
    socket.transmit((int)frame.id, (const uint8_t *)frame.data.data());
  }
};

int main(int argc, char **argv) {
  try {
    ros::init(argc, argv, "can_raw_node");

    Node node;
    ros::spin();
  } catch (std::string error) {
    std::cerr << "can_raw error: " << error << '\n';
    return EXIT_FAILURE;
  }
}
