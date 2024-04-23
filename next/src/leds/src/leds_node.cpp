#include "ros/init.h"
#include <ros/ros.h>

#include "main_bus.hpp"
#include "ros/publisher.h"
#include "std_msgs/Bool.h"
#include <can_raw/CanFrame.h>

class Node {
  ros::NodeHandle nh;
  std::vector<ros::Subscriber> subscribers;
  ros::Publisher can_publisher;

  can::Lights lights_status = {0};

public:
  Node() {
    can_publisher = nh.advertise<can_raw::CanFrame>("canbus", 16);

#define sub(kind)                                                              \
  subscribers.push_back(nh.subscribe("leds/" #kind, 16, &Node::kind, this))

    sub(error);
    sub(autonomy);
    sub(motion);

#undef sub
  }

  void republish() {
    can_raw::CanFrame frame;
    frame.id = (int)can::FrameID::Lights;
    can::to_buffer(frame.data.data(), can::serialize(lights_status));
    can_publisher.publish(frame);
  }

  // ROS makes these *really* verbose, so ima use a macro
#define repub_msg(kind)                                                        \
  void kind(std_msgs::Bool b) {                                                \
    lights_status.kind = b.data;                                               \
    republish();                                                               \
  }

  repub_msg(error);
  repub_msg(autonomy);
  repub_msg(motion);

#undef repub_msg
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "leds_node");
  Node node;
  ros::spin();
}
