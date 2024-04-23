#include "main_bus.hpp"
#include <can_convert/ArmStatus.h>
#include <can_raw/CanFrame.h>
#include <cmath>
#include <ros/ros.h>
#include <vector>

ros::Subscriber can_sub;
ros::Publisher status_pub;

float arm_angle;
float bucket_angle;

double bucketAngle(double dist) {
  double dist_in = 0.03937008 * dist;
  double a = 21.5;
  double b = 8;
  double c = 17.72; //actuator
  double gamma = acos((a * a + b * b - c * c) / (2 * a * b));

  double c2 = c + dist_in;
  double angle = acos((a * a + b * b - c2 * c2) / (2 * a * b));
  return angle - gamma;
}

double armAngle(double dist) {
  double dist_in = 0.03937008 * dist;
  double a = 25.1875;
  double b = 17.1875;
  double c = 15.71; //actuator
  double gamma = acos((a * a + b * b - c * c) / (2 * a * b));

  double c2 = c + dist_in;
  double angle = acos((a * a + b * b - c2 * c2) / (2 * a * b));
  return angle - gamma;
}

void callback(const can_raw::CanFrame::ConstPtr &msg) {
  if (msg->id == (int)can::FrameID::ActuatorArmPos) {
    can::ActuatorArmPos arm_pos =
        can::ActuatorArmPos_deserialize(can::from_buffer(msg->data.data()));
    // TODO maybe check if left and right are different?
    arm_angle = armAngle(arm_pos.left_pos);
    arm_angle *= 180;
    arm_angle /= 3.14;
  }
  if (msg->id == (int)can::FrameID::ActuatorBucketPos) {
    can::ActuatorBucketPos bucket_pos =
        can::ActuatorBucketPos_deserialize(can::from_buffer(msg->data.data()));
    // TODO maybe check if left and right are different?
    bucket_angle = bucketAngle(bucket_pos.pos);
    bucket_angle *= 180;
    bucket_angle /= 3.14;
    bucket_angle -= 11.2;
    // std::cout << bucket_pos.pos << " " << bucket_angle << "\n";
  }
  can_convert::ArmStatus status;
  status.arm_angle = arm_angle;
  status.bucket_angle = bucket_angle;
  status_pub.publish(status);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_convert_node");
  ros::NodeHandle nh;

  can_sub = nh.subscribe("canbus_input", 10, callback);

  status_pub = nh.advertise<can_convert::ArmStatus>("/arm_status", 10);

  ros::spin();
  return 0;
}
