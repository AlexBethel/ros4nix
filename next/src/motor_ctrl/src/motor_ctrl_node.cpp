#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_raw/CanFrame.h>
#include <vector>
#include <cmath>
#include "main_bus.hpp"

const double PI = 3.14159;
ros::Publisher can_pub;
std::vector<ros::Subscriber> subscribers;
double wheel_diameter, wheel_base, max_rpm, frame_id;

void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    int rpm_left, rpm_right;
    double wheel_circumference = PI * wheel_diameter;
    // double v_left = msg->linear.x - (msg->angular.z * wheel_base / 2.0);
    // double v_right = msg->linear.x + (msg->angular.z * wheel_base / 2.0);
    double v_left = msg->linear.x - msg->angular.z;
    double v_right = msg->linear.x + msg->angular.z;

    rpm_left = (int) std::min(max_rpm, (v_left * 60) / wheel_circumference);
    rpm_right = (int) std::min(max_rpm, (v_right * 60) / wheel_circumference);
    std::cout << rpm_left << " " << rpm_right << "\n";

    can::MotorCommands cmd = {
        .left = {
            .speed = rpm_left
        },
        .right = {
            .speed = rpm_right
        }
    };
    uint8_t buffer[8];
    can::to_buffer(buffer, can::serialize(cmd));

    can_raw::CanFrame can_frame;
    can_frame.id = (short int) can::FrameID::MotorCommands;
    memcpy(&can_frame.data, buffer, sizeof(buffer));

    can_pub.publish(can_frame);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_ctrl_node");
    ros::NodeHandle nh;

    nh.getParam("wheel_diameter", wheel_diameter);
    nh.getParam("wheel_base", wheel_base);
    nh.getParam("max_rpm", max_rpm);
    nh.getParam("frame_id", frame_id);
    std::vector<std::string> topics;
    nh.getParam("topics", topics);

    for (const auto& topic : topics) {
        subscribers.push_back(nh.subscribe(topic, 10, callback));
    }

    can_pub = nh.advertise<can_raw::CanFrame>("/canbus", 10);

    ros::spin();
    return 0;
}

