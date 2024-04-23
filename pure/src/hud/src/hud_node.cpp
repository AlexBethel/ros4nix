#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <can_convert/ArmStatus.h>
#include <theora_image_transport/Packet.h>  // Required for Theora decoding

class HudNode {
public:
    HudNode()
    : it_(nh_) {
        // Initialize subscribers and publishers
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
            &HudNode::imageCb, this, image_transport::TransportHints("theora"));
        image_pub_ = it_.advertise("/camera_hud", 1);

        text_sub_ = nh_.subscribe<can_convert::ArmStatus>("/arm_status", 1, &HudNode::textCb, this);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Convert Theora Image to CV Image
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw text on the video stream
        cv::putText(cv_ptr->image, overlay_text, cv::Point(30, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void textCb(const can_convert::ArmStatus::ConstPtr& msg) {
        arm_angle = msg->arm_angle;
        bucket_angle = msg->bucket_angle;
        std::stringstream ss;
        ss << "Bucket Angle: " << int(-bucket_angle) << "°, Arm Angle: " << int(arm_angle) << "°";
        overlay_text = ss.str();
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber text_sub_;
    std::string overlay_text;
    float arm_angle = 0;
    float bucket_angle = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hud_node");
    HudNode csNode;
    ros::spin();
    return 0;
}
