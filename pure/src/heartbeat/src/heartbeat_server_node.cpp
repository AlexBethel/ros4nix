#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heartbeat_server_node");
    ros::NodeHandle n;

    ros::Publisher heartbeat_pub = n.advertise<std_msgs::Empty>("/heartbeat", 1000);
    
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        std_msgs::Empty msg;
        
        heartbeat_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
