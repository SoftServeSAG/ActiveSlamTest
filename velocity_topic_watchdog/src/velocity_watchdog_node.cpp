//
// This package creates node which tracks timeout for the cmd_vel message
//
#include <ros/ros.h>
#include "velocity_topic_watchdog/VelocityTopicWatchdog.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "velocity_topic_watchdog");
    ros::NodeHandle nh;

    velocity_topic_watchdog::VelocityTopicWatchdog watchdog(nh);

    ros::spin();

    ROS_WARN_STREAM("velocity_topic_watchdog node " << " SHUT_DOWN ");
    return 0;
}