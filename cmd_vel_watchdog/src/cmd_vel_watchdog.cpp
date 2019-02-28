//
// Created by user on 2/28/19.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher *pub_ptr;

ros::Timer *global_timer;

geometry_msgs::Twist *stop_msg_ptr;

//once timer reaches the shutdown limit, timer callback will be activated

void commandVelocityReceived(const geometry_msgs::Twist &msgIn){
    // TODO create callback
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_watchdog");
    ros::NodeHandle nh;

    stop_msg_ptr = new geometry_msgs::Twist();
    stop_msg_ptr->linear.x = 0;
    stop_msg_ptr->linear.y = 0;
    stop_msg_ptr->linear.z = 0;
    stop_msg_ptr->angular.x = 0;
    stop_msg_ptr->angular.y = 0;
    stop_msg_ptr->angular.z = 0;

    const std::string &this_node_name = ros::this_node::getName();
    ROS_INFO_STREAM(this_node_name << " created");

    pub_ptr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1));

    ROS_INFO_STREAM(this_node_name <<" advertised publisher to " <<pub_ptr->getTopic());

    ros::Subscriber sub  = nh.subscribe("cmd_vel", 1, &commandVelocityReceived);
    ROS_INFO_STREAM(this_node_name <<" subscribed to topic " << sub.getTopic());


    while(ros::ok()){
        // TODO declare logic if needed
    }

    ROS_WARN_STREAM(this_node_name <<" SHUT_DOWN ");
    delete pub_ptr;
    delete global_timer;
    delete stop_msg;
}