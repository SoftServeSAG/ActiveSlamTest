//
// This package creates node which tracks timeout for the cmd_vel message
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace cmd_vel_watchdog{

ros::Publisher *pub_ptr;
ros::Timer *global_timer;
geometry_msgs::Twist *stop_msg_ptr;

static const double DEFAULT_WATCHDOG_TIMEOUT (3.0);
ros::Duration *watchdog_duration;
double watchdog_timeout_param (3.0);

void watchdogTimerCallback(const ros::TimerEvent &timerEvent){
    ROS_WARN_STREAM("velocity topic watchdog activated!");
    pub_ptr->publish(*stop_msg_ptr);
    ROS_WARN_STREAM("STOP MSG PUBLISHED to [" << pub_ptr->getTopic() << "]");
    global_timer->stop();
}

void initGlobals(ros::NodeHandle &nh){
    watchdog_duration = new ros::Duration(watchdog_timeout_param);

    stop_msg_ptr = new geometry_msgs::Twist();
    stop_msg_ptr->linear.x = 0;
    stop_msg_ptr->linear.y = 0;
    stop_msg_ptr->linear.z = 0;
    stop_msg_ptr->angular.x = 0;
    stop_msg_ptr->angular.y = 0;
    stop_msg_ptr->angular.z = 0;

    global_timer = new ros::Timer(nh.createTimer(*watchdog_duration, watchdogTimerCallback));
    global_timer->start();
}

bool isZeroSpeed(const geometry_msgs::Twist &msgIn){
    return msgIn.angular.z != 0 &&
    msgIn.linear.x != 0 &&
    msgIn.angular.x != 0 &&
    msgIn.angular.y != 0 &&
    msgIn.linear.x != 0  &&
    msgIn.linear.y != 0 ;
}


// resetting timer once received massage
void commandVelocityReceived(const geometry_msgs::Twist &msgIn){
    ROS_DEBUG_STREAM(
            "velocity msg received" << " Linear= " <<
                                    msgIn.linear.x <<" " << msgIn.linear.y <<" " << msgIn.linear.z <<
                                    " Angular= " <<
                                    msgIn.angular.x <<" " << msgIn.angular.y <<" " << msgIn.angular.z
    );
    if ( ! isZeroSpeed(msgIn) ) {
        global_timer->setPeriod(*watchdog_duration, true);
        global_timer->start();
        ROS_DEBUG_STREAM("timer reset");
    }
}


} // namespace cmd_vel_watchdog
int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_watchdog");
    ros::NodeHandle nh;

    if (!nh.getParam("watchdog_patience", cmd_vel_watchdog::watchdog_timeout_param)){
        ROS_INFO_STREAM(
                "watchdog_patience param is not found, setting to default = " <<
                                                                              cmd_vel_watchdog::DEFAULT_WATCHDOG_TIMEOUT << " sec");
        cmd_vel_watchdog::watchdog_timeout_param = cmd_vel_watchdog::DEFAULT_WATCHDOG_TIMEOUT;
    }
    ROS_INFO_STREAM("watchdog time set to " << cmd_vel_watchdog::watchdog_timeout_param << " [sec]");

    cmd_vel_watchdog::initGlobals(nh);



    const std::string &default_velocity_topic = "cmd_vel";
    const std::string &this_node_name = ros::this_node::getName();
    ROS_INFO_STREAM(this_node_name << " created");

    cmd_vel_watchdog::pub_ptr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>(default_velocity_topic, 1));

    ROS_INFO_STREAM(this_node_name <<" advertised publisher to " << cmd_vel_watchdog::pub_ptr->getTopic());

    ros::Subscriber sub  = nh.subscribe(default_velocity_topic, 1, &cmd_vel_watchdog::commandVelocityReceived);
    ROS_INFO_STREAM(this_node_name <<" subscribed to topic " << sub.getTopic());

    while(ros::ok()){
        ros::spin();
    }

    ROS_WARN_STREAM(this_node_name <<" SHUT_DOWN ");
    delete cmd_vel_watchdog::pub_ptr;
    delete cmd_vel_watchdog::global_timer;
    delete cmd_vel_watchdog::stop_msg_ptr;
    delete cmd_vel_watchdog::watchdog_duration;
}