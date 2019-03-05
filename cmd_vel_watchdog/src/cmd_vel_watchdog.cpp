//
// This package creates node which tracks timeout for the cmd_vel message
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher *pub_ptr;
ros::Timer *global_timer;
geometry_msgs::Twist *stop_msg_ptr;

static const ros::Duration *DEFAULT_WATCHDOG_TIMEOUT;
void watchdogTimerCallback(const ros::TimerEvent &timerEvent);

void watchdogTimerCallback(const ros::TimerEvent &timerEvent){
    ROS_WARN_STREAM("velocity topic callback activated!");
    pub_ptr->publish(*stop_msg_ptr);
    ROS_WARN_STREAM("STOP MSG PUBLISHED!!!");
    global_timer->stop();
}

void initGlobals(ros::NodeHandle &nh){
    DEFAULT_WATCHDOG_TIMEOUT = new ros::Duration(3.0);

    stop_msg_ptr = new geometry_msgs::Twist();
    stop_msg_ptr->linear.x = 0;
    stop_msg_ptr->linear.y = 0;
    stop_msg_ptr->linear.z = 0;
    stop_msg_ptr->angular.x = 0;
    stop_msg_ptr->angular.y = 0;
    stop_msg_ptr->angular.z = 0;

    global_timer = new ros::Timer(nh.createTimer(*DEFAULT_WATCHDOG_TIMEOUT, watchdogTimerCallback));
    global_timer->start();
}

bool isZeroSpeed(const geometry_msgs::Twist &msgIn){
    return
    msgIn.angular.z != 0 &&
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
        global_timer->setPeriod(*DEFAULT_WATCHDOG_TIMEOUT, true);
        global_timer->start();
        ROS_DEBUG_STREAM("timer reset");
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_watchdog");
    ros::NodeHandle nh;

    initGlobals(nh);

    const std::string &default_velocity_topic = "cmd_vel";
    const std::string &this_node_name = ros::this_node::getName();
    ROS_INFO_STREAM(this_node_name << " created");

    pub_ptr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>(default_velocity_topic, 1));

    ROS_INFO_STREAM(this_node_name <<" advertised publisher to " <<pub_ptr->getTopic());

    ros::Subscriber sub  = nh.subscribe(default_velocity_topic, 1, &commandVelocityReceived);
    ROS_INFO_STREAM(this_node_name <<" subscribed to topic " << sub.getTopic());

    while(ros::ok()){
        if (sub.getNumPublishers() == 0){
            sub  = nh.subscribe(default_velocity_topic, 1, &commandVelocityReceived);
        }
//        ros::spinOnce();
ros::spin();
    }

    ROS_WARN_STREAM(this_node_name <<" SHUT_DOWN ");
    delete pub_ptr;
    delete global_timer;
    delete stop_msg_ptr;
}