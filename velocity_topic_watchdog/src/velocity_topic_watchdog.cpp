//
// This package creates node which tracks timeout for the cmd_vel message
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <velocity_topic_watchdog/velocity_topic_watchdog.h>
#include <memory>


geometry_msgs::TwistPtr VelocityTopicWatchdog::makeTwistMsg(
        const double lx,
        const double ly,
        const double lz,
        const double ax,
        const double ay,
        const double az) {
    geometry_msgs::TwistPtr twistPtr = geometry_msgs::TwistPtr();
    twistPtr->linear.x = lx;
    twistPtr->linear.y = ly;
    twistPtr->linear.z = lz;
    twistPtr->angular.x = ax;
    twistPtr->angular.y = ay;
    twistPtr->angular.z = az;
    return twistPtr;
}

VelocityTopicWatchdog::VelocityTopicWatchdog(ros::NodeHandle &nh) : nh_(nh){
    if (!readParameters()) {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    subscriber_ = nh_.subscribe(subscriberTopic_, 1,
                                        &VelocityTopicWatchdog::commandVelocityReceived, this);
    publisher_ = nh.advertise<geometry_msgs::Twist>(subscriberTopic_, 1, true); // true for 'latched'
    const std::string &this_node_name = ros::this_node::getName();
    ROS_INFO_STREAM("Successfully launched node. [" << this_node_name << "]");
}

bool VelocityTopicWatchdog::readParameters() {
    bool status = (!nh_.getParam("subscriber_topic", subscriberTopic_));
    status &= (!nh_.getParam("watchdog_patience", watchdog_timeout_param_));
    ROS_ASSERT_MSG(watchdog_timeout_param_ > 0,
            "Watchdog_timeout should be positive.  Value = %f", watchdog_timeout_param_);
    return status;
}

void VelocityTopicWatchdog::commandVelocityReceived(const geometry_msgs::Twist &msgIn) {
    ROS_DEBUG_STREAM(
            "velocity msg received" << " Linear= " <<
                                    msgIn.linear.x <<" " << msgIn.linear.y <<" " << msgIn.linear.z <<
                                    " Angular= " <<
                                    msgIn.angular.x <<" " << msgIn.angular.y <<" " << msgIn.angular.z
    );
    if ( !isZeroSpeed(msgIn) ) {
        global_timer->setPeriod(*watchdog_duration, true);
        global_timer->start();
        ROS_DEBUG("timer reset");
    }
}

bool VelocityTopicWatchdog::isZeroSpeed(const geometry_msgs::Twist &msgIn){
    return msgIn.angular.z != 0 &&
           msgIn.linear.x != 0 &&
           msgIn.angular.x != 0 &&
           msgIn.angular.y != 0 &&
           msgIn.linear.x != 0  &&
           msgIn.linear.y != 0 ;
}

namespace velocity_topic_watchdog{

    ros::Publisher *pub_ptr;
    ros::Timer *global_timer;
    geometry_msgs::Twist *stop_msg_ptr;

static const double DEFAULT_WATCHDOG_TIMEOUT (3.0);
ros::Duration *watchdog_duration;
double watchdog_timeout_param (3.0);

// Once timer since last message runs out callback activates
void watchdogTimerCallback(const ros::TimerEvent &timerEvent){
    ROS_INFO_STREAM("velocity_topic_watchdog activated!");
    pub_ptr->publish(*stop_msg_ptr);
    ROS_INFO_STREAM("STOP MSG PUBLISHED to [" << pub_ptr->getTopic() << "]");
    global_timer->stop();
}

void initGlobals(ros::NodeHandle &nh){
    watchdog_duration = new ros::Duration(watchdog_timeout_param);

    // initializing stop_msg fields
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


} // namespace velocity_topic_watchdog

int main(int argc, char** argv){
    ros::init(argc, argv, "velocity_topic_watchdog");
    ros::NodeHandle nh;

    if (!nh.getParam("watchdog_patience", velocity_topic_watchdog::watchdog_timeout_param)){
        ROS_INFO_STREAM(
                "watchdog_patience param is not found, setting to default = " <<
                                                                              velocity_topic_watchdog::DEFAULT_WATCHDOG_TIMEOUT << " sec");
        velocity_topic_watchdog::watchdog_timeout_param = velocity_topic_watchdog::DEFAULT_WATCHDOG_TIMEOUT;
    }
    ROS_INFO_STREAM("watchdog time set to " << velocity_topic_watchdog::watchdog_timeout_param << " [sec]");

    velocity_topic_watchdog::initGlobals(nh);



    const std::string &default_velocity_topic = "cmd_vel";
    const std::string &this_node_name = ros::this_node::getName();
    ROS_INFO_STREAM(this_node_name << " created");

    velocity_topic_watchdog::pub_ptr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>(default_velocity_topic, 1));

    ROS_INFO_STREAM(this_node_name <<" advertised publisher to " << velocity_topic_watchdog::pub_ptr->getTopic());

    ros::Subscriber sub  = nh.subscribe(default_velocity_topic, 1, &velocity_topic_watchdog::commandVelocityReceived);
    ROS_INFO_STREAM(this_node_name <<" subscribed to topic " << sub.getTopic());

    while(ros::ok()){
        ros::spin();
    }

    ROS_WARN_STREAM(this_node_name <<" SHUT_DOWN ");
    delete velocity_topic_watchdog::pub_ptr;
    delete velocity_topic_watchdog::global_timer;
    delete velocity_topic_watchdog::stop_msg_ptr;
    delete velocity_topic_watchdog::watchdog_duration;
}