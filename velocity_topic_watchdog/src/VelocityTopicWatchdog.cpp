//
// This package creates node which tracks timeout for the cmd_vel message
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <velocity_topic_watchdog/VelocityTopicWatchdog.hpp>
#include <memory>

namespace velocity_topic_watchdog{

geometry_msgs::Twist VelocityTopicWatchdog::makeTwistMsg(
        const double lx,
        const double ly,
        const double lz,
        const double ax,
        const double ay,
        const double az) {
    geometry_msgs::Twist twist = geometry_msgs::Twist();
    twist.linear.x = lx;
    twist.linear.y = ly;
    twist.linear.z = lz;
    twist.angular.x = ax;
    twist.angular.y = ay;
    twist.angular.z = az;
    return twist;
}

VelocityTopicWatchdog::VelocityTopicWatchdog(ros::NodeHandle &nh)
:
nh_(nh),
subscriberTopic_("cmd_vel"),
stop_msg_(makeTwistMsg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
{

    if (!readParameters()) {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    resolved_node_name_ = ros::this_node::getName();

    subscriber_ = nh_.subscribe(subscriberTopic_, 1,
                                        &VelocityTopicWatchdog::commandVelocityReceived, this);

    ROS_INFO_STREAM( ros::this_node::getName() <<" subscribed to topic " << subscriber_.getTopic());
    publisher_ = nh.advertise<geometry_msgs::Twist>(subscriberTopic_, 1, true); // true for 'latched'
    ROS_INFO_STREAM(resolved_node_name_ <<" advertised publisher to " << publisher_.getTopic());

    watchdog_duration_ = ros::Duration(watchdog_timeout_param_);
    watchdog_timer_ = nh.createTimer(watchdog_duration_, &VelocityTopicWatchdog::watchdogTimerCallback, this);
    ROS_INFO_STREAM("watchdog time set to " << watchdog_timeout_param_ << " [sec]");
    watchdog_timer_.start();

    ROS_INFO_STREAM("Successfully launched node. [" << resolved_node_name_ << "]");
}

bool VelocityTopicWatchdog::readParameters() {
    bool status = (nh_.getParam("subscriber_topic", subscriberTopic_));
    status &= (nh_.getParam("watchdog_patience", watchdog_timeout_param_));
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
        watchdog_timer_.setPeriod(watchdog_duration_, true);
        watchdog_timer_.start();
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
// Once timer since last message runs out callback activates
void VelocityTopicWatchdog::watchdogTimerCallback(const ros::TimerEvent &timerEvent) {
    ROS_INFO_STREAM("velocity_topic_watchdog activated!");
    publisher_.publish(stop_msg_);
    ROS_INFO_STREAM("STOP MSG PUBLISHED to [" << publisher_.getTopic() << "]");
    watchdog_timer_.stop(); // resets timer
}

}  // namespace velocity_topic_watchdog
