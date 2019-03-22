//
// Created by user on 3/21/19.
//

#ifndef PROJECT_VELOCITY_TOPIC_WATCHDOG_H
#define PROJECT_VELOCITY_TOPIC_WATCHDOG_H


class VelocityTopicWatchdog{
//VelocityTopicWatchdog(ros::NodeHandle &nh);
explicit VelocityTopicWatchdog(ros::NodeHandle &nh);
~VelocityTopicWatchdog() = default;


private:
    geometry_msgs::Twist makeTwistMsg(
            double lx = 0.0,
            double ly = 0.0,
            double lz = 0.0,
            double ax = 0.0,
            double ay = 0.0,
            double az = 0.0);

    ros::NodeHandle& nh_;
    std::string resolved_node_name;
    ros::Publisher publisher_;
    std::string subscriberTopic_ = "cmd_vel";
    ros::Subscriber subscriber_;
    ros::Timer watchdog_timer_;
    geometry_msgs::Twist stop_msg_;
    ros::Duration watchdog_duration_;

    double watchdog_timeout_param_ = 3.0; // default

    /*!
 * Reads and verifies the ROS parameters.
 * @return true if successful.
 */
    bool readParameters();
bool isZeroSpeed(const geometry_msgs::Twist &msgIn);
void commandVelocityReceived(const geometry_msgs::Twist &msgIn);
void watchdogTimerCallback(const ros::TimerEvent &timerEvent);

};

#endif //PROJECT_VELOCITY_TOPIC_WATCHDOG_H
