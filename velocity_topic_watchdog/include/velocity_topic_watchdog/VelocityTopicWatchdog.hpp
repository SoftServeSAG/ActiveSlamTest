//
// Created by user on 3/21/19.
//

#ifndef PROJECT_VELOCITY_TOPIC_WATCHDOG_H
#define PROJECT_VELOCITY_TOPIC_WATCHDOG_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


namespace velocity_topic_watchdog{

    /*!
     * @class VelocityTopicWatchdog
     * @details should be used with the velocity topic and publish "STOP" messages when the last published message is considered too old
     *  best to use as the part of the "bringup" robot launcher to work together with roscore.
     *  -- also can be used as a daemon for a ROS-bot, but then requires launching roscore upon bringup
     * @listen some velocity topic
     * @publish same velocity topic which is listening
     */
class VelocityTopicWatchdog{
public:
    explicit VelocityTopicWatchdog(ros::NodeHandle &nh);
    ~VelocityTopicWatchdog() = default;

private:

    // buffers for reading parameters
    double watchdog_timeout_param_;
    std::string subscriberTopic_;

    // to avoid creation of same msg each time
    const geometry_msgs::Twist stop_msg_;

    ros::NodeHandle& nh_;
    std::string resolved_node_name_;

    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::Timer watchdog_timer_;
    ros::Duration watchdog_duration_;


    /*!
 * Reads and verifies the ROS parameters.
 * @return true if successful.
 */
    bool readParameters();

/**
 * @brief helper function to check this Twist message means "stop" command (all fields are zeroes)
 * @param msgIn message we are considering in this function
 * @return if this message corresponds to steady state (all fields are zeros)
 */
bool isZeroSpeed(const geometry_msgs::Twist &msgIn) const;
/*!
 * @brief updates state of a watchdog upon receiving message
 * resets watchdog timer
 *
 * @param msgIn message from topic that triggered this callback
 */
void commandVelocityReceived(const geometry_msgs::Twist &msgIn);

/*!
 * @details once timer goes out, watchdog starts to publish "STOP" messages
 * @param timerEvent timer event triggered this callback
 */
void watchdogTimerCallback(const ros::TimerEvent &timerEvent);

/*!
 * @brief Helper function for tidy creation of geometry_msgs
 * @details serves as replacement of constructor with parameters
 * @param lx linear x
 * @param ly linear y
 * @param lz linear z
 * @param ax angular x
 * @param ay angular y
 * @param az angular z
 * @return constructed message with instantiated fields
 */
geometry_msgs::Twist makeTwistMsg(
        double lx = 0.0,
        double ly = 0.0,
        double lz = 0.0,
        double ax = 0.0,
        double ay = 0.0,
        double az = 0.0);


};

} // namespace velocity_topic_watchdog


#endif //PROJECT_VELOCITY_TOPIC_WATCHDOG_H
