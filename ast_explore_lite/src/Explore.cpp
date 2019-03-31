/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <thread>


#include "explore/Explore.h"
#include "explore/Frontier.h"

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{

Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout, hidden_timeout;
  int use_each_k_point;
  double min_frontier_size, max_frontier_angular_size; // parameters for FrontierSearch class
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  private_nh_.param("hidden_progress_timeout", hidden_timeout, 10.0);
  progress_timeout_ = ros::Duration(timeout);
  hidden_progress_timeout_ = ros::Duration(hidden_timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3); // todo why so strange coeff?
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);
  private_nh_.param("use_each_k_point", use_each_k_point, 1);
  private_nh_.param("max_frontier_angular_size", max_frontier_angular_size, 10.0);
  private_nh_.param("hidden_distance_threshold", hidden_distance_threshold, 3.0);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size,  use_each_k_point, max_frontier_angular_size,
                                                 hidden_distance_threshold);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

  std_msgs::ColorRGBA *blue;
  std_msgs::ColorRGBA *red;
  std_msgs::ColorRGBA *green;

    visualization_msgs::Marker *default_msg_template;

// todo consicer cleanup
void set_default_frontier_marker(visualization_msgs::Marker& m){
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
}


void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;
  m = *default_msg_template;
  m.header.frame_id = costmap_client_.getGlobalFrameID();

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.id = 0;

  m.action = visualization_msgs::Marker::DELETEALL;
  markers.push_back(m);
  ++m.id;
  m.action = visualization_msgs::Marker::ADD;

// TODO when the work on visualisation will stabilize, make a separate functuion or even subpackage for it
  for (auto& frontier : frontiers) {
    m.header.frame_id = costmap_client_.getGlobalFrameID();

      m.type = visualization_msgs::Marker::SPHERE_LIST;
      ++m.id;
      m.pose.position = {};
      m.scale.x = 0.2;
      m.scale.y = 0.2;
      m.scale.z = 0.2;
      m.points = {frontier.interpolated_line.first, frontier.middle, frontier.interpolated_line.second};
      m.color = *red;
      m.header.stamp = ros::Time::now();
      markers.push_back(m);

      m.type = visualization_msgs::Marker::LINE_STRIP;
    ++m.id;
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = {frontier.interpolated_line.first, frontier.middle, frontier.interpolated_line.second};
    m.color = *red;
    m.header.stamp = ros::Time::now();
    markers.push_back(m);

    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ++m.id;
    m.text = "F";
    m.pose.position = frontier.interpolated_line.first;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;
    m.points = {};
    m.color = *blue;
    m.header.stamp = ros::Time::now();
    markers.push_back(m);

    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ++m.id;
    m.text = "M";
    m.pose.position = frontier.middle;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;
    m.points = {};
    m.color = *blue;
    m.header.stamp = ros::Time::now();
    markers.push_back(m);

    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ++m.id;
    m.text = "L";
    m.pose.position = frontier.interpolated_line.second;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;
    m.points = {};
    m.color = *blue;
    m.header.stamp = ros::Time::now();
    markers.push_back(m);

    m.type = visualization_msgs::Marker::POINTS;
    ++m.id;
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    std::vector<geometry_msgs::Point> translated_vector{ frontier.vectors_to_points.begin(), frontier.vectors_to_points.end()};
    for (auto &i : translated_vector) { // todo introduce tenzor usage
        i.x += frontier.reference_robot_pose.x;
        i.y += frontier.reference_robot_pose.y;
    }
    m.points = translated_vector;
    if (goalOnBlacklist(frontier.middle))
      m.color = *red;
    else
      m.color = frontier.hidden ? *green : *blue;

    m.header.stamp = ros::Time::now();
    markers.push_back(m);
  }

  size_t current_markers_count = markers.size();

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    visualization_msgs::MarkerArray empty_marker_msg;
//    empty_marker_msg.markers
    marker_array_publisher_.publish(empty_marker_msg);
    visualizeFrontiers(frontiers);
    ROS_WARN_STREAM("NO FRONTIERS LEFT, PUBLISHING EMPTY LIST");
    stop();
    return;
  }

  // TODO move this and other frontiers init stuff to class constructor
  for (auto &fr: frontiers) {
    fr.hidden = frontier_exploration::Frontier::is_hidden(fr, hidden_distance_threshold);
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // TODO if there will be no use of this container -- remove it
  // find non blacklisted frontier
  std::vector<frontier_exploration::Frontier> hidden_frontiers {frontiers.size()};
  auto it = std::copy_if (frontiers.begin(), frontiers.end(), hidden_frontiers.begin(), [this](frontier_exploration::Frontier &fr){return fr.hidden;} );
//  auto hidden_n = ;
  hidden_frontiers.resize(std::distance(hidden_frontiers.begin(), it));

  auto frontier = std::find_if_not(hidden_frontiers.begin(), hidden_frontiers.end(),
                                       [this](const frontier_exploration::Frontier& f) {
                                           return goalOnBlacklist(f.middle);
                                       });
  // todo REFROMAT FUNCTION
  if (frontier == hidden_frontiers.end()) {
    // FIXME with current implementeation and recalculation on each planning step this is useless
    // TODO provide upon frontiers caching
    frontier =
            std::find_if_not(frontiers.begin(), frontiers.end(),
                             [this](const frontier_exploration::Frontier& f) {
                                 return goalOnBlacklist(f.middle);
                             }); // KD as frontiers stored sorted by their cost shold do the thing...
    if (frontier == frontiers.end()) {
      stop();
      return;
    }
  }

  geometry_msgs::Point target_position = frontier->middle;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;

  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }

  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_ ||
          (frontier->hidden && ros::Time::now() - last_progress_ > hidden_progress_timeout_) ) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }
  // TODO add heuristic to persue same goal while not achieved update
  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}


// FIXME KD: seems useless, as upon getting close to frontier we trigger replanning...
// todo needs renaming at least...
void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
//  marker_array_publisher_.publish();
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

void initialize_variables(){
// TODO rewrite in better style with alocator
  red = new std_msgs::ColorRGBA();
  blue = new std_msgs::ColorRGBA();
  green = new std_msgs::ColorRGBA();

  blue->r = 0;
  blue->g = 0;
  blue->b = 1.0;
  blue->a = 1.0;

  red->r = 1.0;
  red->g = 0;
  red->b = 0;
  red->a = 1.0;

  green->r = 0;
  green->g = 1.0;
  green->b = 0;
  green->a = 1.0;

  default_msg_template = new visualization_msgs::Marker();
  default_msg_template->ns = "frontiers";
  // lives forever
  default_msg_template->lifetime = ros::Duration(0);
  default_msg_template->frame_locked = false;
};

}  // namespace explore


int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  explore::initialize_variables();
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
