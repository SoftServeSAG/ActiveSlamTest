#include <explore/frontier_search.h>

#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <explore/costmap_tools.h>

#include <boost/geometry.hpp>
#include <geometry_msgs/Vector3.h>
#include <cmath>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
{
}

    std::pair<geometry_msgs::Point, geometry_msgs::Point> FrontierSearch::approxFrontierPlanarFarthest(Frontier &fr,
                                                                                                       geometry_msgs::Point &reference_robot){
        double max_dist = 0.0;
        double heuristics_dist {0.0};
        geometry_msgs::Point p1, p2;
      for (auto &point: fr.points){
          heuristics_dist =
                  std::hypot(point.x - reference_robot.x , point.y - reference_robot.y) +  /* robot_point_dist */
                  std::hypot(point.x - fr.centroid.x , point.y - fr.centroid.y); /* centroid_point dist */
          if (heuristics_dist > max_dist){
              max_dist = heuristics_dist;
              p1 = point;
          }
      }
      max_dist = 0.0;
       for (auto &point: fr.points){
           heuristics_dist =
                   std::hypot(point.x - p1.x , point.y - p1.y) +  /*  point-point dist */
                   std::hypot(point.x - fr.centroid.x , point.y - fr.centroid.y); /* centroid_point dist */
            if (heuristics_dist > max_dist){
                max_dist = heuristics_dist;
                p2 = point;
            }
        }
        return {p1, p2};
}


    std::pair<geometry_msgs::Point, geometry_msgs::Point> FrontierSearch::approximateFrontierViewAngle(
            frontier_exploration::Frontier &fr, geometry_msgs::Point &reference_robot) {
        geometry_msgs::Point p1, p2;
        if (!fr.vectors_to_points.empty()){
            p1.x = fr.vectors_to_points.front().x + reference_robot.x;
            p1.y = fr.vectors_to_points.front().y + reference_robot.y;
            p2.x = fr.vectors_to_points.back().x + reference_robot.x;
            p2.y = fr.vectors_to_points.back().y + reference_robot.y;
            ROS_DEBUG_STREAM( " should " <<  fr.vectors_to_points.front() << fr.vectors_to_points.back());
        } // else default zero values
        return {p1, p2};
    }

    std::vector<Frontier> splitFrontier(Frontier& fr, geometry_msgs::Point reference_robot_pose){
    Frontier fr1, fr2;
    assert(!fr.vectors_to_points.empty() > 0);
    size_t fr1_pts_number = fr.vectors_to_points.size() / 2;
    // TODO remove it, while for backward compatibility
        fr1.points = fr.points;
        fr2.points = fr.points;
//        fr1.min_distance
    fr1.vectors_to_points = std::vector<geometry_msgs::Point>(fr.vectors_to_points.begin(), fr.vectors_to_points.begin() + fr1_pts_number);
    fr1.interpolated_line = {fr.interpolated_line.first, fr.middle};
    fr1.centroid = fr1.vectors_to_points[fr1.vectors_to_points.size() / 2];
    fr1.centroid.x += reference_robot_pose.x;
    fr1.centroid.y += reference_robot_pose.y;


    fr2.vectors_to_points = std::vector<geometry_msgs::Point>( fr.vectors_to_points.begin() + fr1_pts_number,  fr.vectors_to_points.end());
    fr2.interpolated_line = {fr.middle, fr.interpolated_line.second};
    fr2.centroid = fr2.vectors_to_points[fr2.vectors_to_points.size() / 2];
        fr2.centroid.x += reference_robot_pose.x;
        fr2.centroid.y += reference_robot_pose.y;

    return {fr1, fr2};
}
std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        std::vector<Frontier> new_frontiers = buildNewFrontier(nbr, pos, frontier_flag);
        for (auto &new_frontier: new_frontiers){
            if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
                frontier_list.push_back(new_frontier);
            }
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}


double getDegreesDistance(geometry_msgs::Point p1,geometry_msgs::Point p2,geometry_msgs::Point reference){
    // using vector dotproduct to get angle between two reference points v_A dot v_B == |v_A| * |v_B| * cos(angle(A^B))
    double v1_x{p1.x - reference.x},
    v1_y{p1.y - reference.y},
    v2_x{p2.x - reference.x},
    v2_y{p2.y - reference.y},
    len_v1 = std::hypot(v1_x, v1_y),
    len_v2 = std::hypot(v2_x, v2_y);
    double dot_prod = v1_x * v2_x + v1_y * v2_y;
    double angle = std::acos(dot_prod / (len_v1 * len_v2)) * 180.0 / M_PI;
    return angle;
}


bool cmp_by_angle(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return atan2(p1.y, p1.x) < atan2(p2.y, p2.x);
}


std::vector<Frontier> FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                                       unsigned int reference,
                                                       std::vector<bool> &frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  ROS_ERROR_STREAM(ix << "   ---   "  << iy);

  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

    ROS_ERROR_STREAM(output.initial.x << "   ---   "  << output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);
  geometry_msgs::Point reference_robot_pose;
  reference_robot_pose.x = reference_x;
  reference_robot_pose.y = reference_y;

//  ROS_ERROR_STREAM( reference_robot_pose.x << "   " << reference_robot_pose.y);
// TODO make a parameter
size_t choose_each = 6;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();
    double world_scale_X =  costmap_->getSizeInMetersX();
    double world_scale_Y =  costmap_->getSizeInMetersY();
    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        // selecting sparce points
        if (output.points.size() % choose_each == 0){
            geometry_msgs::Point reference_scope_point;
            reference_scope_point.x = wx - reference_x;
            reference_scope_point.y = wy - reference_y;
            output.vectors_to_points.push_back(reference_scope_point);
        }

//        costmap_.

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++; // FIXME R U SRIOS!?

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = std::hypot(reference_x - wx, reference_y - wy);
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

//  output.min_distance = std::hypot(reference_x - output.middle.x, reference_y - output.middle.y);
  // average out frontier centroid

  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  /*KD*/

//  todo doit with lambda
  std::sort(output.vectors_to_points.begin(), output.vectors_to_points.end(), cmp_by_angle);

  // fixme find out why there are occuring empty frontiers (empty raw points)
//  ROS_WARN_STREAM(output.vectors_to_points.size() << "  VEC LENGTH");
//  if (output.vectors_to_points.size() > 0)
//      ROS_WARN_STREAM(output.vectors_to_points.front() << "  VEC LENGTH" << output.vectors_to_points.back());
//  else
//      ROS_WARN_STREAM(output.points.size() << "  FULL LENGTH ");
  output.interpolated_line = approximateFrontierViewAngle(output, reference_robot_pose);


// angular distance for vectors
// TODO move to approppriate frontier
if (!output.vectors_to_points.empty()){
    double degrees_distance =  std::abs( atan2(output.interpolated_line.first.y, output.interpolated_line.first.x) - atan2(output.interpolated_line.second.y, output.interpolated_line.second.x) )
            * 180.0 / M_PI;
    getDegreesDistance(output.interpolated_line.first, output.interpolated_line.second, reference_robot_pose);

    if (degrees_distance > 10.0){
        ROS_WARN_STREAM("Detected too wide frontier, " << degrees_distance << "splitting on two");

        auto splitted = splitFrontier(output, reference_robot_pose);
        for (auto &fr: splitted){
            degrees_distance =  std::abs( atan2(fr.interpolated_line.first.y, fr.interpolated_line.first.x) - atan2(fr.interpolated_line.second.y, fr.interpolated_line.second.x) )
                    * 180.0 / M_PI;

            ROS_WARN_STREAM("Ner frontier, " << degrees_distance);
        }
        return splitted;
    }
}

  ROS_ERROR_STREAM("distance in degrees = " << getDegreesDistance(output.interpolated_line.first, output.interpolated_line.second, reference_robot_pose));
  // next splitting onto pieces while they are more then 10deg from robots perspective


  return {output};
}



bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance
  // * costmap_->getResolution()
          )
          -
         (gain_scale_
         /*KD*/ * frontier.vectors_to_points.size()
         // * frontier.size
        // * costmap_->getResolution()
         );
}

}
