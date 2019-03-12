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

    std::pair<geometry_msgs::Point, geometry_msgs::Point> FrontierSearch::getFrontierEdges(Frontier &fr, geometry_msgs::Point &reference_robot){
//      ROS_ERROR_STREAM("GOT REFERENCE POINT ["  <<  reference_robot.x << ":"<< reference_robot.y  << "]");
//      ROS_ERROR_STREAM("GOT CENTROID POINT ["  <<  fr.centroid.x << ":"<< fr.centroid.y  << "]");
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
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
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


Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
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
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

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
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

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

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = std::hypot(reference_x - wx, reference_x - wx);
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

  // average out frontier centroid

  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  /*KD*/
  output.interpolated_line = getFrontierEdges(output, reference_robot_pose);
  ROS_ERROR_STREAM("distance in degrees = " << getDegreesDistance(output.interpolated_line.first, output.interpolated_line.second, reference_robot_pose));
  // next splitting onto pieces while they are more then 10deg from robots perspective

  return output;
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
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}

}
