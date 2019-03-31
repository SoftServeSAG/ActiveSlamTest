
#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <boost/geometry.hpp>
#include <geometry_msgs/Vector3.h>
#include <cmath>

#include "explore/costmap_tools.h"
#include "explore/FrontierSearch.h"
#include "explore/explore_utils.h"
#include "explore/Frontier.h"

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;


FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size,
                               int use_every_k_point,
                               double max_frontier_angular_size,
                               double hidden_distance_threshold)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size) // TODO provide direct reading from parameter server in constructor
  , use_every_k_point_(use_every_k_point)
  , max_frontier_angular_size_(max_frontier_angular_size)
  , hidden_distance_threshold_(hidden_distance_threshold)
{
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
            if (new_frontier.vectors_to_points.size() * costmap_->getResolution() >= min_frontier_size_) {
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
  // TODO  do we need sort if only min is interesting? maybe once we have cache behaviour prtial sort will be ok...

  return frontier_list;
}


std::vector<Frontier> FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                                       unsigned int reference,
                                                       std::vector<bool> &frontier_flag)
{
  // initialize frontier structure
  Frontier output;

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  //costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  FrontierParams fr_par;
  fr_par.reference_robot_pose = makePointMsg(reference_x, reference_y);
  fr_par.sparsify_k_times = this->use_every_k_point_; // TODO pass those parameters during parameter reading
  fr_par.max_angular_size = this->max_frontier_angular_size_;
  fr_par.min_frontier_size = this->min_frontier_size_;
  fr_par.hidden_distance_threshold = this->hidden_distance_threshold_;
  fr_par.needSparsify = true;

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

        fr_par.vectors_to_points.push_back(makePointMsg(
                wx - reference_x,
                wy - reference_y));

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

    std::vector<Frontier> splitted_frontiers;
    // fixme find out why there are occuring empty frontiers (empty raw points)
if (!fr_par.vectors_to_points.empty()){
    splitted_frontiers = {Frontier(fr_par)};
    output = Frontier(fr_par);
    double degrees_distance = angular_vector_distance(output.interpolated_line.first, output.interpolated_line.second, output.reference_robot_pose);
    if (degrees_distance > this->max_frontier_angular_size_){
        ROS_INFO_STREAM("Detected wide frontier [" << degrees_distance <<"]" << "PTS [" <<output.vectors_to_points.size()<< "]  SPLITTING...");
        splitted_frontiers = Frontier::splitFrontier(output, max_frontier_angular_size_);
    }
}
  return splitted_frontiers;
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

// todo Detect other room case


double FrontierSearch::frontierCost(const Frontier& frontier)
{
    // TODO add rotation angle heuristics
    // TODO geodesial distance heuristics
    // todo add wall reachability for successfull mapping
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
