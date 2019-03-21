
#include <mutex>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>

#include <boost/geometry.hpp>
#include <geometry_msgs/Vector3.h>
#include <cmath>

#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>
#include <explore/explore_utils.h>


namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;


inline double angular_vector_distance(const double x1,const double y1,const double x2,const double y2){
    return std::abs( atan2(y1, x1) - atan2(y2, x2) ) * 180.0 / M_PI;
}
inline double angular_vector_distance(const geometry_msgs::Point &v1,const  geometry_msgs::Point &v2) {
    return angular_vector_distance(v1.x, v1.y, v2.x, v2.y);
}
// for the case when vectors are presented by points with single point-of-origin
inline double angular_vector_distance(const geometry_msgs::Point &p1,const  geometry_msgs::Point &p2,const  geometry_msgs::Point &reference_pt){
    return angular_vector_distance(p1.x - reference_pt.x, p1.y - reference_pt.y, p2.x - reference_pt.x, p2.y - reference_pt.y);
}

// for now is not used
inline geometry_msgs::Point getClosestPointTo(const std::vector<geometry_msgs::Point> &distance_vectors, const geometry_msgs::Point &goal){
    auto min_elem_iter = std::min_element(distance_vectors.begin(), distance_vectors.end(),
            [](const geometry_msgs::Point &p1,const geometry_msgs::Point &p2)
            {return p1.x + p1.y < p2.x + p2.y;}); //using Manhatten distance
    return *min_elem_iter;
}

inline geometry_msgs::Point Frontier::toReferenceFrame(const geometry_msgs::Point &pt_in_absolute_frame){
    return makePointMsg(
            pt_in_absolute_frame.x - reference_robot_pose.x,
            pt_in_absolute_frame.y - reference_robot_pose.y,
            0);
}
inline geometry_msgs::Point  Frontier::fromReferenceFrame(const geometry_msgs::Point &pt_in_reference_frame){
    return makePointMsg(pt_in_reference_frame.x + reference_robot_pose.x,
                        pt_in_reference_frame.y + reference_robot_pose.y,
                        0);
}

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size,
                               int use_every_k_point,
                               double max_frontier_angular_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , use_every_k_point_(use_every_k_point)
  , max_frontier_angular_size_(max_frontier_angular_size)
{
}
    std::pair<geometry_msgs::Point, geometry_msgs::Point> FrontierSearch::approxFrontierByPlanarFarthest(Frontier &fr,
                                                                                                         geometry_msgs::Point &reference_robot){
        double max_dist = 0.0;
        double heuristics_dist {0.0};
        geometry_msgs::Point p1, p2;
      for (auto &point: fr.vectors_to_points){
          heuristics_dist =
                  std::hypot(point.x , point.y ) +  /* robot_point_dist */
                  std::hypot(point.x , point.y ); /* centroid_point dist */
          if (heuristics_dist > max_dist){
              max_dist = heuristics_dist;
              p1 = point;
          }
      }
      max_dist = 0.0;
       for (auto &point: fr.vectors_to_points){
           heuristics_dist =
                   std::hypot(point.x - p1.x , point.y - p1.y) +  /*  point-point dist */
                   std::hypot( (/*transforming to map frame*/point.x + reference_robot.x) - fr.centroid.x , (point.y + reference_robot.y) - fr.centroid.y); /* centroid_point dist */
            if (heuristics_dist > max_dist){
                max_dist = heuristics_dist;
                p2 = point;
            }
        }
        return {p1, p2};
}


    std::pair<geometry_msgs::Point, geometry_msgs::Point> FrontierSearch::approximateFrontierByViewAngle(
            frontier_exploration::Frontier &fr) {
        geometry_msgs::Point p1, p2;
        if (!fr.vectors_to_points.empty()){
            p1 = fr.fromReferenceFrame(fr.vectors_to_points.front());
            p2 = fr.fromReferenceFrame(fr.vectors_to_points.back());
            ROS_DEBUG_STREAM( " approximated borders with view angle " <<  fr.vectors_to_points.front() << fr.vectors_to_points.back());
        } // else default zero values
        return {p1, p2};
    }

    std::vector<Frontier> FrontierSearch::splitFrontier(Frontier& fr){
        Frontier fr1, fr2;
        std::vector<Frontier> frontiers{fr1, fr2}, vector_buf, res;
        size_t splits = frontiers.size();
        size_t current_element {0}, split_size {fr.vectors_to_points.size() / splits};
        for (auto& this_fr: frontiers){
            this_fr.reference_robot_pose = fr.reference_robot_pose;
            this_fr.vectors_to_points = std::vector<geometry_msgs::Point>(
                    fr.vectors_to_points.begin() + current_element,
                    fr.vectors_to_points.begin() + current_element + split_size);

            current_element += split_size;
            this_fr.interpolated_line = {this_fr.vectors_to_points.front(), this_fr.vectors_to_points.back()};
            this_fr.centroid = *(this_fr.vectors_to_points.begin() + split_size / 2);  // not true centroid, but who cares... //todo maybe

            this_fr.centroid = this_fr.fromReferenceFrame(this_fr.centroid);
            this_fr.interpolated_line.first = this_fr.fromReferenceFrame(this_fr.interpolated_line.first);
            this_fr.interpolated_line.second = this_fr.fromReferenceFrame(this_fr.interpolated_line.second);

            if (fr.vectors_to_points.end() - fr.vectors_to_points.begin() - current_element < split_size) {
                this_fr.vectors_to_points.insert(this_fr.vectors_to_points.end(),
                                                 fr.vectors_to_points.begin() + current_element,
                                                 fr.vectors_to_points.end()); // uppend last chunk in case of division leftover
            }

            double fr_angular_dist = angular_vector_distance(this_fr.interpolated_line.first, this_fr.interpolated_line.second);
            ROS_DEBUG_STREAM("FR has [" << fr_angular_dist << "]  deg  [" << frontiers[0].vectors_to_points.size() << "] PTS");
            if (fr_angular_dist > this->max_frontier_angular_size_
                && fr1.vectors_to_points.size() > 6)  // to allways have two subdrontiers with middlepoints
            {
                ROS_DEBUG_STREAM("PERFORM FRONTIER RECURSIVE SPLITTING");
                vector_buf = splitFrontier(this_fr);
            } else{
                vector_buf = {this_fr};
            }
            res.insert(res.end(), vector_buf.begin(), vector_buf.end());
        }
        return res;
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
  output.centroid.x = 0;
  output.centroid.y = 0;

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
  output.reference_robot_pose.x = reference_x;
  output.reference_robot_pose.y = reference_y;

size_t  cntr{0};

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

        // leaving only each n-th point
        if (cntr++ % this->use_every_k_point_ == 0){
            geometry_msgs::Point reference_scope_point;
            reference_scope_point.x = wx - reference_x;
            reference_scope_point.y = wy - reference_y;
            output.vectors_to_points.push_back(reference_scope_point);
            output.centroid.x += wx; // map frame coords
            output.centroid.y += wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }
  // average out frontier centroid
  output.centroid.x /= output.vectors_to_points.size();
  output.centroid.y /= output.vectors_to_points.size();
  /*KD*/

  std::sort(output.vectors_to_points.begin(), output.vectors_to_points.end(),
          [](const geometry_msgs::Point &p1,const  geometry_msgs::Point &p2)
          {return atan2(p1.y, p1.x) < atan2(p2.y, p2.x);}
          );

  // fixme find out why there are occuring empty frontiers (empty raw points)
  output.interpolated_line = approximateFrontierByViewAngle(output);
  std::vector<Frontier> splitted_frontiers{output};
if (!output.vectors_to_points.empty()){

    double degrees_distance = angular_vector_distance(output.interpolated_line.first, output.interpolated_line.second, output.reference_robot_pose);
    if (degrees_distance > this->max_frontier_angular_size_){
        ROS_INFO_STREAM("Detected wide frontier [" << degrees_distance <<"]" << "PTS [" <<output.vectors_to_points.size()<< "]  SPLITTING...");
        splitted_frontiers = splitFrontier(output);
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
bool FrontierSearch::is_hidden(frontier_exploration::Frontier &fr, double thresh_distance){
    return std::hypot(fr.centroid.x - fr.reference_robot_pose.x, fr.centroid.y - fr.reference_robot_pose.y) < thresh_distance;
}

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
