#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/costmap_2d.h>
#include "explore/Frontier.h"
namespace frontier_exploration
{


/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch() = default;

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size,
                 int use_every_k_point,double max_frontier_angular_size);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::Point position);

  /**
   * @brief calculates if frontier should be considered hidden
   * @param fr considered frontier itself
   * @param distance_thresh if distance to frontier less then thresh -- to consider it hidden
   */
  bool is_hidden(frontier_exploration::Frontier &fr, double distance_thresh);


protected:
//    std::pair<geometry_msgs::Point, geometry_msgs::Point> approxFrontierByPlanarFarthest(Frontier &fr,
//                                                                                         geometry_msgs::Point &reference_robot);
//    std::pair<geometry_msgs::Point, geometry_msgs::Point> approximateFrontierByViewAngle(Frontier &fr);
//    std::vector<Frontier> splitFrontier(Frontier& fr);


  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  std::vector<Frontier> buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                         std::vector<bool> &frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

private:
  costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
  int use_every_k_point_{1};
  double max_frontier_angular_size_{10.0}; // TODO make a parameter on parameter server
};
}
#endif
