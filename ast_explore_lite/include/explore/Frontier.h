//
// Created by user on 3/22/19.
//

#ifndef PROJECT_FRONTIER_H
#define PROJECT_FRONTIER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>


namespace frontier_exploration {
/**
 * @brief Represents a frontier
 */
    struct FrontierParams{
        double cost{std::numeric_limits<double>::infinity()};
        bool is_hidden{false};
        geometry_msgs::Point reference_robot_pose;
        geometry_msgs::Point initial;
        geometry_msgs::Point middle;
        std::pair<geometry_msgs::Point,geometry_msgs::Point> interpolated_line;
        std::vector<geometry_msgs::Point> vectors_to_points;
        double min_frontier_size{3};
        int left_each{1};
        double max_angular_size{10.0};
    };

    struct Frontier {
        //todo improve this struct functionality
        double min_distance{std::numeric_limits<double>::infinity()};
        double cost{0.0};
        geometry_msgs::Point initial;
        geometry_msgs::Point middle;
        std::pair<geometry_msgs::Point,geometry_msgs::Point> interpolated_line;
        std::vector<geometry_msgs::Point> vectors_to_points;
        geometry_msgs::Point reference_robot_pose;

        bool hidden{false};

        /**
         * @brief default constructor (in case we need to initialize container of values, but can't pass arguments for
         * building each)
         */
        Frontier() = default;

        Frontier(FrontierParams &params):
        cost(params.cost),
        reference_robot_pose(params.reference_robot_pose),
        hidden(params.is_hidden),
        initial(params.initial),
        middle(params.middle),
        interpolated_line(params.interpolated_line),
        vectors_to_points(params.vectors_to_points)
        {

        }

    public:
        // service functions
        geometry_msgs::Point toReferenceFrame(const geometry_msgs::Point &pt);
        geometry_msgs::Point fromReferenceFrame(const geometry_msgs::Point &pt_in_reference_frame);
        static std::pair<geometry_msgs::Point, geometry_msgs::Point> approximateFrontierByViewAngle(
                frontier_exploration::Frontier &fr);
        static std::pair<geometry_msgs::Point, geometry_msgs::Point> approxFrontierByPlanarFarthest(Frontier &fr,
                                                                                             geometry_msgs::Point &reference_robot);

        static std::vector<Frontier> splitFrontier(Frontier& fr, double max_angular_size);

    };


}


#endif //PROJECT_FRONTIER_H
