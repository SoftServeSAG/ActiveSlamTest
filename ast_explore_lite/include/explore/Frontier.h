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
        geometry_msgs::Point reference_robot_pose;
        std::vector<geometry_msgs::Point> vectors_to_points;
        double min_frontier_size{3};
        int sparsify_k_times{1};
        double max_angular_size{10.0};
        double hidden_distance_threshold;
        bool presorted = false;
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

        explicit Frontier(FrontierParams &params);

    public:
        // service functions
        geometry_msgs::Point toReferenceFrame(const geometry_msgs::Point &pt);
        geometry_msgs::Point fromReferenceFrame(const geometry_msgs::Point &pt_in_reference_frame);
        static std::pair<geometry_msgs::Point, geometry_msgs::Point> approximateFrontierByViewAngle(
                frontier_exploration::Frontier &fr);
        // unused
        static std::pair<geometry_msgs::Point, geometry_msgs::Point> approxFrontierByPlanarFarthest(
                frontier_exploration::Frontier &fr);

        static std::vector<Frontier> splitFrontier(Frontier& fr, double max_angular_size);
        static bool is_hidden(frontier_exploration::Frontier &fr, double thresh_distance);

    };


}


#endif //PROJECT_FRONTIER_H
