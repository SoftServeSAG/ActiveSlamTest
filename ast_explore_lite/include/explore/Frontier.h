//
// Created by user on 3/22/19.
//

#ifndef PROJECT_FRONTIER_H
#define PROJECT_FRONTIER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>


namespace frontier_exploration {
    struct Frontier;
/**
 * @brief Represents a frontier, all required info for building one and flags with instructions for builder
 * @remark global parameters need to be reallocated from here
 */
    struct FrontierParams{
        double cost{std::numeric_limits<double>::infinity()};
        geometry_msgs::Point reference_robot_pose;
        std::vector<geometry_msgs::Point> vectors_to_points;
        double min_frontier_size{3};
        int sparsify_k_times{1};
        double max_angular_size{10.0};
        double hidden_distance_threshold{0};// all frontiers are of same status if not defined else
        bool presorted = false;
        bool needSparsify = false;
        FrontierParams(const Frontier& fr);
        FrontierParams() = default;
    };

    struct Frontier {
        //todo improve this struct functionality
        double min_distance {std::numeric_limits<double>::infinity()}; //todo drop it, now unused but required for explore algo
        double cost{0.0};
        geometry_msgs::Point middle;
        std::pair<geometry_msgs::Point,geometry_msgs::Point> interpolated_line;
        std::vector<geometry_msgs::Point> vectors_to_points;
        geometry_msgs::Point reference_robot_pose;
        bool hidden{false};
        double hidden_dist_threshold {0}; // todo move it from structure

        /**
         * @brief default constructor (in case we need to initialize container of values, but can't pass arguments for
         * building each)
         */
        Frontier() = default;

        explicit Frontier(FrontierParams &params);

    public:
        // service functions
        /**
         * @param pt point coordinates in map frame
         * @return point in robot coordinates
         */
        geometry_msgs::Point toReferenceFrame(const geometry_msgs::Point &pt);
        /**
         * @param pt_in_reference_frame
         * @return point in map frame coordinates
         */
        geometry_msgs::Point fromReferenceFrame(const geometry_msgs::Point &pt_in_reference_frame);
        /**
         * @brief uses view angle from robot current position to approximate frontier pointcloud with a line
         * @param fr frontier to be approximated
         * @return endpoint of a line approximated by this method
         */
        static std::pair<geometry_msgs::Point, geometry_msgs::Point> approximateFrontierByViewAngle(
                frontier_exploration::Frontier &fr);
        /**
         * @brief uses planar distance between points to  approximate frontier pointcloud with a line
         * idea is that approximated point shold lie on some convex figure (therefore lie farther from each-other)
         * @param fr frontier to be approximated
         * @return endpoint of a line approximated by this method
         */
        static std::pair<geometry_msgs::Point, geometry_msgs::Point> approxFrontierByPlanarFarthest(
                frontier_exploration::Frontier &fr);

        /**
         * @brief splits frontier on subfrontiers recursively
         * @param fr
         * @param max_angular_size comparison currently performed only by view angle
         * @return vector of splitted frontiers
         */
        static std::vector<Frontier> splitFrontier(Frontier& fr, double max_angular_size);

    };


}


#endif //PROJECT_FRONTIER_H
