//
// Created by user on 3/22/19.
//

#include "explore/Frontier.h"
#include "explore/explore_utils.h"

namespace frontier_exploration{

    Frontier::Frontier(FrontierParams &params):
        cost(params.cost),
        reference_robot_pose(params.reference_robot_pose),
        initial(params.initial),
        middle(params.middle)
    {
        for (int i = 0; i < params.vectors_to_points.size(); ++i){
        if (i % params.sparsify_k_times == 0)
            vectors_to_points.push_back(params.vectors_to_points[i]);
    }

    // sorting by wiev angle from the robot's reference pose
    std::sort(vectors_to_points.begin(), vectors_to_points.end(),
    [](const geometry_msgs::Point &p1,const  geometry_msgs::Point &p2)
    {return atan2(p1.y, p1.x) < atan2(p2.y, p2.x);}
    );
    interpolated_line = Frontier::approximateFrontierByViewAngle(*this);
    hidden = is_hidden(*this,  params.hidden_distance_threshold);
    }



std::pair<geometry_msgs::Point, geometry_msgs::Point> Frontier::approxFrontierByPlanarFarthest(Frontier &fr,
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
                std::hypot( (/*transforming to map frame*/point.x + reference_robot.x) - fr.middle.x , (point.y + reference_robot.y) - fr.middle.y); /* centroid_point dist */
        if (heuristics_dist > max_dist){
            max_dist = heuristics_dist;
            p2 = point;
        }
    }
    return {p1, p2};
}


std::pair<geometry_msgs::Point, geometry_msgs::Point> Frontier::approximateFrontierByViewAngle(
        frontier_exploration::Frontier &fr) {
    geometry_msgs::Point p1, p2;
    if (!fr.vectors_to_points.empty()){
        p1 = fr.fromReferenceFrame(fr.vectors_to_points.front());
        p2 = fr.fromReferenceFrame(fr.vectors_to_points.back());
        ROS_DEBUG_STREAM( " approximated borders with view angle " <<  fr.vectors_to_points.front() << fr.vectors_to_points.back());
    } // else default zero values
    return {p1, p2};
}

    inline geometry_msgs::Point Frontier::toReferenceFrame(const geometry_msgs::Point &pt_in_absolute_frame){
        return makePointMsg(
                pt_in_absolute_frame.x - reference_robot_pose.x,
                pt_in_absolute_frame.y - reference_robot_pose.y,
                0);
    }

    inline  geometry_msgs::Point  Frontier::fromReferenceFrame(const geometry_msgs::Point &pt_in_reference_frame){
        return makePointMsg(pt_in_reference_frame.x + reference_robot_pose.x,
                            pt_in_reference_frame.y + reference_robot_pose.y,
                            0);
    }
    bool Frontier::is_hidden(frontier_exploration::Frontier &fr, double thresh_distance){
        return std::hypot(fr.middle.x - fr.reference_robot_pose.x, fr.middle.y - fr.reference_robot_pose.y) < thresh_distance;
    }

    std::vector<Frontier> Frontier::splitFrontier(Frontier& fr, double max_angular_size){
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
            this_fr.middle = *(this_fr.vectors_to_points.begin() + split_size / 2);  // not true centroid, but who cares... //todo maybe

            this_fr.middle = this_fr.fromReferenceFrame(this_fr.middle);
            this_fr.interpolated_line.first = this_fr.fromReferenceFrame(this_fr.interpolated_line.first);
            this_fr.interpolated_line.second = this_fr.fromReferenceFrame(this_fr.interpolated_line.second);

            if (fr.vectors_to_points.end() - fr.vectors_to_points.begin() - current_element < split_size) {
                this_fr.vectors_to_points.insert(this_fr.vectors_to_points.end(),
                                                 fr.vectors_to_points.begin() + current_element,
                                                 fr.vectors_to_points.end()); // uppend last chunk in case of division leftover
            }

            double fr_angular_dist = angular_vector_distance(this_fr.interpolated_line.first, this_fr.interpolated_line.second);
            ROS_DEBUG_STREAM("FR has [" << fr_angular_dist << "]  deg  [" << frontiers[0].vectors_to_points.size() << "] PTS");
            if (fr_angular_dist > max_angular_size
                && fr1.vectors_to_points.size() > 6)  // to allways have two subdrontiers with middlepoints
            {
                ROS_DEBUG_STREAM("PERFORM FRONTIER RECURSIVE SPLITTING");
                vector_buf = splitFrontier(this_fr, max_angular_size);
            } else{
                vector_buf = {this_fr};
            }
            res.insert(res.end(), vector_buf.begin(), vector_buf.end());
        }
        return res;
    }


}