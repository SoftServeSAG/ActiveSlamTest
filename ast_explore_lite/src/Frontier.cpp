//
// Created by user on 3/22/19.
//

#include "explore/Frontier.h"
#include "explore/explore_utils.h"

namespace frontier_exploration{

    Frontier::Frontier(FrontierParams &params):
        cost(params.cost),
        reference_robot_pose(params.reference_robot_pose),
        hidden_dist_threshold(params.hidden_distance_threshold) // todo it should not be in frontier object
        {

        // moving saving only some part of detected frontier points
        for (int i = 0; i < params.vectors_to_points.size(); ++i){
        if (i % params.sparsify_k_times == 0)
            vectors_to_points.push_back(params.vectors_to_points[i]);
        }

        if (!params.presorted){
            // sorting by view angle from the robot's reference pose
            auto compareViewAngle = [](const geometry_msgs::Point &p1,const  geometry_msgs::Point &p2)
            {return atan2(p1.y, p1.x) < atan2(p2.y, p2.x);};
            std::sort(vectors_to_points.begin(), vectors_to_points.end(), compareViewAngle);
        }

//        if(!vectors_to_points.empty()) <-- beware this bug is still around, but checked outside
            middle = *(vectors_to_points.begin() + vectors_to_points.size() / 2);
            middle = fromReferenceFrame(middle);
    interpolated_line = Frontier::approximateFrontierByViewAngle(*this);
    std::vector<geometry_msgs::Point> closest_points_candidates = {this->middle, this->interpolated_line.first, this->interpolated_line.second};
    geometry_msgs::Point closest_point  = getClosestPointTo(  closest_points_candidates,  reference_robot_pose);
    min_distance = std::hypot(closest_point.x - reference_robot_pose.x , closest_point.y - reference_robot_pose.y);
    hidden = min_distance  < params.hidden_distance_threshold;
    }



std::pair<geometry_msgs::Point, geometry_msgs::Point> Frontier::approxFrontierByPlanarFarthest(Frontier &fr){
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
                std::hypot( (/*transforming to map frame*/point.x + fr.reference_robot_pose.x) - fr.middle.x , (point.y + fr.reference_robot_pose.y) - fr.middle.y); /* centroid_point dist */
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

    std::vector<Frontier> Frontier::splitFrontier(Frontier& fr, double max_angular_size){
        std::vector<Frontier> vector_buf, res;
        size_t split_size {fr.vectors_to_points.size() / 2};
        FrontierParams fr_par(fr); // saving all parameters from initial frontier
        // defining subfrontiers points by itarator onto original points vector
        using subfrontier_points = std::pair<std::vector<geometry_msgs::Point>::iterator,std::vector<geometry_msgs::Point>::iterator>;
        subfrontier_points first_points = {fr.vectors_to_points.begin(), fr.vectors_to_points.begin() + split_size};
        subfrontier_points second_points = {fr.vectors_to_points.begin() + split_size, fr.vectors_to_points.end()};
        auto splits = {first_points, second_points};
        for (auto& split: splits){
            Frontier this_fr = Frontier(fr_par);
            this_fr.vectors_to_points = std::vector<geometry_msgs::Point>(split.first, split.second);

            double fr_angular_dist = angular_vector_distance(this_fr.interpolated_line.first, this_fr.interpolated_line.second);
            ROS_DEBUG_STREAM("FR has [" << fr_angular_dist << "]  deg  [" << this_fr.vectors_to_points.size() << "] PTS");
            if (fr_angular_dist > max_angular_size
                && this_fr.vectors_to_points.size() > 6)  // to allways have two subdrontiers with middlepoints
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

    FrontierParams::FrontierParams(const frontier_exploration::Frontier& fr):
            cost{fr.cost},
            reference_robot_pose{fr.reference_robot_pose},
            vectors_to_points{fr.vectors_to_points},
            min_frontier_size{3},
            sparsify_k_times{1},
            max_angular_size{10.0},
            hidden_distance_threshold(fr.hidden_dist_threshold){
    }
}
