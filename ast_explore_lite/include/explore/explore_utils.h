//
// Created by user on 3/19/19.
//

#ifndef PROJECT_EXPLORE_UTILS_H
#define PROJECT_EXPLORE_UTILS_H

#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration{
    static geometry_msgs::Point makePointMsg(const double x = 0, const double y = 0, const double z = 0){
        geometry_msgs::Point p;
        p.x = x; p.y = y; p.z = z;
        return p;
    }

    inline double angular_vector_distance(const double x1, const double y1,const double x2,const double y2){
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
    inline geometry_msgs::Point getClosestPointTo(const std::vector<geometry_msgs::Point> &distance_vectors,
                                                  const geometry_msgs::Point &goal){
        auto min_elem_iter = std::min_element(distance_vectors.begin(), distance_vectors.end(),
                                              [](const geometry_msgs::Point &p1,const geometry_msgs::Point &p2)
                                              {return p1.x + p1.y < p2.x + p2.y;}); //using Manhatten distance
        return *min_elem_iter;
    }
}

#endif //PROJECT_EXPLORE_UTILS_H
