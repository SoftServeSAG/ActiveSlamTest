//
// Created by user on 3/19/19.
//

#ifndef PROJECT_EXPLORE_UTILS_H
#define PROJECT_EXPLORE_UTILS_H

#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration{
    static geometry_msgs::Point pointMsgBuilder(const double x, const double y, const double z){
        geometry_msgs::Point p;
        p.x = x; p.y = y; p.z = z;
        return p;
    }
}

#endif //PROJECT_EXPLORE_UTILS_H
