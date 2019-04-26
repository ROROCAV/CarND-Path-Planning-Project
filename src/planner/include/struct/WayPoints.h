//
// Created by luyifan on 4/21/19.
//

#ifndef PATH_PLANNING_WAYPOINTS_H
#define PATH_PLANNING_WAYPOINTS_H

#include "common/common.h"

class WayPoint{
public:
    WayPoint(double x, double y);
    ~WayPoint();

    double x;
    double y;
    double s;
    double d;
    double kappa;
};

#endif //PATH_PLANNING_WAYPOINTS_H
