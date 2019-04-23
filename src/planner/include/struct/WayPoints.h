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

    void setX(double x);
    void setY(double y);
    void setS(double s);
    void setD(double d);
    void setKappa(double kappa);

private:
    double x_;
    double y_;
    double s_;
    double d_;
    double kappa_;
};

#endif //PATH_PLANNING_WAYPOINTS_H
