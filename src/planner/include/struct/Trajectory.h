//
// Created by luyifan on 4/21/19.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include "WayPoints.h"
#include "common/common.h"

class Trajectory{
public:
    Trajectory();
    Trajectory(vector<double> x, vector<double> y);
    ~Trajectory();

    void computeKappa();

    vector<double> xs();
    vector<double> ys();

    vector<WayPoint> points;
    double cost;
};


#endif //PATH_PLANNING_TRAJECTORY_H
