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

    inline int length(){return val_.size();}
    void insertWayPoint(WayPoint pt);
    void setCost(double cost);
    void computeKappa();
    inline vector<WayPoint> val(){ return val_; }

private:
    vector<WayPoint> val_;
    double cost_;
};


#endif //PATH_PLANNING_TRAJECTORY_H
