//
// Created by luyifan on 4/23/19.
//

#ifndef PATH_PLANNING_EGO_H
#define PATH_PLANNING_EGO_H
#include "common/common.h"

class Ego{
public:
    Ego(double x, double y, double yaw, double v, double s, double d);
    ~Ego();

    Eigen::Vector3d poUTM();
    double velocity();
    Eigen::Vector2d poFrenet();
    void updateState(double x, double y, double yaw, double v, double s, double d);
    inline Eigen::Vector2d shape(){return Eigen::Vector2d(length_, width_);}

private:
    double x_;
    double y_;
    double yaw_;
    double v_;
    double s_;
    double d_;
    double acc_;

    double length_;
    double width_;
};

#endif //PATH_PLANNING_EGO_H
