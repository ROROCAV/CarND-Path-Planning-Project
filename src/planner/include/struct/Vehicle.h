//
// Created by luyifan on 4/26/19.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "common/common.h"

enum state{ID=0, X=1, Y=2, VX=3, VY=4, S=5, D=6};

class Vehicle{
public:
    Vehicle();
    Vehicle(int id, double x, double y, double s, double d, double v_x, double v_y);
    ~Vehicle();

    int id;
    double x;
    double y;
    double s;
    double d;
    double v_x;
    double v_y;
    int age;
};

#endif //PATH_PLANNING_VEHICLE_H
