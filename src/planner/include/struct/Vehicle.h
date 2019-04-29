//
// Created by luyifan on 4/26/19.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "common/common.h"

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

    vector<vector<double> > prediction;//first是age,表示n个时刻之前的数据
    vector<pair<int, vector<double> > > history;
};

#endif //PATH_PLANNING_VEHICLE_H
