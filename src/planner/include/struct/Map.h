//
// Created by luyifan on 19-4-24.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include "common/common.h"

class Map{
public:
    Map(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy);
    ~Map();

    inline vector<double> x(){ return x_;}
    inline vector<double> y(){ return y_;}
    inline vector<double> s(){ return s_;}
    inline vector<double> dx(){ return dx_;}
    inline vector<double> dy(){ return dy_;}

private:
    vector<double> x_;
    vector<double> y_;
    vector<double> s_;
    vector<double> dx_;
    vector<double> dy_;
};

#endif //PATH_PLANNING_MAP_H
