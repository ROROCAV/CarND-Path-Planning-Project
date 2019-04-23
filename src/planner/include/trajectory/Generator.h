//
// Created by luyifan on 4/21/19.
//

#ifndef PATH_PLANNING_GENERATOR_H
#define PATH_PLANNING_GENERATOR_H

#include "common/helpers.h"
#include "common/common.h"
#include "struct/Ego.h"
#include "struct/Trajectory.h"

class Generator{
public:
    Generator();
    ~Generator();

    void generate(const vector<double>& previous_path_x, const vector<double>& previous_path_y, double car_x, double car_y,
                  double car_yaw, const vector<vector<double> >& vehicles, vector<double>& next_x_vals, vector<double>& next_y_vals);

    void laneKeeping(Trajectory pre_traj_utm, Ego ego, const vector<vector<double> >& vehicles, Trajectory next_traj_utm);

private:

};

#endif //PATH_PLANNING_GENERATOR_H
