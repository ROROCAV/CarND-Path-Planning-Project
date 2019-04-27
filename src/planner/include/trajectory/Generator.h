//
// Created by luyifan on 4/21/19.
//

#ifndef PATH_PLANNING_GENERATOR_H
#define PATH_PLANNING_GENERATOR_H

#include "common/helpers.h"
#include "common/common.h"
#include "struct/Ego.h"
#include "struct/Trajectory.h"
#include "struct/Map.h"
#include "common/spline.h"
#include "common/Collision.h"

class Generator{
public:
    Generator(Map* map);
    ~Generator();

    void example(Trajectory pre_traj_utm, Ego* ego, const vector<vector<double> >& vehicles,
                 vector<double>& next_x, vector<double>& next_y);

    void laneKeeping(Trajectory pre_traj_utm, Ego* ego, const vector<vector<vector<double> > >& predictions,
                     vector<double>& next_x, vector<double>& next_y);

    Trajectory inference(Trajectory pre_traj_utm, Ego* ego, int lane, double ref_vel);

private:
    Map* map_;
    double ref_vel_;
    int ref_lane_;
};

#endif //PATH_PLANNING_GENERATOR_H
