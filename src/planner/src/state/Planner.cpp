//
// Created by luyifan on 19-4-30.
//

#include "state/StStop.h"
#include "state/planner.h"

Planner::Planner(Ego* ego, Map* map):ego(ego), map(map){
    generator = new Generator(map);
}

void Planner::inputEnv(Trajectory *pre_path, vector<Vehicle> *predictions) {
    this->pre_path = pre_path;
    this->predictions = predictions;
}

void Planner::outputTrajectory(vector<double> &xs, vector<double> &ys) {
    xs.clear();
    ys.clear();
    for(auto pt:next_path.points){
        xs.emplace_back(pt.x);
        ys.emplace_back(pt.y);
    }
}