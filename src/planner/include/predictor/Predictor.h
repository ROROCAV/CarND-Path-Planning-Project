//
// Created by luyifan on 19-4-24.
//

#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include "common/common.h"
#include "struct/Map.h"
#include "struct/Vehicle.h"

class Predictor{
public:
    Predictor(Map* map, int life);
    ~Predictor();

    void update(vector<vector<double> > sensor_fusion);

    vector<vector<double> > predict();
private:
    Map* map_;
    vector<vector<Vehicle> > vehicles_;
    int life_;
};


#endif //PATH_PLANNING_PREDICTION_H
