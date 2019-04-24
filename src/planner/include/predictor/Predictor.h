//
// Created by luyifan on 19-4-24.
//

#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include "common/common.h"
#include "struct/Map.h"

class Predictor{
public:
    Predictor(Map* map);
    ~Predictor();

    vector<vector<double> > predict(vector<vector<double> >& vehicles);
private:
    Map* map_;
};


#endif //PATH_PLANNING_PREDICTION_H
