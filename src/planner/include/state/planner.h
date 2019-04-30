//
// Created by luyifan on 19-4-30.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include "common/common.h"
#include "state_common.h"
#include "struct/Ego.h"
#include "struct/Vehicle.h"
#include "struct/Trajectory.h"
#include "struct/Map.h"
#include "trajectory/Generator.h"

class StStop;
//状态机第一个状态是stop状态
class Planner : public sc::state_machine<Planner, StStop, boost::pool_allocator < char >, sc::exception_translator<> >{
private:
    Planner(Ego* ego, Map* map);
    //把复制构造函数和=操作符也设为私有,防止被复制
    Planner(const Planner &);
    Planner &operator=(const Planner &);

public:
    static Planner* instance(Ego* ego, Map* map){
        static Planner instance(ego, map);
        return &instance;
    };

public:
    Ego* ego;
    Map* map;
    vector<Vehicle>* predictions;
    Trajectory* pre_path;
    Trajectory next_path;
    Generator* generator;

public:
    void inputEnv(Trajectory* pre_path, vector<Vehicle>* predictions);
    void outputTrajectory(vector<double>& xs, vector<double>& ys);
};

#endif //PATH_PLANNING_PLANNER_H
