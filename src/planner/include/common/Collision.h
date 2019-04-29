//
// Created by luyifan on 4/27/19.
//

#ifndef PATH_PLANNING_COLLISION_H
#define PATH_PLANNING_COLLISION_H

#include "common.h"

inline pair<int, Vehicle> collisionDetect(const Trajectory& ego_infer, const vector<Vehicle>* predictions, Ego* ego, Map* map){
    vector<vector<double> > ego_pre;
    for(auto wp : ego_infer.points){
        vector<double> pt;
        pt.emplace_back(wp.x);
        pt.emplace_back(wp.y);
        ego_pre.emplace_back(pt);
    }

    pair<int, Vehicle> result;
    result.first = MY_INT_MAX;
    result.second = Vehicle();
    if (ego_pre.empty()){
        return result;
    }

    pair<int, Vehicle> collision;
    vector<pair<int, Vehicle> > possibles;
    //遍历所有车辆
    for(auto& veh : *predictions){
        //正后方的车不考虑
        int veh_lane = map->getLane(veh.d);
        int my_lane = map->getLane(ego->poFrenet()[1]);
        if(veh_lane == my_lane && veh.s < ego->poFrenet()[0])
            continue;

        if(ego_pre.size() - 10 > veh.prediction.size()){
            cerr<<"Warning! The size of ego trajectory is much longger than prediction!!!"<<endl;
            return collision;
        }
        //i时刻下，自身与其他一辆车的位置
        for(int i = 0; i < min(ego_pre.size(), veh.prediction.size()); i++){
            double ego_x = ego_pre[i][0];
            double ego_y = ego_pre[i][1];
            double veh_x = veh.prediction[i][0];
            double veh_y = veh.prediction[i][1];
            //碰撞发生
            if(distance(ego_x, ego_y, veh_x, veh_y) < ego->shape()[1]){
                collision.first = i;
                collision.second = veh;
                possibles.emplace_back(collision);
            }
        }
    }
    //寻找最近的碰撞点
    if(!possibles.empty()){
        sort(possibles.begin(), possibles.end(),
             [](const pair<int, Vehicle>& a, const pair<int, Vehicle>& b)
             { return a.first < b.first; });
        result = possibles.front();
    }
    return result;
}

#endif //PATH_PLANNING_COLLISION_H
