//
// Created by luyifan on 4/27/19.
//

#ifndef PATH_PLANNING_COLLISION_H
#define PATH_PLANNING_COLLISION_H

#include "common.h"

inline pair<int, double> collisionDetect(const vector<vector<double> >& ego_pre, const vector<vector<vector<double> > >& predictions, const Ego* ego){
    pair<int, double> collision;
    vector<pair<int, double> > possibles;
    //遍历所有车辆
    for(auto veh_t : predictions){
        if(ego_pre.size() - 10 > veh_t.size()){
            cerr<<"Warning! The size of ego trajectory is much longger than prediction!!!"<<endl;
            return collision;
        }
        //i时刻下，自身与其他一辆车的位置
        for(int i = 0; i < min(ego_pre.size(), veh_t.size()); i++){
            double ego_x = ego_pre[i][0];
            double ego_y = ego_pre[i][1];
            double veh_x = veh_t[i][0];
            double veh_y = veh_t[i][1];
            //碰撞发生
            if(distance(ego_x, ego_y, veh_x, veh_y) < ego->shape()[0]){
                collision[0] = i;
                collision[1] = sqrt((ego_x-veh_x)*(ego_x-veh_x) +
                                    (ego_y-veh_y)*(ego_y-veh_y));
                possibles.emplace_back(collision);
            }
        }
    }
    //寻找最近的碰撞点
    sort(possibles.begin(), possibles.end(),
            [](const pair<int, double>& a, const pair<int, double>& b)
            { return a[0] < a[1]; })

    pair<int, double> result = possibles.front();
    return result;
}

#endif //PATH_PLANNING_COLLISION_H
