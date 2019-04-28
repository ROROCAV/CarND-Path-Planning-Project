//
// Created by luyifan on 4/28/19.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#include "struct/Trajectory.h"
#include "struct/Ego.h"
#include "struct/Map.h"
#include "common/common.h"
#include "common/spline.h"

//resolution表示两个路点间的时间差
//ref_v单位是米每秒
//time表示预测多长时间的轨迹
Trajectory getPath(Ego* ego, Map* map, int lane, double ref_v, double resolution, int time);

#endif //PATH_PLANNING_HELPER_H
