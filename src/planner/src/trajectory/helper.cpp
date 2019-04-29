//
// Created by luyifan on 4/28/19.
//

#include "trajectory/helper.h"


Trajectory getPath(Ego* ego, Map* map, int lane, double ref_v, double resolution, int time){
    Trajectory next_path;
    double car_x = ego->poUTM()[0];
    double car_y = ego->poUTM()[1];
    double car_yaw = ego->poUTM()[2];
    double car_s = ego->poFrenet()[0];
    double car_d = ego->poFrenet()[1];

    double pre_x = car_x - cos(car_yaw);
    double pre_y = car_y - sin(car_yaw);
    double start_x = car_x;
    double start_y = car_y;
    double yaw = car_yaw;

    if(ref_v <= 0)
        return next_path;

    int loop = ceil(time / resolution)/5;
    for(int l = 0; l < loop; l++){
        Trajectory sparse;
        //头两个点确定初始的路径弧度
        sparse.points.emplace_back(WayPoint(pre_x, pre_y));
        sparse.points.emplace_back(WayPoint(start_x, start_y));

        //在Frenet坐标系下，从起点开始，每30m设置一个导航路点
        vector<double> sd = map->getSD(start_x, start_y, yaw);
        //4秒完全进入目标lane
        vector<double> next_wp0 = map->getXY(sd[0] + std::max(3 * ref_v, 10.0), (0.5*map->width()+map->width()*lane));
        vector<double> next_wp1 = map->getXY(sd[0] + std::max(3 * ref_v, 10.0)+30, (0.5*map->width()+map->width()*lane));
        vector<double> next_wp2 = map->getXY(sd[0] + std::max(3 * ref_v, 10.0)+60, (0.5*map->width()+map->width()*lane));

        sparse.points.emplace_back(WayPoint(next_wp0[0], next_wp0[1]));
        sparse.points.emplace_back(WayPoint(next_wp1[0], next_wp1[1]));
        sparse.points.emplace_back(WayPoint(next_wp2[0], next_wp2[1]));

        //确保汽车或者是前路径的最后一个点的起始值和起始角度为0，后面有用
        for(auto& wp : sparse.points){
            double shift_x = wp.x - start_x;
            double shift_y = wp.y - start_y;
            wp.x = shift_x*cos(0-yaw)-shift_y*sin(0-yaw);
            wp.y = shift_x*sin(0-yaw)+shift_y*cos(0-yaw);
        }

        //create a spline
        tk::spline s;
        s.set_points(sparse.xs(), sparse.ys());
        //计算如何控制spline上采样点的分辨率使得我们按照期望速度行驶
        double target_x = 30;
        double target_y = s(target_x);
        double target_dist = sqrt(target_x*target_x + target_y*target_y);
        double x_add_on = 0;
        for(int i = 0; i < 5; i++){
            //以汽车或者是前路径的最后一个点的位置和角度为坐标原点和x轴方向
            //在spline曲线上采样等间距点，保证速度不大于期望速度
            //点间距时间为0.1秒
            double N = (target_dist/(0.1*ref_v));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            //将得到的一系列(x, y)投影到原始坐标系
            x_point = (x_ref * cos(yaw) - y_ref*sin(yaw));
            y_point = (x_ref * sin(yaw) + y_ref*cos(yaw));

            x_point += start_x;
            y_point += start_y;

            next_path.points.emplace_back(WayPoint(x_point, y_point));
        }
        pre_x = next_path.points[next_path.points.size() - 2].x;
        pre_y = next_path.points[next_path.points.size() - 2].y;
        start_x = next_path.points.back().x;
        start_y = next_path.points.back().y;
        yaw = atan2((start_y - pre_y),(start_x - pre_x));
    }
    return next_path;
}