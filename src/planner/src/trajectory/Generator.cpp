//
// Created by luyifan on 4/21/19.
//
#include "trajectory/Generator.h"

Generator::Generator(Map* map): map_(map) {}

Generator::~Generator(){}

void Generator::example(Trajectory pre_traj_utm, Ego* ego, const vector<vector<double> >& vehicles,
                        vector<double>& next_x, vector<double>& next_y){
    next_x.clear();
    next_y.clear();

    int lane = 1;
    double ref_vel = 49.5; //mph
    Trajectory sparse; //稀疏的导航路点

    double car_s = ego->poFrenet()[0];
    double car_d = ego->poFrenet()[1];
    double car_x = ego->poUTM()[0];
    double car_y = ego->poUTM()[1];
    double car_yaw = deg2rad(ego->poUTM()[2]);
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = car_yaw;
    int pre_path_size = pre_traj_utm.length();

    //如果剩下的点不多了，从车当前位置开始规划
    if(pre_path_size < 2){
        double pre_car_x = car_x - cos(car_yaw);
        double pre_car_y = car_y - cos(car_yaw);

        sparse.insertWayPoint(WayPoint(pre_car_x, pre_car_y));
        sparse.insertWayPoint(WayPoint(car_x, car_y));
    }
    //用前一条路的终点作为本次的起点
    else{
        ref_x = pre_traj_utm.val().back().x();
        ref_y = pre_traj_utm.val().back().y();

        double pre_ref_x = pre_traj_utm.val()[pre_path_size - 2].x();
        double pre_ref_y = pre_traj_utm.val()[pre_path_size - 2].y();
        ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x);

        sparse.insertWayPoint(WayPoint(pre_ref_x, pre_ref_y));
        sparse.insertWayPoint(WayPoint(ref_x, ref_y));
    }

    //在Frenet坐标系下，从起点开始，每30m设置一个导航路点
    vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_->s(), map_->x(), map_->y());
    vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_->s(), map_->x(), map_->y());
    vector<double> next_wp2 = getXY(car_s + 60, (2+4*lane), map_->s(), map_->x(), map_->y());

    sparse.insertWayPoint(WayPoint(next_wp0[0], next_wp0[1]));
    sparse.insertWayPoint(WayPoint(next_wp1[0], next_wp1[1]));
    sparse.insertWayPoint(WayPoint(next_wp2[0], next_wp2[1]));


    for(auto& wp : sparse.val()){
        //shift car reference angle to 0 degrees.
        double shift_x = wp.x() - ref_x;
        double shift_y = wp.y() - ref_y;

        wp.setX(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
        wp.setY(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
    }

}


void Generator::laneKeeping(Trajectory pre_traj_utm, Ego* ego, const vector<vector<double> >& vehicles,
                            vector<double>& next_x, vector<double>& next_y) {
    double s = ego->poFrenet()[0];
    double d = ego->poFrenet()[1];
    double s_dot = ego->velocity();
    //Eigen::Vector3d s_start(s, )

    int path_size = pre_traj_utm.length();//上一次规划的路劲剩下没跑的长度

    //第一次规划
    if(path_size == 0){

    }


}