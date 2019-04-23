//
// Created by luyifan on 4/21/19.
//
#include "trajectory/Generator.h"

Generator::Generator(){}

Generator::~Generator(){}

void Generator::generate(const vector<double>& previous_path_x, const vector<double>& previous_path_y, double car_x, double car_y,
              double car_yaw, const vector<vector<double> >& vehicles, vector<double>& next_x_vals, vector<double>& next_y_vals){
    double pos_x;
    double pos_y;
    double angle;
    int path_size = previous_path_x.size();//上一次规划的路劲剩下没跑的长度

    //先把上一次规划的路径放进去
    for(int i = 0; i < path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    //如果是第一次规划
    if(path_size == 0)
    {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    }
    else
    {
        pos_x = previous_path_x[path_size-1];
        pos_y = previous_path_y[path_size-1];

        double pos_x2 = previous_path_x[path_size-2];
        double pos_y2 = previous_path_y[path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    }

    double dist_inc = 0.5;


    for(int i = 0; i < 50-path_size; i++)
    {
        next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
        next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
        pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
        pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    }
}

void Generator::laneKeeping(Trajectory pre_traj_utm, Ego ego, const vector<vector<double> > &vehicles,
                            Trajectory next_traj_utm) {
    double s = ego.poFrenet()[0];
    double d = ego.poFrenet()[1];
    double s_dot = ego.velocity();
    //Eigen::Vector3d s_start(s, )

    int path_size = pre_traj_utm.length();//上一次规划的路劲剩下没跑的长度

    //第一次规划
    if(path_size == 0){

    }


}