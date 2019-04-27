//
// Created by luyifan on 4/21/19.
//
#include "trajectory/Generator.h"

Generator::Generator(Map* map): map_(map), ref_vel_(0) {
    ref_lane_ = 1;
}

Generator::~Generator(){}

void Generator::example(Trajectory pre_traj_utm, Ego* ego, const vector<vector<double> >& vehicles,
                        vector<double>& next_x, vector<double>& next_y){
    int pre_path_size = pre_traj_utm.points.size();
    double car_s = ego->poFrenet()[0];
    double car_d = ego->poFrenet()[1];
    double car_x = ego->poUTM()[0];
    double car_y = ego->poUTM()[1];
    double car_yaw = deg2rad(ego->poUTM()[2]);
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = car_yaw;
    next_x.clear();
    next_y.clear();

    //处理其他车辆
    if(pre_path_size > 0)
        car_s = pre_traj_utm.points.back().s;

    bool too_close = false;
    for(int i = 0; i < vehicles.size(); i++){
        //car is in my lane
        float d = vehicles[i][D];
        if(d < (0.5*map_->width()+map_->width()*ref_lane_+0.5*map_->width()) && d > (0.5*map_->width()+map_->width()*ref_lane_-0.5*map_->width())){
            double vx = vehicles[i][VX];
            double vy = vehicles[i][VY];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = vehicles[i][S];
            //预测一段时间后车辆前进的距离
            check_car_s += (double)(pre_path_size*0.02*check_speed);

            if((check_car_s > car_s) && (check_car_s - car_s) < 30){
                //减减速或者换道
                //ref_vel_ = 29.5;//mph
                too_close = true;
            }
        }
    }

    //渐进式的速度加减
    if(too_close){
        ref_vel_ -= .224;//.224是计算出的每0.02秒最大加速度
    }
    else if(ref_vel_ < 49.5){
        ref_vel_ += .224;
    }

    Trajectory sparse; //稀疏的导航路点
    //如果剩下的点不多了，从车当前位置开始规划
    if(pre_path_size < 2){
        double pre_car_x = car_x - cos(car_yaw);
        double pre_car_y = car_y - sin(car_yaw);

        sparse.points.emplace_back(WayPoint(pre_car_x, pre_car_y));
        sparse.points.emplace_back(WayPoint(car_x, car_y));
    }
    //用前一条路的终点作为本次的起点
    else{
        ref_x = pre_traj_utm.xs().back();
        ref_y = pre_traj_utm.ys().back();

        double pre_ref_x = pre_traj_utm.xs()[pre_path_size - 2];
        double pre_ref_y = pre_traj_utm.ys()[pre_path_size - 2];
        ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x);

        sparse.points.emplace_back(WayPoint(pre_ref_x, pre_ref_y));
        sparse.points.emplace_back(WayPoint(ref_x, ref_y));
    }

    //在Frenet坐标系下，从起点开始，每30m设置一个导航路点

    vector<double> next_wp0 = map_->getXY(car_s + 30, (0.5*map_->width()+map_->width()*ref_lane_));
    vector<double> next_wp1 = map_->getXY(car_s + 60, (0.5*map_->width()+map_->width()*ref_lane_));
    vector<double> next_wp2 = map_->getXY(car_s + 90, (0.5*map_->width()+map_->width()*ref_lane_));

    sparse.points.emplace_back(WayPoint(next_wp0[0], next_wp0[1]));
    sparse.points.emplace_back(WayPoint(next_wp1[0], next_wp1[1]));
    sparse.points.emplace_back(WayPoint(next_wp2[0], next_wp2[1]));

    //确保汽车或者是前路径的最后一个点的起始值和起始角度为0，后面有用
    for(auto& wp : sparse.points){
        //shift car reference angle to 0 degrees.
        double shift_x = wp.x - ref_x;
        double shift_y = wp.y - ref_y;

        wp.x = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
        wp.y = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
    }

    //create a spline
    tk::spline s;
    s.set_points(sparse.xs(), sparse.ys());

    //将剩下的点放入下一次的轨迹
    Trajectory next_traj_utm;
    for(auto wp : pre_traj_utm.points)
        next_traj_utm.points.emplace_back(wp);

    //计算如何控制spline上采样点的分辨率使得我们按照期望速度行驶
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;

    for(int i = 0; i < 50 - pre_path_size; i++){
        //以汽车或者是前路径的最后一个点的位置和角度为坐标原点和x轴方向
        //在spline曲线上采样等间距点，保证速度不大于期望速度
        double N = (target_dist/(0.02*ref_vel_/2.24));
        double x_point = x_add_on+(target_x)/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;
        //将得到的一系列(x, y)投影到原始坐标系
        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_traj_utm.points.emplace_back(WayPoint(x_point, y_point));
    }
    next_x = next_traj_utm.xs();
    next_y = next_traj_utm.ys();
}


void Generator::laneKeeping(Trajectory pre_traj_utm, Ego* ego, const vector<vector<vector<double> > >& predictions,
                            vector<double>& next_x, vector<double>& next_y) {
    int pre_path_size = pre_traj_utm.points.size();
    double car_s = ego->poFrenet()[0];
    double car_d = ego->poFrenet()[1];
    double car_x = ego->poUTM()[0];
    double car_y = ego->poUTM()[1];
    double car_yaw = deg2rad(ego->poUTM()[2]);
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = car_yaw;
    double max_speed = 49.5;//道路允许的最大速度
    int lane = map_->getCurrentLane(car_d);
    next_x.clear();
    next_y.clear();

    if(pre_path_size > 0)
        car_s = pre_traj_utm.points.back().s;
    //先按照当前速度生成一条匀速轨迹
    Trajectory infer_1 = inference(pre_traj_utm, ego, lane, ego->velocity());
    //结合预测轨迹，碰撞检查
    pair<int, double> col = collisionDetect(infer_1, predictions, ego);
    cout<<"collision at: "<<col.first<<endl;
    //渐进式的速度加减
    if(col.first < 50){
        ref_vel_ -= .224;//.224是计算出的每0.02秒最大加速度
    }
    else if(ref_vel_ < max_speed){
        ref_vel_ += .224;
    }

    Trajectory sparse; //稀疏的导航路点
    //如果剩下的点不多了，从车当前位置开始规划
    if(pre_path_size < 2){
        double pre_car_x = car_x - cos(car_yaw);
        double pre_car_y = car_y - sin(car_yaw);

        sparse.points.emplace_back(WayPoint(pre_car_x, pre_car_y));
        sparse.points.emplace_back(WayPoint(car_x, car_y));
    }
    //用前一条路的终点作为本次的起点
    else{
        ref_x = pre_traj_utm.xs().back();
        ref_y = pre_traj_utm.ys().back();

        double pre_ref_x = pre_traj_utm.xs()[pre_path_size - 2];
        double pre_ref_y = pre_traj_utm.ys()[pre_path_size - 2];
        ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x);

        sparse.points.emplace_back(WayPoint(pre_ref_x, pre_ref_y));
        sparse.points.emplace_back(WayPoint(ref_x, ref_y));
    }

    //在Frenet坐标系下，从起点开始，每30m设置一个导航路点
    vector<double> next_wp0 = map_->getXY(car_s + 30, (0.5*map_->width()+map_->width()*lane));
    vector<double> next_wp1 = map_->getXY(car_s + 60, (0.5*map_->width()+map_->width()*lane));
    vector<double> next_wp2 = map_->getXY(car_s + 90, (0.5*map_->width()+map_->width()*lane));

    sparse.points.emplace_back(WayPoint(next_wp0[0], next_wp0[1]));
    sparse.points.emplace_back(WayPoint(next_wp1[0], next_wp1[1]));
    sparse.points.emplace_back(WayPoint(next_wp2[0], next_wp2[1]));

    //确保汽车或者是前路径的最后一个点的起始值和起始角度为0，后面有用
    for(auto& wp : sparse.points){
        //shift car reference angle to 0 degrees.
        double shift_x = wp.x - ref_x;
        double shift_y = wp.y - ref_y;

        wp.x = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
        wp.y = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
    }

    //create a spline
    tk::spline s;
    s.set_points(sparse.xs(), sparse.ys());

    //将剩下的点放入下一次的轨迹
    Trajectory next_traj_utm;
    for(auto wp : pre_traj_utm.points)
        next_traj_utm.points.emplace_back(wp);

    //计算如何控制spline上采样点的分辨率使得我们按照期望速度行驶
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_add_on = 0;
    for(int i = 0; i < 50 - pre_path_size; i++){
        //以汽车或者是前路径的最后一个点的位置和角度为坐标原点和x轴方向
        //在spline曲线上采样等间距点，保证速度不大于期望速度
        double N = (target_dist/(0.02*ref_vel_/2.24));
        double x_point = x_add_on+(target_x)/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;
        //将得到的一系列(x, y)投影到原始坐标系
        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_traj_utm.points.emplace_back(WayPoint(x_point, y_point));
    }
    next_x = next_traj_utm.xs();
    next_y = next_traj_utm.ys();
}

//预测的时候，为了减少计算量，生成粗略的路径，每两个路点间隔1米
Trajectory Generator::inference(Trajectory pre_traj_utm, Ego* ego, int lane, double ref_vel) {
    int pre_path_size = pre_traj_utm.points.size();
    double car_s = ego->poFrenet()[0];
    double car_d = ego->poFrenet()[1];
    double car_x = ego->poUTM()[0];
    double car_y = ego->poUTM()[1];
    double car_yaw = deg2rad(ego->poUTM()[2]);
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = car_yaw;

    if(pre_path_size > 0)
        car_s = pre_traj_utm.points.back().s;

    Trajectory sparse; //稀疏的导航路点
    //如果剩下的点不多了，从车当前位置开始规划
    if(pre_path_size < 2){
        double pre_car_x = car_x - cos(car_yaw);
        double pre_car_y = car_y - sin(car_yaw);

        sparse.points.emplace_back(WayPoint(pre_car_x, pre_car_y));
        sparse.points.emplace_back(WayPoint(car_x, car_y));
    }
        //用前一条路的终点作为本次的起点
    else{
        ref_x = pre_traj_utm.xs().back();
        ref_y = pre_traj_utm.ys().back();

        double pre_ref_x = pre_traj_utm.xs()[pre_path_size - 2];
        double pre_ref_y = pre_traj_utm.ys()[pre_path_size - 2];
        ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x);

        sparse.points.emplace_back(WayPoint(pre_ref_x, pre_ref_y));
        sparse.points.emplace_back(WayPoint(ref_x, ref_y));
    }

    //在Frenet坐标系下，从起点开始，每30m设置一个导航路点
    vector<double> next_wp0 = map_->getXY(car_s + 30, (0.5*map_->width()+map_->width()*lane));
    vector<double> next_wp1 = map_->getXY(car_s + 60, (0.5*map_->width()+map_->width()*lane));
    vector<double> next_wp2 = map_->getXY(car_s + 90, (0.5*map_->width()+map_->width()*lane));

    sparse.points.emplace_back(WayPoint(next_wp0[0], next_wp0[1]));
    sparse.points.emplace_back(WayPoint(next_wp1[0], next_wp1[1]));
    sparse.points.emplace_back(WayPoint(next_wp2[0], next_wp2[1]));

    //确保汽车或者是前路径的最后一个点的起始值和起始角度为0，后面有用
    for(auto& wp : sparse.points){
        double shift_x = wp.x - ref_x;
        double shift_y = wp.y - ref_y;

        wp.x = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
        wp.y = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
    }

    //create a spline
    tk::spline s;
    s.set_points(sparse.xs(), sparse.ys());
    //将剩下的点放入下一次的轨迹
    Trajectory next_traj_utm;
    for(auto wp : pre_traj_utm.points)
        next_traj_utm.points.emplace_back(wp);

    //计算如何控制spline上采样点的分辨率使得我们按照期望速度行驶
    double target_x = 30;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_add_on = 0;
    for(int i = 0; i < 100 - pre_path_size; i++){
        //以汽车或者是前路径的最后一个点的位置和角度为坐标原点和x轴方向
        //在spline曲线上采样等间距点，保证速度不大于期望速度
        double N = (target_dist/(0.02*ref_vel/2.24));
        double x_point = x_add_on+(target_x)/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;
        //将得到的一系列(x, y)投影到原始坐标系
        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_traj_utm.points.emplace_back(WayPoint(x_point, y_point));
    }
    return next_traj_utm;
}