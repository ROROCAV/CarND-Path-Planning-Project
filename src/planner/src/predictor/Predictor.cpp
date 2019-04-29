//
// Created by luyifan on 19-4-24.
//

#include "predictor/Predictor.h"

Predictor::Predictor(Map *map, int life): map_(map), life_(life) {}

Predictor::~Predictor() {}

vector<Vehicle>* Predictor::predict() {
    for(auto& veh : vehicles_){
        veh.prediction.clear();
        double vx = veh.v_x;
        double vy = veh.v_y;
        double velocity = sqrt(vx*vx+vy*vy);
        double s = veh.s;
        double d = veh.d;
        //向后推100个时刻，间隔0.1秒,10秒
        for(int t = 0; t < 100; t++){
            s += velocity * 0.1;
            vector<double> tmp = map_->getXY(s, d);
            veh.prediction.emplace_back(tmp);
        }
    }
    //cout<<"predictions size: "<<vehicles_.front().prediction.size()<<endl;
    return &vehicles_;
}

void Predictor::update(vector<vector<double> > sensor_fusion) {
    for(auto v : sensor_fusion){
        int id = v[ID];
        double x = v[X];
        double y = v[Y];
        double vx = v[VX];
        double vy = v[VY];
        double s = v[S];
        double d = v[D];

        vector<Vehicle>::iterator it;
        it = std::find_if(vehicles_.begin(), vehicles_.end(),
                [id](const Vehicle& veh){ return veh.id == id; });
        //该车已经在跟踪列表中
        if(it != vehicles_.end()){
            //更新当前状态
            it->x = x;
            it->y = y;
            it->v_x = vx;
            it->v_y = vy;
            it->s = s;
            it->d = d;
            //更新历史轨迹
            vector<double> tmp = {x, y};
            it->history.emplace_back(std::make_pair(0, tmp));
        }
        //新来一辆车
        else{
            Vehicle new_veh(id, x, y, s, d, vx, vy);
            vector<double> tmp = {x, y};
            new_veh.history.emplace_back(std::make_pair(0, tmp));
            vehicles_.emplace_back(new_veh);
        }
    }

    //age > life_就删除了
    if(vehicles_.size()){
        vector<Vehicle>::iterator it1 = vehicles_.begin();
        while(it1 != vehicles_.end()){
            vector<pair<int, vector<double> > >::iterator it2 = it1->history.begin();
            while(it2 != it1->history.end()){
                if(it2->first > life_){
                    it2 = it1->history.erase(it2);
                }
                else{
                    it2++;
                }
            }
            if(it1->history.empty()){
                it1 = vehicles_.erase(it1);
            }
            else{
                it1++;
            }
        }
    }
    /*
    for(auto veh:vehicles_){
        cout<<"--------vehcile "<<veh.id<<"--------"<<endl;
        for(auto h: veh.history){
            cout<<"---"<<h.first<<"";
        }
        cout<<"---"<<endl;
    }
    */
    //将所有记录的车辆的age都加1
    for(auto& veh : vehicles_){
        for(auto& h : veh.history){
            h.first++;
        }
    }
}