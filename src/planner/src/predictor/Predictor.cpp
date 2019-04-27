//
// Created by luyifan on 19-4-24.
//

#include "predictor/Predictor.h"

Predictor::Predictor(Map *map, int life): map_(map), life_(life) {}

Predictor::~Predictor() {}

vector<vector<vector<double> > > Predictor::predict() {
    vector<vector<vector<double> > > predictions;
    for(auto veh_t : vehicles_){
        vector<vector<double> > v_pre;
        double vx = veh_t.back().v_x;
        double vy = veh_t.back().v_y;
        double velocity = sqrt(vx*vx+vy*vy);
        double s = veh_t.back().s;
        double d = veh_t.back().d;
        //向后推50个时刻，间隔0.02秒
        for(int t = 0; t < 50; t++){
            s += velocity * 0.02;
            v_pre.emplace_back(map_->getXY(s, d));
        }
        predictions.emplace_back(v_pre);
    }
    return predictions;
}

void Predictor::update(vector<vector<double> > sensor_fusion) {
    for(auto v : sensor_fusion){
        bool bFind = false;
        int id = v[ID];
        double x = v[X];
        double y = v[Y];
        double vx = v[VX];
        double vy = v[VY];
        double s = v[S];
        double d = v[D];
        Vehicle new_veh(id, x, y, s, d, vx, vy);

        for(auto& veh_t : vehicles_){
            if(id == veh_t.front().id){
                veh_t.emplace_back(new_veh);
                bFind = true;
            }
        }
        //新来一辆车
        if(!bFind){
            vector<Vehicle> new_veh_t;
            new_veh_t.emplace_back(new_veh);
            vehicles_.emplace_back(new_veh_t);
        }
    }
    //将所有记录的车辆的age都加1
    for(auto& veh_t : vehicles_){
        for(auto& veh : veh_t){
            veh.age++;
        }
    }
    //age大于一定值就删除了
    vector<vector<Vehicle> >::iterator it1 = vehicles_.begin();
    while(it1 != vehicles_.end()){
        vector<Vehicle>::iterator it2 = it1->begin();
        while (it2 != it1->end()){
            if(it2->age > life_)
                it2 = it1->erase(it2);
            else
                it2++;
        }
        if(it1->empty())
            it1 = vehicles_.erase(it1);
        else
            it1++;
    }
    for(auto veh_t:vehicles_){
        //cout<<"--------vehcile "<<veh_t.back().id<<"--------"<<endl;
        for(auto veh: veh_t){
            //cout<<"---"<<veh.age<<"";
        }
        //cout<<"---"<<endl;
    }
}