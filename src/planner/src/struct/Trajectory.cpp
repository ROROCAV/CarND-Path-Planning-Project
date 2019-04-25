//
// Created by luyifan on 4/21/19.
//

#include "struct/Trajectory.h"

Trajectory::Trajectory() {}

Trajectory::Trajectory(vector<double> x, vector<double> y){
    if(x.size() != y.size()){
        cerr<<"size of x and y are not equal!"<<endl;
        return ;
    }
    val_.clear();
    for(int i = 0; i < x.size(); i++){
        val_.emplace_back(WayPoint(x[i], y[i]));
    }
}

Trajectory::~Trajectory(){}

void Trajectory::insertWayPoint(WayPoint pt) {
    val_.emplace_back(pt);
}

void Trajectory::setCost(double cost) {
    cost_ = cost;
}

void Trajectory::computeKappa() {
    if(val_.size() < 4){
        cerr<<"The trajectory is almost empty. No enough way points to compute kappa."<<endl;
    }
    for(int i = 1; i < val_.size() - 1; i++){
        //TODO
        double k = 0.0;
        val_[i].setKappa(k);
    }
}

vector<double> Trajectory::xs() {
    vector<double> xs;
    for(auto pt:val_){
        xs.emplace_back(pt.x());
    }
    return xs;
}

vector<double> Trajectory::ys() {
    vector<double> ys;
    for(auto pt:val_){
        ys.emplace_back(pt.y());
    }
    return ys;
}