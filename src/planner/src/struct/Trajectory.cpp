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
    points.clear();
    for(int i = 0; i < x.size(); i++){
        points.emplace_back(WayPoint(x[i], y[i]));
    }
}

Trajectory::~Trajectory(){}

void Trajectory::computeKappa() {
    if(points.size() < 4){
        cerr<<"The trajectory is almost empty. No enough way points to compute kappa."<<endl;
    }
    for(int i = 1; i < points.size() - 1; i++){
        //TODO
        double k = 0.0;
    }
}

vector<double> Trajectory::xs() {
    vector<double> xs;
    for(auto pt:points){
        xs.emplace_back(pt.x);
    }
    return xs;
}

vector<double> Trajectory::ys() {
    vector<double> ys;
    for(auto pt:points){
        ys.emplace_back(pt.y);
    }
    return ys;
}