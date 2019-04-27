//
// Created by luyifan on 4/23/19.
//

#include "struct/Ego.h"

Ego::Ego(double x, double y, double yaw, double v, double s, double d): x_(x), y_(y), yaw_(yaw), s_(s), d_(d) {
    acc_ = 0;
    length_ = 3;
    width_ = 2;

}

Ego::~Ego() {}

Eigen::Vector3d Ego::poUTM() {
    return Eigen::Vector3d(x_, y_, yaw_);
}

double Ego::velocity() {
    return v_;
}

Eigen::Vector2d Ego::poFrenet() {
    return Eigen::Vector2d(s_, d_);
}

void Ego::updateState(double x, double y, double yaw, double v, double s, double d){
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    s_ = s;
    d_ = d;
    v_ = v;
    //acc_ =
}