//
// Created by luyifan on 4/21/19.
//

#include "struct/WayPoints.h"

WayPoint::WayPoint(double x, double y): x_(x), y_(y), kappa_(0) {

}

WayPoint::~WayPoint(){}

void WayPoint::setX(double x) {x_ = x;}

void WayPoint::setY(double y) {y_ = y;}

void WayPoint::setS(double s) {s_ = s;}

void WayPoint::setD(double d) {d_ = d;}

void WayPoint::setKappa(double kappa){kappa_ = kappa;}