//
// Created by luyifan on 4/26/19.
//

#include "struct/Vehicle.h"

Vehicle::Vehicle():id(0), x(0), y(0), s(0), d(0), v_x(0), v_y(0) {
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double v_x, double v_y)
:id(id), x(x), y(y), s(s), d(d), v_x(v_x), v_y(v_y){
}

Vehicle::~Vehicle() {}
