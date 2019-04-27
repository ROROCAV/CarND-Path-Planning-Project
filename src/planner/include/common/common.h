//
// Created by luyifan on 4/21/19.
//

#ifndef PATH_PLANNING_COMMON_H
#define PATH_PLANNING_COMMON_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/QR>
#include "helpers.h"

using std::cout;
using std::cerr;
using std::vector;
using std::endl;
using std::string;
using std::pair;
using std::min;

enum state{ID=0, X=1, Y=2, VX=3, VY=4, S=5, D=6};

#define MY_INT_MAX std::numeric_limits<int>::max()
#define MY_DOUBLE_MAX std::numeric_limits<double>::max ()

#endif //PATH_PLANNING_COMMON_H
