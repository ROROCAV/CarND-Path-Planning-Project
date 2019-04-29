//
// Created by luyifan on 19-4-24.
//

#include "struct/Map.h"
Map::Map(string map_file){
    loadMap(map_file);
    width_ = 4;
}

Map::Map(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy):
x_(x), y_(y), s_(s), dx_(dx), dy_(dy){
    width_ = 4;
}

Map::~Map() {}

void Map::loadMap(string map_file) {
    std::ifstream in_map_(map_file.c_str(), std::ifstream::in);
    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        x_.emplace_back(x);
        y_.emplace_back(y);
        s_.emplace_back(s);
        dx_.emplace_back(d_x);
        dy_.emplace_back(d_y);
    }
}

vector<double> Map::getXY(double s, double d) {
    int prev_wp = -1;

    while (s > s_[prev_wp+1] && (prev_wp < (int)(s_.size()-1))) {
        ++prev_wp;
    }

    int wp2 = (prev_wp+1)%x_.size();

    double heading = atan2((y_[wp2]-y_[prev_wp]),
                           (x_[wp2]-x_[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-s_[prev_wp]);

    double seg_x = x_[prev_wp]+seg_s*cos(heading);
    double seg_y = y_[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-3.1415926/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}

vector<double> Map::getSD(double x, double y, double theta) {
    int next_wp = NextWaypoint(x,y, theta, x_,y_);//地图中，(x,y,theta)后面的路点的UTM

    int prev_wp;
    prev_wp = next_wp-1;
    if (next_wp == 0) {
        prev_wp  = x_.size()-1;
    }
    //前一个点和后一个点的连线L1
    double n_x = x_[next_wp]-x_[prev_wp];
    double n_y = y_[next_wp]-y_[prev_wp];
    //(x, y)和前一个点的连线L2
    double x_x = x - x_[prev_wp];
    double x_y = y - y_[prev_wp];

    //把L2投影到L1
    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    //投影的垂线就是d
    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000-x_[prev_wp];
    double center_y = 2000-y_[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i) {
        frenet_s += distance(x_[i],y_[i],x_[i+1],y_[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};
}

int Map::getLane(double d) {
    return floor(d /width_);
}