#include <uWS/uWS.h>
#include <fstream>

#include "common/common.h"
#include "common/helpers.h"
#include "json.hpp"

#include "trajectory/Generator.h"
#include "predictor/Predictor.h"

#include "struct/WayPoints.h"
#include "struct/Trajectory.h"
#include "struct/Ego.h"

#include "state/planner.h"
#include "state/StStop.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

Ego* ego = nullptr;
Map* map = nullptr;
Predictor* predictor = nullptr;
Generator* generator = nullptr;
Planner* planner = nullptr;

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file = "../data/highway_map.csv";

  //初始化所有对象
  ego = new Ego(0, 0, 0, 0, 0, 0);
  map = new Map(map_file);
  generator = new Generator(map);
  predictor = new Predictor(map, 2);
  planner = Planner::instance(ego, map);
  planner->initiate();//启动状态机
  planner->process_event(EvActivate());//车辆启动

  vector<double>* map_xs = map->x();
  vector<double>* map_ys = map->y();
  vector<double>* map_ss = map->s();
  vector<double>* map_dx = map->dx();
  vector<double>* map_dy = map->dy();
  h.onMessage([map_xs, map_ys, map_ss,
               map_dx, map_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //TODO: realize your code here.
          /*感知预处理*/
          ego->updateState(car_x, car_y, car_yaw, car_speed, car_s, car_d);
          predictor->update(sensor_fusion);
          vector<Vehicle>* prediictions = predictor->predict();
          Trajectory pre_traj_utm(previous_path_x, previous_path_y);
          if(pre_traj_utm.points.size()){
            pre_traj_utm.points.back().s = end_path_s;
            pre_traj_utm.points.back().d = end_path_d;
          }
          planner->inputEnv(&pre_traj_utm, prediictions);
          /*规划*/
          planner->process_event(EvSysTick());//处理一次状态
          //generator->laneKeeping(pre_traj_utm, ego, prediictions, next_x_vals, next_y_vals);
          planner->outputTrajectory(next_x_vals, next_y_vals);
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}