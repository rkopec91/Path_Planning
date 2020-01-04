#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "car.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

Car car;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          // Previous path data given to the car
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          vector<double> points_x;
          vector<double> points_y;

          double reference_x = car_x;
          double reference_y = car_y;
          double reference_yaw = deg2rad(car_yaw);
          double reference_vel;
          double x_value;
          double y_value;

          if(previous_path_x.size() < 2)
          {
            points_x.push_back(car_x - cos(car_yaw));
            points_x.push_back(car_x);

            points_y.push_back(car_y - sin(car_yaw));
            points_y.push_back(car_y);

            reference_vel = car_speed;
          }
          else
          {
            reference_x = previous_path_x[previous_path_x.size()-1];
            reference_y = previous_path_y[previous_path_x.size()-1];

            x_value = previous_path_x[previous_path_x.size()-2];
            y_value = previous_path_y[previous_path_x.size()-2];

            double x_differnece = reference_x-x_value;
            double y_difference = reference_y-y_value;

            reference_yaw = atan2(y_difference, x_differnece);
            reference_vel = car.targetVehicleSpeed;

            points_x.push_back(x_value);
            points_x.push_back(reference_x);

            points_y.push_back(y_value);
            points_y.push_back(reference_y);
          }

          vector<double> frenet = getFrenet(reference_x, reference_y, reference_yaw, map_waypoints_x, map_waypoints_y);

          double next_move = car.planLane(frenet[0], frenet[1], sensor_fusion);
          double lane = car.currentLane;
          double next_d = (lane * 4) + 2 + next_move;

          int check_lane = car.calculateLane(next_d);
          vector<double> front_vehicle = car.getClosestVehicle(frenet[0], check_lane, sensor_fusion, true);
          vector<double> back_vehicle = car.getClosestVehicle(frenet[0], check_lane, sensor_fusion, false);

          if (front_vehicle[0] < 10 or back_vehicle[0] < 10 or car.avgScores[check_lane] <= -5) {
            next_d = (lane * 4) + 2;
            if (check_lane != lane) {
              car.targetVehicleSpeed = car.currentLeadVehicleSpeed;
            }
          }

          vector <double> way_point1 = getXY(car_s+50, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> way_point2 = getXY(car_s+100, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> way_point3 = getXY(car_s+150, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          points_x.push_back(way_point1[0]);
          points_x.push_back(way_point2[0]);
          points_x.push_back(way_point3[0]);

          points_y.push_back(way_point1[1]);
          points_y.push_back(way_point2[1]);
          points_y.push_back(way_point3[1]);

          if (points_x.size() > 2) {
            for (int i = 0; i < points_x.size(); i++) {
              points_x[i] = ((points_x[i] - reference_x)*cos(0-reference_yaw)-(points_y[i] - reference_y)*sin(0-reference_yaw));
              points_y[i] = ((points_x[i] - reference_x)*sin(0-reference_yaw)+(points_y[i] - reference_y)*cos(0-reference_yaw));
            }


            tk::spline s;

            s.set_points(points_x, points_y);

            double target_x = 30;
            double target_y = s(target_x);
            double target_dist = pow(pow(target_x,2)+pow(target_y,2),0.5);

            double x = 0;

            for(int i = 0; i < 50 - previous_path_x.size(); i++) {
              if (reference_vel < car.targetVehicleSpeed - 0.16) {
                reference_vel += 0.16;
              } else if (reference_vel > car.targetVehicleSpeed + 0.16) {
                reference_vel -= 0.16;
              }

              double N = (target_dist/(.02*reference_vel));
              double x_point = x+(target_x)/N;
              double y_point = s(x_point);

              x = x_point;

              double x_reference_point = x_point;
              double y_reference_point = y_point;

              x_point = (x_reference_point*cos(reference_yaw)-y_reference_point*sin(reference_yaw));
              y_point = (x_reference_point*sin(reference_yaw)+y_reference_point*cos(reference_yaw));

              x_point += reference_x;
              y_point += reference_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
          }
          car.targetVehicleSpeed = reference_vel;

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