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

          vector<double> points_x;
          vector<double> points_y;

          double reference_velocity;
          double reference_x = car_x;
          double reference_y = car_y;
          double reference_yaw = deg2rad(car_yaw);

          next_x_vals.insert(std::end(next_x_vals), std::begin(previous_path_x), std::end(previous_path_x));
          next_y_vals.insert(std::end(next_y_vals), std::begin(previous_path_y), std::end(previous_path_y));

          if(previous_path_x.size() < 2)
          {
            points_x.push_back(car_x - cos(car_yaw));
            points_x.push_back(car_x);

            points_y.push_back(car_y - sin(car_yaw));
            points_y.push_back(car_y);

            reference_velocity = car_speed;
          }
          else
          {
            reference_x = previous_path_x.end()[-1];
            reference_y = previous_path_y.end()[-1];
            double previous_x_point = previous_path_x.end()[-2];
            double previous_y_point = previous_path_y.end()[-2];

            reference_yaw = atan2(reference_y-previous_y_point,reference_x-previous_x_point);
            reference_velocity = car.target_speed;

            points_x.push_back(previous_x_point);
            points_x.push_back(reference_x);

            points_y.push_back(previous_y_point);
            points_y.push_back(reference_y);
          }

          vector<double> frenet_vector = getFrenet(reference_x, reference_y, reference_yaw, map_waypoints_x, map_waypoints_y);

          double next_d_position = (car.current_lane * 4) + 2 + car.planLaneTransition(frenet_vector[0], frenet_vector[1], sensor_fusion);
          int min_distance = 10;
          int next_lane = car.getLane(next_d_position);
          vector<double> front_vehicle = car.getClosestVehicle(frenet_vector[0], next_lane, sensor_fusion, true);
          vector<double> trailing_vehicle = car.getClosestVehicle(frenet_vector[0], next_lane, sensor_fusion, false);

          if (front_vehicle[0] < min_distance or trailing_vehicle[0] < min_distance or car.average_scores[next_lane] <= -5) {
            next_d_position = (car.current_lane * 4) + 2;
            if (next_lane != car.current_lane) {
              car.target_speed = car.lead_vehicle_speed;
            }
          }

          vector <double> waypoint1 = getXY(car_s+50, next_d_position, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> waypoint2 = getXY(car_s+100, next_d_position, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector <double> waypoint3 = getXY(car_s+150, next_d_position, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          points_x.push_back(waypoint1[0]);
          points_x.push_back(waypoint2[0]);
          points_x.push_back(waypoint3[0]);
          points_y.push_back(waypoint1[1]);
          points_y.push_back(waypoint2[1]);
          points_y.push_back(waypoint3[1]);

          if (points_x.size() > 2) {
            for (int i = 0; i < points_x.size(); i++) {
              double move_x_direction = points_x[i] - reference_x;
              double move_y_direction = points_y[i] - reference_y;

              points_x[i] = (move_x_direction*cos(0-reference_yaw)-move_y_direction*sin(0-reference_yaw));
              points_y[i] = (move_x_direction*sin(0-reference_yaw)+move_y_direction*cos(0-reference_yaw));
            }

            tk::spline s;

            s.set_points(points_x, points_y);

            double target_dist = sqrt(pow(30,2)+pow(s(30),2));

            double x_added = 0;

            for(int i = 0; i < 50 - previous_path_x.size(); i++) {
              if (reference_velocity < car.target_speed - 0.16) {
                reference_velocity += 0.16;
              } else if (reference_velocity > car.target_speed + 0.16) {
                reference_velocity -= 0.16;
              }

              double x = x_added+(30)/(target_dist/(.02*reference_velocity));
              double y = s(x);

              x_added = x;

              double next_x_point = (x * cos(reference_yaw) - y * sin(reference_yaw));
              double next_y_point = (x * sin(reference_yaw) + y * cos(reference_yaw));

              next_x_point += reference_x;
              next_y_point += reference_y;

              next_x_vals.push_back(next_x_point);
              next_y_vals.push_back(next_y_point);
            }
          }

          car.target_speed = reference_velocity;

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