#include "car.hpp"

int Car::planLane(double s, double d, vector<vector<double>> sensor_fusion) {
  int lane = calculateLane(d);
  int newLane;
  double distance = getClosestVehicle(s, lane, sensor_fusion, true)[0];

  currentLane = lane;

  if (distance > 20) {
    newLane = lane;
    targetVehicleSpeed = 22.352 - 0.5;
    avgScores = {0,0,0};
    return 0;
  } else {
    newLane = calculateScore(s, lane, sensor_fusion);
    vector <double> vehicle = getClosestVehicle(s, newLane, sensor_fusion, true);
    targetVehicleSpeed = vehicle[1];
  }

  if (newLane == lane) {
    return 0;
  } else if (newLane < lane) {
    return -4;
  } else {
    return 4;
  }
}

int Car::calculateLane(double d) {
  if (d < 4) {
    return 0;
  } else if (d < 8) {
    return 1;
  } else {
    return 2;
  }
}

vector<double> Car::getClosestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction) {
  double distance = 10000;
  double velocity = 22.352 - 0.5;
  double vehicle_s;
  double vehicle_d;
  double vehicle_v;
  int vehicle_lane;

  for(int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++) {
    vehicle_s = sensor_fusion[vehicle][5];
    vehicle_d = sensor_fusion[vehicle][6];
    vehicle_v = sqrt(pow(sensor_fusion[vehicle][3], 2)+pow(sensor_fusion[vehicle][4], 2));
    vehicle_lane = calculateLane(vehicle_d);

    if (vehicle_lane == lane) {
      if (direction == true) {
        if (vehicle_s > s and (vehicle_s - s) < distance) {
          distance = vehicle_s - s;
          velocity = vehicle_v;
        }
      } else {
        if (s >= vehicle_s and (s - vehicle_s) < distance) {
          distance = s - vehicle_s;
          velocity = vehicle_v;
        }
      }
    }
  }
  if (distance <= 0) {
    distance = 1.0;
  }
  if (lane == currentLane and direction == true) {
    currentLeadVehicleSpeed = velocity;
  }
  return {distance, velocity};
}

int Car::calculateScore(double s, int lane, vector<vector<double>> sensor_fusion) {
  vector <double> scores = {0,0,0};
  vector <double> front_vehicle;
  vector <double> back_vehicle;

  for (int i = 0; i < 3; i++) {
    if (i == lane) {
      scores[i] += 0.5;
    }
    front_vehicle = getClosestVehicle(s, i, sensor_fusion, true);
    back_vehicle = getClosestVehicle(s, i, sensor_fusion, false);
    if (front_vehicle[0] > 1000 and back_vehicle[0] > 1000) {
      scores[i] += 5;
    } else {
      if (front_vehicle[0] < 10) {
        scores[i] -= 5;
      }
      if (back_vehicle[0] < 10) {
        scores[i] -= 5;
      }
      scores[i] += 1 - (10/(front_vehicle[0]/3));
      scores[i] += 1 - (10/(back_vehicle[0]/3));
      scores[i] += 1 - (10/(front_vehicle[1]/2));
      scores[i] += 1 / (back_vehicle[1]/2);
    }

    avgScores[i] = (avgScores[i] * 10) - avgScores[i];
    avgScores[i] += scores[i];
    avgScores[i] /= 10;
  }

  if (lane == 0) {
    return max_element(avgScores.begin(), avgScores.end() - 1) - avgScores.begin();
  } else if (lane == 1) {
    return max_element(avgScores.begin(), avgScores.end())  - avgScores.begin();
  } else {
    return max_element(avgScores.begin() + 1, avgScores.end())  - avgScores.begin();
  }
}