#include "car.hpp"

int Car::planLaneTransition(double s, double d, vector<vector<double>> sensor_fusion) {
  current_lane = getLane(d);
  int next_lane;

  if (getClosestVehicle(s, current_lane, sensor_fusion, true)[0] > 20) {
    next_lane = current_lane;
    target_speed = 22.302;
    average_scores = {0,0,0};
    return 0;
  } else {
    next_lane = calculateLaneScore(s, current_lane, sensor_fusion);
    target_speed = getClosestVehicle(s, next_lane, sensor_fusion, true)[1];
  }

  if (next_lane == current_lane) {
    return 0;
  } else if (next_lane > current_lane) {
    return 4;
  } else {
    return -4;
  }
}

int Car::getLane(double d) {
  if (d < 4) {
    return 0;
  } else if (d < 8) {
    return 1;
  } else {
    return 2;
  }
}

vector<double> Car::getClosestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool front) {
  double distance = 10000;
  double velocity = 22.302;

  for(int i = 0; i < sensor_fusion.size(); i++) {

    if (getLane(sensor_fusion[i][6]) == lane) {
      if (front) {
        if (sensor_fusion[i][5] > s and (sensor_fusion[i][5] - s) < distance) {
          velocity = pow(pow(sensor_fusion[i][3], 2)+pow(sensor_fusion[i][4], 2),0.5);
          distance = sensor_fusion[i][5] - s;
        }
      } else {
        if (s >= sensor_fusion[i][5] and (s - sensor_fusion[i][5]) < distance) {
          velocity = pow(pow(sensor_fusion[i][3], 2)+pow(sensor_fusion[i][4], 2),0.5);
          distance = s - sensor_fusion[i][5];
        }
      }
    }
  }
  if (distance <= 0) { distance = 1.0; }
  if (lane == current_lane and front == true) { lead_vehicle_speed = velocity; }
  return {distance, velocity};
}

void Car::updateAverageScores(int i, vector <double> scores) {
    average_scores[i] = (average_scores[i] * 10) - average_scores[i] + scores[i];
    average_scores[i] /= 10;
}

int Car::compareLanes(int lane) {
  int maximum;
  if (lane == 0) {
    maximum = max_element(average_scores.begin(), average_scores.end() - 1) - average_scores.begin();
  } else if (lane == 1) {
    maximum = max_element(average_scores.begin(), average_scores.end())  - average_scores.begin();
  } else {
    maximum = max_element(average_scores.begin() + 1, average_scores.end())  - average_scores.begin();
  }
  return maximum;
}

int Car::calculateLaneScore(double s, int lane, vector<vector<double>> sensor_fusion) {
  vector <double> scores = {0,0,0};
  vector <double> front_vehicle;
  vector <double> trailing_vehicle;

  for (int i = 0; i < 3; i++) {

    if (i == lane) { scores[i] += 0.5; }

    front_vehicle = getClosestVehicle(s, i, sensor_fusion, true);
    trailing_vehicle = getClosestVehicle(s, i, sensor_fusion, false);
    if (front_vehicle[0] > 1000 and trailing_vehicle[0] > 1000) {
      scores[i] += 5;
    } else {
      if (trailing_vehicle[0] < 10) {
        scores[i] -= 5;
      }
      if (front_vehicle[0] < 10) {
        scores[i] -= 5; 
      }
      scores[i] += 1 - (10/(front_vehicle[1]/2));
      scores[i] -= 1 / (trailing_vehicle[1]/2);

      scores[i] += 1 - (10/(front_vehicle[0]/3));
      scores[i] += 1 - (10/(trailing_vehicle[0]/3));
    }
    updateAverageScores(i, scores);
  }
  return compareLanes(lane);
}