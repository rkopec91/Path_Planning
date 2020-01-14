#include "car.hpp"

int Car::planLane(double s, double d, vector<vector<double>> sensor_fusion) {

  if (d < 4) {
    int lane 0;
  } else if (d < 8) {
    int lane 1;
  } else {
    int lane 2;
  }
  
  int newLane;
  double distance = getClosestVehicle(s, lane, sensor_fusion, true)[0];

  currentLane = lane;

  if (distance > 20) {
    newLane = lane;
    target_speed = 22.352 - 0.5;
    average_scores = {0,0,0};
    return 0;
  } else {
    newLane = calculateScore(s, lane, sensor_fusion);
    vector <double> vehicle = getClosestVehicle(s, newLane, sensor_fusion, true);
    target_speed = vehicle[1];
  }

  if (newLane == lane) {
    return 0;
  } else if (newLane < lane) {
    return -4;
  } else {
    return 4;
  }
}

vector<double> Car::getClosestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction) {
  double distance = 100000;
  double velocity = 22.352 - 0.5;
  int dlane;

  for(int i = 0; i < sensor_fusion.size(); i++) {

    if (sensor_fusion[i][6] < 4) {
      dlane = 0;
    } else if (sensor_fusion[i][6] < 8) {
      dlane = 1;
    } else {
      dlane = 2;
    }

    if (dlane == lane) {
      if (direction == true) {
        if (sensor_fusion[i][5] > s and (sensor_fusion[i][5] - s) < distance) {
          distance = sensor_fusion[i][5] - s;
          velocity = sqrt(pow(sensor_fusion[i][3], 2)+pow(sensor_fusion[i][4], 2));
        }
      } else {
        if (s >= sensor_fusion[i][5] and (s - sensor_fusion[i][5]) < distance) {
          distance = s - sensor_fusion[i][5];
          velocity = sqrt(pow(sensor_fusion[i][3], 2)+pow(sensor_fusion[i][4], 2));
        }
      }
    }
  }
  if (distance <= 0) {
    distance = 1.0;
  }
  if (lane == currentLane and direction == true) {
    current_lead_speed = velocity;
  }
  return {distance, velocity};
}

int Car::calculateScore(double s, int lane, vector<vector<double>> sensor_fusion) {
  vector <double> scores = {0,0,0};
  vector <double> inFrontVehicle;
  vector <double> behindVehicle;

  for (int i = 0; i < 3; i++) {
    if (i == lane) {
      scores[i] += 0.5;
    }
    inFrontVehicle = getClosestVehicle(s, i, sensor_fusion, true);
    behindVehicle = getClosestVehicle(s, i, sensor_fusion, false);
    if (inFrontVehicle[0] > 1000 and behindVehicle[0] > 1000) {
      scores[i] += 5;
    } else {
      if (inFrontVehicle[0] < 10) {
        scores[i] -= 5;
      }
      if (behindVehicle[0] < 10) {
        scores[i] -= 5;
      }
      scores[i] += 1 - (10/(inFrontVehicle[1]/2));
      scores[i] += 1 / (behindVehicle[1]/2);
      scores[i] += 1 - (10/(inFrontVehicle[0]/3));
      scores[i] += 1 - (10/(behindVehicle[0]/3));
    }

    average_scores[i] = (average_scores[i] * 10) - average_scores[i];
    average_scores[i] += scores[i];
    average_scores[i] /= 10;
  }

  if (lane == 0) {
    max = max_element(average_scores.begin(), average_scores.end() - 1) - average_scores.begin();
    return max;
  } else if (lane == 1) {
    max = max_element(average_scores.begin(), average_scores.end())  - average_scores.begin();
    return max;
  } else {
    max = max_element(average_scores.begin() + 1, average_scores.end())  - average_scores.begin();
    return max;
  }
}