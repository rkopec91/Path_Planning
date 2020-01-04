#ifndef Car_hpp
#define Car_hpp

#include <vector>
#include <string>

using namespace std;

class Car {
  public:
    int currentLane;
    double currentLeadVehicleSpeed = 22.352 - 0.5;
    double targetVehicleSpeed;
    vector<double> avgScores = {0,0,0};

    int planLane(double s, double d, vector<vector<double>> sensor_fusion);

    int calculateLane(double d);

    vector<double> getClosestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);

  private:
    int calculateScore(double s, int lane, vector<vector<double>> sensor_fusion);
};

#endif