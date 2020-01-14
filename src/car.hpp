#ifndef Car_hpp
#define Car_hpp

#include <vector>
#include <string>

using namespace std;

class Car {
  public:
    int currentLane;
    double current_lead_speed = 22.352 - 0.5;
    double target_speed;
    int carlane=0;
    vector<double> average_scores = {0,0,0};

    int planLane(double s, double d, vector<vector<double>> sensor_fusion);

    vector<double> getClosestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);

  private:
    int calculateScore(double s, int lane, vector<vector<double>> sensor_fusion);
};

#endif