#ifndef Car_hpp
#define Car_hpp

#include <vector>
#include <string>

using namespace std;

class Car {
  public:
    double lead_vehicle_speed = 22.302;
    int current_lane;
    double target_speed;
    vector<double> average_scores = {0,0,0};

    int planLaneTransition(double s, double d, vector<vector<double>> sensor_fusion);
    int getLane(double d);

    vector<double> getClosestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);
    
  private:
    int calculateLaneScore(double s, int lane, vector<vector<double>> sensor_fusion);
    void updateAverageScores(int i, vector <double> scores);
    int compareLanes(int lane);
};

#endif