#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "utils.h"
#include "map.h"
#include "road.h"
#include <fstream>

//using namespace std;

class Planner {

protected:

  int n,counter1;
  STATE state;
  vector<double> start_s;
  vector<double> end_s;
  vector<double> start_d;
  vector<double> end_d;
  bool new_points;
  ofstream car_instances;
  

public:
  Planner();
  ~Planner(){};

  vector<double> JMT(vector<double> start, vector <double> end, double T);
  void estimate_new_points(Map& map, vector<vector<double> >& trajectory);
  void create_trajectory(Map& map, Road& road, Vehicle& car, vector<vector<double> >& trajectory);
  void create_trajectoryHH(Map& map, Road& road, Vehicle& car, vector<vector<double> >& trajectory);
  void start_car_instances();
  void car_instances_data(Road& road, Vehicle& car);  

  void set_state(LANE current_lane, LANE target_lane);

  /*  Actions */
  void apply_action(Vehicle& car, LANE current_lane, LANE target_lane);
  void start_car(Vehicle& car);
  void stay_in_lane(Vehicle& car);
  void reduce_speed(Vehicle& car);
  void change_lane(Vehicle& car, LANE target_lane);

};

#endif /* PLANNER_H */
