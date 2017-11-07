#include "road.h"

using namespace std;

void Road::update_road(vector<Vehicle> left_lane, vector<Vehicle> center_lane, vector<Vehicle> right_lane){
  this->left_lane = left_lane;
  this->center_lane = center_lane;
  this->right_lane = right_lane;
}

vector<Vehicle> Road::get_lane_status(LANE lane){
  vector<Vehicle> rlane;
  if (lane == LANE::LEFT) {
    rlane = this->left_lane;
  } else if (lane == LANE::CENTER) {
    rlane = this->center_lane;
  } else {
    rlane = this->right_lane;
  }
  return rlane;
}

Vehicle Road::get_radar_lane_status(Vehicle& car, LANE lane){
  vector<Vehicle> currlane= this->get_lane_status(lane);
  double min=10000;
  Vehicle stat(1,10000,10000,10000,10000,10000);
  
  for (int i = 0; i < currlane.size(); i++) {
    double distance = currlane[i].get_s() - car.get_s();
    if (distance<min && distance>0 && distance<RADAR_DISTANCE){
      min=distance;
      double x=currlane[i].get_x();
      double y=currlane[i].get_y();
      double v=currlane[i].get_v();
      double s=currlane[i].get_s();
      double d=currlane[i].get_d();
      double yaw=currlane[i].get_yaw();
      stat.update_vehicle_values(x,y,v,s,d,yaw);
    } 
  }

  return stat;
}

bool Road::safe_lane(Vehicle& car, LANE lane){
  vector<Vehicle> r_car_lane = this->get_lane_status(lane);
  bool safe = true;
  for (int i = 0; i < r_car_lane.size(); i++) {
    double distance = r_car_lane[i].get_s() - car.get_s();
    if(distance > 0 && distance < SAFETY_DISTANCE){
      safe = false;
    }
  }
  return safe;
}

bool Road::free_lane(Vehicle& car, LANE lane){
  vector<Vehicle> target_lane = this->get_lane_status(lane);
  bool is_free = true;
  for (int i = 0; i < target_lane.size(); i++) {
    double distance = std::abs(target_lane[i].get_s() - car.get_s());
    if(distance < GUARD_DISTANCE){
      is_free = false;
    }
  }
  return is_free;
}

LANE Road::lane_change_available(Vehicle& car){
  LANE car_lane = car.lane();
  LANE target_lane = car_lane;

  if (car_lane == LANE::LEFT || car_lane == LANE::RIGHT) {
    if (this->free_lane(car, LANE::CENTER)) {
      target_lane = LANE::CENTER;
    }
  } else {
    if (this->free_lane(car, LANE::LEFT)) {
      target_lane = LANE::LEFT;
    } else if (this->free_lane(car, LANE::RIGHT)) {
      target_lane = LANE::RIGHT;
    }
  }
  return target_lane;
}
