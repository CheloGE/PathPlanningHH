#include "planner.h"

using namespace std;
//returns the lane as String for printing
string slane(LANE lane){ 
  if (lane == LANE::LEFT){
    return "LEFT";
  } else if(lane == LANE::CENTER){
    return "CENTER";
  } else{
    return "RIGHT";
  }
}

string sstate(STATE state){
  if (state == STATE::KEEP_LANE){
    return "KEEP_LANE";
  } else if(state == STATE::CHANGE_LEFT){
    return "CHANGE_LEFT";
  } else if(state == STATE::START){
    return "START";
  } else{
    return "CHANGE_RIGHT";
  }
}

LANE get_lane(double d){
  LANE lane;
  if (d < 4.0) {
    lane = LANE::LEFT;
  }
  else if ((d >= 4.0) && (d < 8.0)) {
    lane = LANE::CENTER;
  }
  else {
    lane = LANE::RIGHT;
  }
  return lane;
}

double get_lane_d(LANE lane){
  double d;
  if (lane == LANE::LEFT) {
    d = 2.0;
  }
  else if (lane == LANE::CENTER) {
    d = 6.0;
  }
  else {
    d = 10.0;
  }
  return d;
}

double get_lane_d(double D){
  double d;
  if (D < 4.0) {
    d = 2.0;
  }
  else if ((D >= 4.0) && (D < 8.0)) {
    d = 6.0;
  }
  else {
    d = 10.0;
  }
  return d;
}
/*planner starts here*/
Planner::Planner(){
  this->state = STATE::START;
}

/* JMT*/
vector<double> Planner::JMT(vector<double> start, vector <double> end, double T) {
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  */

  // prepare matrix A with coefficents
  Eigen::MatrixXd A(3,3);
  Eigen::MatrixXd B(3,1);

  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;

  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];

  Eigen::MatrixXd Ai = A.inverse();
  Eigen::MatrixXd C = Ai*B;

  return {start[0], start[1], .5*start[2], C.data()[0], C.data()[1], C.data()[2]};
}

void Planner::estimate_new_points(Map& map, vector<vector<double> >& trajectory){

  // jmt
  double T = this->n * AT;
  vector<double> poly_s = this->JMT(this->start_s, this->end_s, T);
  vector<double> poly_d = this->JMT(this->start_d, this->end_d, T);

  double t, next_s, next_d, mod_s, mod_d;
  vector <double> XY;
  for(int i = 0; i < n; i++) {

    t = AT*i; //time for each point from t=0

    // /* JMT */
    // cout << "----------JMT----------" << endl;
    // cout << "t= " << t << endl;

    next_s = 0.0;
    next_d = 0.0;
    for (int a = 0; a < poly_s.size(); a++) {
      next_s += poly_s[a] * pow(t, a); //completes the polynomial aprox by using both coefficients of ...
      next_d += poly_d[a] * pow(t, a); //JMT and the time of each point for getting a smooth path
    }
    mod_s = fmod(next_s, TRACK_DISTANCE); //gives me a value between 0 and the track distance
    mod_d = fmod(next_d, ROAD_WIDTH); //gives me a value between 0 and the road width

    XY = map.getXY(mod_s, mod_d); //gives me the values in cartesian coordinates \
                                    equivalent to the input frenet coordinates

    trajectory[0].push_back(XY[0]);
    trajectory[1].push_back(XY[1]);
  }

}

void Planner::create_trajectory(Map& map, Road& road, Vehicle& car, vector<vector<double> >& trajectory) {

  cout << "STATE: " << sstate(this->state) << endl;
  cout << "LANE: " << slane(car.lane()) << endl;

  int current_points = trajectory[0].size();
  this->new_points = false;

  if (current_points < POINTS) { //update new points around every 50 points left which in practice means\
                                  that it updates every second 50*0.02=1seg 
    this->new_points = true;

    // first trajectory
    if (this->state == STATE::START) {

      this->start_car(car);
      this->start_car_instances();
    } else if (this->state == STATE::KEEP_LANE) {
      this->car_instances_data(road,car); //create instances for car
      
      // FREE LANE
      if (road.safe_lane(car, car.lane())) {
        this->stay_in_lane(car);
      // LANE CHANGE NEEDED
      } else {
        LANE target_lane = road.lane_change_available(car); //throws best target lane possible in the\
                                                              current lane giving first priority to the\
                                                               left lane and then to the right.
        if (target_lane == car.lane()){
          // not possible -> reduce speed
          this->reduce_speed(car);
        } else {
          this->change_lane(car, target_lane);
        }
      }
    } else {
      this->car_instances_data(road,car); //create instances for car
      
      LANE new_lane = get_lane(car.prev_d()[0]);
      if(road.safe_lane(car, new_lane)){
        this->stay_in_lane(car);
      } else {
        // not possible -> reduce speed
        this->reduce_speed(car);
      }
    }
  }

  // have we generated new points?
  if (this->new_points) {
    this->estimate_new_points(map, trajectory); //estimates new points based on this->start_s,\
                                                   this->end_s, this->start_d,this->end_d  
  }

}

/* FSM */
void Planner::set_state(LANE current_lane, LANE target_lane){
  if (current_lane == target_lane){
    this->state = STATE::KEEP_LANE;
  } else {
    // not equal
    if(current_lane == LANE::LEFT){
      this->state = STATE::CHANGE_RIGHT;
    } else if(current_lane == LANE::RIGHT){
      this->state = STATE::CHANGE_LEFT;
    } else {
      if(target_lane == LANE::LEFT){
        this->state = STATE::CHANGE_LEFT;
      } else {
        this->state = STATE::CHANGE_RIGHT;
      }
    }
  }
}

/* APPLY ACTION */
void Planner::apply_action(Vehicle& car, LANE current_lane, LANE target_lane){
  car.set_previous_s(this->end_s);
  car.set_previous_d(this->end_d);
  this->set_state(current_lane, target_lane);
}

/* ACTIONS */
void Planner::start_car(Vehicle& car){
  cout << "ACTION: start_car" << endl;
  this->n = CYCLES*POINTS; // 4 cycles to start
  double target_v = SPEED_LIMIT;//SPEED_LIMIT*0.5; //desired velocity in m/s
  double target_s = car.get_s() + n * AT * target_v; // desired position in frenet coordinates

  this->start_s = {car.get_s(), car.get_v(), 0.0}; //vector with current position and velocity 
  this->end_s= {target_s, target_v, 0.0}; //vector with desired position and velocity

  this->start_d = {get_lane_d(car.lane()), 0.0, 0.0}; //vector with current lane
  this->end_d = {get_lane_d(car.lane()), 0.0, 0.0}; //vector with desired lane

  this->apply_action(car, car.lane(), car.lane());
}

void Planner::stay_in_lane(Vehicle& car){
  cout << "ACTION: stay_in_lane" << endl;
  this->n = CYCLES*POINTS;
  double target_v = min(car.prev_s()[1]*1.3, SPEED_LIMIT);
  double target_s = car.prev_s()[0] + n * AT * target_v;

  this->start_s = {car.prev_s()[0], car.prev_s()[1], car.prev_s()[2]};
  this->end_s = {target_s, target_v, 0.0};

  double target_d = get_lane_d(car.prev_d()[0]);

  this->start_d = {get_lane_d(car.prev_d()[0]), 0.0, 0.0};
  this->end_d = {target_d, 0.0, 0.0};

  this->apply_action(car, get_lane(car.prev_d()[0]), get_lane(car.prev_d()[0]));
}

void Planner::reduce_speed(Vehicle& car){
  cout << "ACTION: reduce_speed" << endl;
  this->n = CYCLES*POINTS;
  this->new_points = true;
  double target_v = max(car.prev_s()[1]*0.8, SPEED_LIMIT/2);
  double target_s = car.prev_s()[0] + n * AT * target_v;

  this->start_s = {car.prev_s()[0], car.prev_s()[1], car.prev_s()[2]};
  this->end_s = {target_s, target_v, 0.0};

  double target_d = get_lane_d(car.prev_d()[0]);

  this->start_d = {get_lane_d(car.prev_d()[0]), 0.0, 0.0};
  this->end_d = {target_d, 0.0, 0.0};

  this->apply_action(car, get_lane(car.prev_d()[0]), get_lane(car.prev_d()[0]));
}

void Planner::change_lane(Vehicle& car, LANE target_lane){
  cout << "ACTION: reduce_speed" << endl;
  this->n = CYCLES*POINTS;
  this->new_points = true;
  double target_v = car.prev_s()[1];
  double target_s = car.prev_s()[0] + n * AT * target_v;

  this->start_s = {car.prev_s()[0], car.prev_s()[1], car.prev_s()[2]};
  this->end_s = {target_s, target_v, 0.0};

  double target_d = get_lane_d(target_lane);

  this->start_d = {get_lane_d(car.prev_d()[0]), 0.0, 0.0};
  this->end_d = {target_d, 0.0, 0.0};

  this->apply_action(car, get_lane(car.prev_d()[0]), get_lane(target_d));
}

void Planner::create_trajectoryHH(Map& map, Road& road, Vehicle& car, vector<vector<double>>& trajectory){
  
  cout << "STATE: " << sstate(this->state) << endl;
  cout << "LANE: " << slane(car.lane()) << endl;
  cout << "counter" << this->counter1 << endl;
  cout << "distance next car in left" << road.get_distance_to_next_vehicle_in_lane(car,LANE::LEFT) << endl;
  cout << "distance rear car in left" << road.get_distance_to_the_rear_vehicle_in_lane(car,LANE::LEFT) << endl;
  cout << "distance next car in center" << road.get_distance_to_next_vehicle_in_lane(car,LANE::CENTER) << endl;
  cout << "distance rear car in center" << road.get_distance_to_the_rear_vehicle_in_lane(car,LANE::CENTER) << endl;  
  cout << "distance next car in right" << road.get_distance_to_next_vehicle_in_lane(car,LANE::RIGHT) << endl;
  cout << "distance rear car in right" << road.get_distance_to_the_rear_vehicle_in_lane(car,LANE::RIGHT) << endl;
  cout << "vel next car in left" << road.get_radar_lane_status_front(car,LANE::LEFT).get_v() << endl;
  cout << "vel next car in center" << road.get_radar_lane_status_front(car,LANE::CENTER).get_v() << endl;
  cout << "vel next car in right" << road.get_radar_lane_status_front(car,LANE::RIGHT).get_v() << endl;

  

  int current_points = trajectory[0].size();
  this->new_points = false;

  if (current_points < POINTS) { //update new points around every 50 points left which in practice means\
                                  that it updates every second 50*0.02=1seg 
    this->new_points = true;
    
    if (this->state == STATE::START) {
      this->start_car(car);
      this->counter1=0; //just in case I need it later
      this->start_car_instances(); //creates the header for the instances txt

    }
    else{
    
    //this->change_lane(car, LANE::LEFT); //changes to the left line
    //this->change_lane(car, LANE::RIGHT); //changes to the right line
    this->change_lane(car, LANE::CENTER); //changes to the center line
    this->car_instances_data(road,car); //create instances for car
    
    
    }
  }
  
  
  // have we generated new points?
  if (this->new_points) {
    this->estimate_new_points(map, trajectory); //estimates new points based on this->start_s,\
                                                   this->end_s, this->start_d,this->end_d  
  }

}

void Planner::start_car_instances(){
  car_instances.open("car_instances.txt",ios::trunc);
  if (car_instances.is_open()){
    car_instances <<"distFront_L "<< "distRear_L "<<"VFront_L "<<"VRear_L " \
    <<"distFront_C "<<"distRear_C "<<"VFront_C "<<"VRear_C "\
    <<"distFront_R "<<"distRear_R "<<"VFront_R "<<"VRear_R "\
    <<"H1 "<<"H2 "<<"H3 "<<endl;
    car_instances.close();
  }
  else{
    cout << "car_instances.txt couldn't be initialized" <<endl;
  }
}

void Planner::car_instances_data(Road& road, Vehicle& car){
  car_instances.open("car_instances.txt",ios::app);
  if (car_instances.is_open()){
    car_instances << road.get_distance_to_next_vehicle_in_lane(car,LANE::LEFT)\
    <<" "<< road.get_distance_to_the_rear_vehicle_in_lane(car,LANE::LEFT)\
    <<" "<< road.get_radar_lane_status_front(car,LANE::LEFT).get_v()\
    <<" "<< road.get_radar_lane_status_rear(car,LANE::LEFT).get_v()\
    <<" "<< road.get_distance_to_next_vehicle_in_lane(car,LANE::CENTER)\
    <<" "<< road.get_distance_to_the_rear_vehicle_in_lane(car,LANE::CENTER)
    <<" "<< road.get_radar_lane_status_front(car,LANE::CENTER).get_v()\
    <<" "<< road.get_radar_lane_status_rear(car,LANE::CENTER).get_v()\
    <<" "<< road.get_distance_to_next_vehicle_in_lane(car,LANE::RIGHT)\
    <<" "<< road.get_distance_to_the_rear_vehicle_in_lane(car,LANE::RIGHT)\
    <<" "<< road.get_radar_lane_status_front(car,LANE::RIGHT).get_v()\
    <<" "<< road.get_radar_lane_status_rear(car,LANE::RIGHT).get_v()<<endl;
    car_instances.close();
  }
  else{
    cout << "car_instances.txt couldn't be opened" <<endl;
  }
}  

  