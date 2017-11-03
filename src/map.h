#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>

#include "spline.h"


class Map {

  protected:
  // define wp spline trajectory
    tk::spline wp_spline_x;
    tk::spline wp_spline_y;
    tk::spline wp_spline_dx;
    tk::spline wp_spline_dy;
    

  public:
    
    Map(){};
    Map(std::string map_file_);
    ~Map() {};
    //gets the cartesian coordinates given frenet coordinates
    std::vector<double> getXY(double s, double d);

};

#endif // ROAD_H
