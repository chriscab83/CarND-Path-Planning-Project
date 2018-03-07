/*
 *  Vehicle.h
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
using namespace std;

struct Vehicle {

  double speed, s;
  int lane;

  double range_to_ego, speed_diff_ego;


  bool operator < (const Vehicle& str) const {
    return (range_to_ego < str.range_to_ego); 
  }

};

#endif