/*
 *  Vehicle.h
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
using namespace std;

struct Vehicle {

  bool is_valid;

  int id;

  double speed, s;
  int lane;

  double range;

  bool operator < (const Vehicle& str) const {
    return (range < str.range); 
  }

  void print(ostream& os) {
    os << left << setw(10) << setfill(' ') << is_valid
       << left << setw(10) << setfill(' ') << speed
       << left << setw(10) << setfill(' ') << s
       << left << setw(10) << setfill(' ') << lane
       << left << setw(10) << setfill(' ') << range
       << endl;
  }

};

#endif