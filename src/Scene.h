/*
 *  Scene.h
 */

#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <iostream>
#include "Vehicle.h"

using namespace std;

struct Scene {

  vector<Vehicle> forward[3];
  vector<Vehicle> behind[3];

  void add_vehicle(Vehicle v) {
    if (v.range_to_ego >= 0) {
      auto it = forward[v.lane].begin();
      for (; it < forward[v.lane].end(); ++it) {
        if (v.range_to_ego < (*it).range_to_ego) break;
      }
      forward[v.lane].insert(it, v);
    }
    else {
      auto it = behind[v.lane].begin();
      for (; it < behind[v.lane].end(); ++it) {
        if (v.range_to_ego < (*it).range_to_ego) break;
      }
      behind[v.lane].insert(it, v);
    }
  }

  const int WIDTH = 10;
  void print_lane(ostream& os, int lane) {
    for (int i = 0; i < behind[lane].size(); ++i) {
      os << left << setw(WIDTH) << setfill(' ') << behind[lane][i].range_to_ego;
    }

    os << left << setw(WIDTH) << setfill(' ') << 'E';

    for (int i = 0; i < forward[lane].size(); ++i) {
      os << left << setw(WIDTH) << setfill(' ') << forward[lane][i].range_to_ego;
    }

    os << '\n';
  }

};

#endif