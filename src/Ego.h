/*
 *  Ego.h
 */

#ifndef EGO_H
#define EGO_H

#include "Vehicle.h"

enum EgoStates {
  KEEP_LANE,
  PREP_LANE_CHANGE_LEFT,
  LANE_CHANGE_LEFT,
  PREP_LANE_CHANGE_RIGHT,
  LANE_CHANGE_RIGHT
};

struct Ego : Vehicle {

  double desired_speed;
  int desired_lane;

  EgoStates state = EgoStates::KEEP_LANE;

};

#endif