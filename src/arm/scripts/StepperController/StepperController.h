#ifndef MQP_STEPPER_CONTROLLER_H_
#define MQP_STEPPER_CONTROLLER_H_

const float MAX_SPEED = 600;    // todo - verify constants
const float MIN_SPEED = 100;    // todo: does this affect going in the negative direction?
const int ACCELERATION = 500;
const int X_MAX_POS = 1500;
const int X_MIN_POS = 0;
const int Y_MAX_POS = 750;
const int Y_MIN_POS = 0;

enum robot_states{
  X_MIN_LS_HIT,
  X_MAX_LS_HIT,
  Y_MIN_LS_HIT,
  Y_MAX_LS_HIT,
  WAITING_TO_START,
  HOMING,
  WAITING_FOR_INSTRUCTION,
  MOVING
};


#endif
