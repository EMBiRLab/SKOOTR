#ifndef SKOOTR_H
#define SKOOTR_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>
#include <Servo.h>

#define PI 3.14159265

#define INTERRUPT_PIN 2

// LEG Definitions
#define LEG_ONE 0
#define LEG_TWO 1
#define LEG_THREE 2

typedef enum {
  STAND = 0,
  INACTIVE_PUSH = -1,
  ACTIVE_PUSH = 1,
  STOP = 2,
  INACTIVE_PUSH_NO_LIFT = 3,
  ACTIVE_PUSH_NO_LIFT = 4,
  INACTIVE_PUSH_NO_BALL = 5,
  ACTIVE_PUSH_NO_BALL = 6
} robot_state;

struct leg {
  int index;
  joint *joint1;
  joint *joint2;
  joint *joint3;
  float x;
  float y;
  int offset1;
  int offset2;
};

struct q_struct {
  int q1;
  int q2;
};

struct joint {
  Servo servo;
  int pin;
  float pos = 0;
};

typedef enum { FOOT_STUMP = 0, FOOT_ROLLER = 1 } foot_mode;

#endif