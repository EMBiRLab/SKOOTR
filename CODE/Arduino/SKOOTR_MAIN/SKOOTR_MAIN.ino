#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>
#include <Servo.h>
#include <skootr.h>

// MPU variables
/*************************************************************************/
MPU6050 mpu;
// set true if DMP init was successful
bool dmp_ready = false; 
// holds actual interrupt status byte from MPU
uint8_t mpu_interupt_status;  
// return status after each device operation 
// (0 = success, !0 = error)
uint8_t device_status; 
// expected DMP packet size (default is 42 bytes
uint16_t packet_size;    
// count of all bytes currently in FIFO
// Not being used, has been commented out
// uint16_t fifo_count;     
// FIFO storage buffer
uint8_t fifo_buffer[64]; 
// [w, x, y, z]         quaternion container
Quaternion quaternion;           
// [x, y, z]            accel sensor measurements
VectorInt16 acceleration_measured;         
// [x, y, z]            gravity-free accel sensor measurements
// Not being used, has been commented out
// VectorInt16 acceleration_gravity_free; 
// [x, y, z]            world-frame accel sensor measurements
// Not being used, has been commented out
// VectorInt16 aaWorld; 
// [x, y, z]            gravity vector
VectorFloat gravity; 
// [psi, theta, phi]    Euler angle container
// Not being used, has been commented out
// float euler[3];      
// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float rotational_forces[3]; 
float pitch;
float roll;
float yaw;
// indicates whether MPU interrupt pin has gone high
volatile bool mpu_interrupt = false; 
void dmpDataReady() { mpu_interrupt = true; }
/*************************************************************************/

// leg declaration with offsets
/*************************************************************************/
// insert the values you used in servo_init.ino here
const int universal_offset1 = 50;
const int universal_offset2 = 16;

const int initial_x = 0;
const int initial_y = 0;

int leg_one_initial_offset_x =-1;
int leg_one_initial_offset_y =-19;

int leg_two_initial_offset_x =6;
int leg_two_initial_offset_y =-9;

int leg_three_initial_offset_x =6;
int leg_three_initial_offset_y =-7;

joint leg_one_joint_one;
joint leg_one_joint_two;
joint leg_one_joint_three;
joint leg_two_joint_one;
joint leg_two_joint_two;
joint leg_two_joint_three;
joint leg_three_joint_one;
joint leg_three_joint_two;
joint leg_three_joint_three;
// (also from servo_init.ino)
// insert individual joint offsets for each leg here
leg leg1 = {LEG_ONE,
            &leg_one_joint_one,
            &leg_one_joint_two,
            &leg_one_joint_three,
            initial_x,
            initial_y,
            leg_one_initial_offset_x,
            leg_one_initial_offset_y};
leg leg2 = {LEG_TWO,
            &leg_two_joint_one,
            &leg_two_joint_two,
            &leg_two_joint_three,
            initial_x,
            initial_y,
            leg_two_initial_offset_x,
            leg_two_initial_offset_y};
leg leg3 = {LEG_THREE,
            &leg_three_joint_one,
            &leg_three_joint_two,
            &leg_three_joint_three,
            initial_x,
            initial_y,
            leg_three_initial_offset_x,
            leg_three_initial_offset_y};
leg *legs[3] = {&leg1, &leg2, &leg3};
/*************************************************************************/

// user input variables
/*************************************************************************/
char user_input_selection;
char movement_selected = '0';
bool new_user_input = false;
/*************************************************************************/

// timing variables
/*************************************************************************/
// time between inverseKinematics recalculations
unsigned long delta = 0;
unsigned long timeToMove;
int robot_state = 0;
/*************************************************************************/

// inverseKinematics variables
/*************************************************************************/
const float L0 = 126; // link lengths
const float L1 = 136;
const float L2 = 265;
const float d = 97;
const float Ln = 159;        // sqrt(d*d+L0*L0);
const float psi_dif = 37.59; // atan(d/L0)*180/pi;
float L2_offset = 17;        // difference in L2 length between foot modes
/*************************************************************************/

// movement variables
/*************************************************************************/
// can decrease if legs have insufficent traction with ground
const float y_ground = -115;
// speed for pushing motion
float push_speed = 3;
// speed between pushing motions
float prep_speed = 2;
// additional distance in the y direction that the stop
// function digs into the ground
float stop_offset = 2;
// distance from the center of the robot that the leg extends
// during the stand function
int x_stand = 200;
// height that active legs lift above y_ground during pushing functions
int push_retract_height = 22;
// x positions that active legs move between during pushing functions
int walkSPL[2] = {320, 130};
// defines position of inactive legs during pushing functions
int inactive_push_x = 220;
int inactive_push_y = y_ground + 35;
int walkSPL_nolift[2] = {320, 130};
int y_nolift = y_ground;
int inactive_push_x_nolift = 220;
int walkSPL_noball[2] = {220, 80};
int inactive_push_x_noball = 120;
int y_noball = y_ground - 50;
/*************************************************************************/

int stair_height = 70;

// state machines variables
/*************************************************************************/
// for push functions
float x_push = 0;
float y_push = y_ground;
// for push functions
float x_push_inactive = 0;
float y_push_inactive = y_ground;
bool done;
bool first_time;
/*************************************************************************/

void setup() {

  // make sure these values correctly represent the Arduino digital
  // pins each servo control pin is connected to
  leg_one_joint_two.pin = 5;
  leg_one_joint_one.pin = 4;
  leg_one_joint_three.pin = 6;
  leg_two_joint_one.pin = 7;
  leg_two_joint_two.pin = 8;
  leg_two_joint_three.pin = 9;
  leg_three_joint_one.pin = 10;
  leg_three_joint_two.pin = 11;
  leg_three_joint_three.pin = 12;
  /*leg_two_joint_one.pin = 13;  // can be used to disable one or more legs for
  testing leg_two_joint_two.pin = 14; leg_two_joint_three.pin = 15; */

  for (leg *l : legs) {
    l->joint1->servo.attach(l->joint1->pin);
    l->joint2->servo.attach(l->joint2->pin);
    l->joint3->servo.attach(l->joint3->pin);
  }

  for (int i = 0; i < 180; i++)
    stand();

  // ***** start mpu setup *****
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
// join I2C bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.
  Wire.setWireTimeout(3000, true);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  device_status = mpu.dmpInitialize();
  // supply gyro offsets here
  mpu.setXGyroOffset(258);
  mpu.setYGyroOffset(-70);
  mpu.setZGyroOffset(-180);
  mpu.setXAccelOffset(-4207);
  mpu.setYAccelOffset(-2441);
  mpu.setZAccelOffset(529);
  // make sure it workedx
  if (device_status == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // turn on the DMP
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpu_interupt_status = mpu.getIntStatus();
    dmp_ready = true;
    // get expected DMP packet size for later comparison
    packet_size = mpu.dmpGetFIFOPacketSize();
  } else {
    // Serial.println("bad");
  }
  // ***** end mpu setup *****

  delay(100);
  timeToMove = millis();
  Serial.println("BEGIN LOOP");
}

void loop() {
  if (Serial.available() > 0) { // checks for user input
    user_input_selection = Serial.read();
    new_user_input = true;
  }
  if (new_user_input == true) { // collects user input
    Serial.print("Received char input: ");
    Serial.println(user_input_selection);
    movement_selected = user_input_selection;
    robot_state = 0;
    first_time = true;
    new_user_input = false;
  }
  if (millis() - timeToMove >= delta) {
    switch (movement_selected) { // runs movement function corresponding to user
                                 // input
    case 's':
      stair();
      break;
    case 't':
      traverse_cable();
      break;
    case 'n':
      push_leg1_nolift();
      break;
    case 'b':
      push_leg1_noball();
      break;
    case '1':
      push_leg(LEG_ONE);
      break;
    case '2':
      push_leg(LEG_TWO);
      break;
    case '3':
      push_leg(LEG_THREE);
      break;
    case '4':
      stop();
      break;
    case '5':
      stand_incline(20);
      break;
    case '6':
      stop_incline(20);
      break;
    case '7':
      pivot_incline(0, 20);
      break;
    case '8':
      pivot_incline(1, 20);
      break;
    case '9':
      pivot_incline(2, 20);
      break;
    case 'k':
      pivot(2);
      break;
    case 'q':
      leg_up_down();
      break;
    default:
      stand();
      break;
    }
    timeToMove = millis();
  }

  if (!dmp_ready)
    return;
  // calc roll+pitch
  if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
    mpu.dmpGetQuaternion(&quaternion, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &quaternion);
    mpu.dmpGetYawPitchRoll(rotational_forces, &quaternion, &gravity);
    roll = (rotational_forces[2] * 180 / M_PI);
    pitch = (rotational_forces[1] * 180 / M_PI);
    yaw = (rotational_forces[0] * 180 / M_PI);
  }
}

// _________________ FUNCTIONS __________________________________

// runs inverseKinematics function and then uses joint offsets to turn
// inverseKinematics return values into real servo input
void extendLeg(int leg_index, int foot_mode, int x, int y, bool override) {
  leg *l = *(legs + leg_index);
  l->x = x;
  l->y = y;
  q_struct qs = inverseKinematics(leg_index, foot_mode, override);
  float posj1 = qs.q1 + universal_offset1 + l->offset1 + 90 - psi_dif;
  float posj2 = qs.q2 + universal_offset2 + l->offset2;
  if (foot_mode == 0)
    legs[leg_index]->joint3->pos = 55;
  else
    legs[leg_index]->joint3->pos = 0;
  l->joint1->pos = posj1;
  l->joint2->pos = posj2;
}

// ***Note: Several of the Inverse Kinematics variables in this code differ from
// that in our paper, and the correspondance is listed as follows: paper: phi,
// psi, roll, pitch, yaw code:  q0, gamma, yaw, pitch, roll Note also that the
// IMU is oriented differently from the global frame in the paper, and so the
// matrix transforms are performed in a different order resulting in a different
// equation for phi than the equation for q0 in the paper.

// uses goal pose and getPhi() to perform inverse kinematics and solve for our
// two joint positions. Also controls the foot joints given foot_mode
struct q_struct inverseKinematics(int leg_index, int foot_mode, bool override) {
  leg *l = *(legs + leg_index);
  float x = l->x;
  float y = l->y;
  float phi;
  if (override)
    phi = 90;
  else
    phi = getPhi(leg_index, roll, pitch);
  float psi = phi - psi_dif;
  float x1 = x - Ln * cosd(psi);
  float y1 = y - Ln * sind(psi);
  float L2_copy;
  if (foot_mode == 1)
    L2_copy = L2;
  else
    L2_copy = L2 - L2_offset;
  float q2 = acosd(
      (pow(x1, 2) + pow(y1, 2) - pow(L1, 2) - pow(L2_copy, 2)) /
      (2 * L1 * L2_copy)); // sign here is flipped in respect to the paper,
                           // because joint 2 is oriented "backwards"
  float q1 = atan2d(y1, x1) +
             atan2d((L2_copy * sind(q2)), (L1 + L2_copy * cosd(q2))) - psi;
  if (leg_index == 1) {
    /*Serial.print("x: ");Serial.println(x);
    Serial.print("y: ");Serial.println(y);
    Serial.print("phi: ");Serial.println(phi);
    Serial.print("psi: ");Serial.println(psi);
    Serial.print("x1: ");Serial.println(x1);
    Serial.print("y1: ");Serial.println(y1);
    Serial.print("L1: ");Serial.println(L1);
    Serial.print("L2_copy: ");Serial.println(L2_copy);
    Serial.print("q1: ");Serial.println(q1);
    Serial.print("q2: ");Serial.println(q2);*/
  }
  q_struct qs = {q1, q2};
  return qs;
}

// calculates the angle between the vector normal to the plane of movement and
// the vector from the center of the sphere to the origin of a given joint
float getPhi(int leg_index, float roll, float pitch) {
  float mag =
      acosd(cosd(pitch) * cosd(roll) /
            pow(pow(sind(pitch), 2) + pow(cosd(pitch), 2) * pow(sind(roll), 2) +
                    pow(cosd(pitch), 2) * pow(cosd(roll), 2),
                1 / 2)); // this expression differs from the one in the paper
                         // but is the same mathematically
  switch (leg_index) {
  case 0:
    if (roll >= 0)
      return 90 + mag;
    else
      return 90 - mag;
  case 1:
    if (roll >= 0)
      return 90 - mag;
    else
      return 90 + mag;
  default:
    if (roll >= 0)
      return 90 - mag;
    else
      return 90 + mag;
  }
}

// this function is used to transition between movement functions smoothly.
// Returns true when the robot has arrived at its goal configuration
bool transition(int leg_index, int movement_selected) {
  float x_goal;
  float y_goal;
  float footmode;
  bool override;
  if (movement_selected == STAND) {
    x_goal = x_stand;
    y_goal = y_ground;
    footmode = 1;
    override = true;
  } else if (movement_selected == INACTIVE_PUSH) {
    x_goal = inactive_push_x;
    y_goal = inactive_push_y;
    footmode = 1;
    override = true;
  } else if (movement_selected == ACTIVE_PUSH) {
    x_goal = walkSPL[1];
    y_goal = y_ground;
    footmode = 0;
    override = false;
  } else if (movement_selected == STOP) {
    x_goal = x_stand;
    y_goal = y_ground - stop_offset;
    footmode = 0;
    override = true;
  } else if (movement_selected == INACTIVE_PUSH_NO_LIFT) {
    x_goal = inactive_push_x_nolift;
    y_goal = y_nolift;
    footmode = 1;
    override = true;
  } else if (movement_selected == ACTIVE_PUSH_NO_LIFT) {
    x_goal = walkSPL_nolift[1];
    y_goal = y_nolift;
    footmode = 0;
    override = false;
  } else if (movement_selected == INACTIVE_PUSH_NO_BALL) {
    x_goal = inactive_push_x_noball;
    y_goal = y_noball;
    footmode = 1;
    override = true;
  } else if (movement_selected == ACTIVE_PUSH_NO_BALL) {
    x_goal = walkSPL_noball[1];
    y_goal = y_noball;
    footmode = 0;
    override = false;
  }

  float x = legs[leg_index]->x;
  float y = legs[leg_index]->y;
  if (abs(x - x_goal) <= 2 && abs(y - y_goal) <= 2) {
    return true;
  }
  if (x > x_goal) {
    if (y > y_goal)
      extendLeg(leg_index, footmode, x - 1, y - 1, override);
    else if (y < y_goal)
      extendLeg(leg_index, footmode, x - 1, y + 1, override);
    else
      extendLeg(leg_index, footmode, x - 1, y, override);
  } else if (x < x_goal) {
    if (y > y_goal)
      extendLeg(leg_index, footmode, x + 1, y - 1, override);
    else if (y < y_goal)
      extendLeg(leg_index, footmode, x + 1, y + 1, override);
    else
      extendLeg(leg_index, footmode, x + 1, y, override);
  } else {
    if (y > y_goal)
      extendLeg(leg_index, footmode, x, y - 1, override);
    else if (y < y_goal)
      extendLeg(leg_index, footmode, x, y + 1, override);
  }
  return false;
}

void writeAll() { // movement functions set the positions of the joints in the
                  // servo structs, but this function needs to be called to
                  // actually actuate the joints.
  for (leg *l : legs) {
    l->joint1->servo.write(l->joint1->pos);
    l->joint2->servo.write(l->joint2->pos);
    l->joint3->servo.write(l->joint3->pos);
  }
}

float sind(float x) { // helper trig functions:
  return sin(x * PI / 180.0);
}
float cosd(float x) { return cos(x * PI / 180.0); }
float tand(float x) { return tan(x * PI / 180.0); }
float acosd(float x) { return acos(x) * 180.0 / PI; }
float atan2d(float y, float x) { return atan2(y, x) * 180.0 / PI; }

bool stand_check() {
  bool check = true;
  if (!transition(LEG_ONE, STAND))
    check = false;
  if (!transition(LEG_TWO, STAND))
    check = false;
  if (!transition(LEG_THREE, STAND))
    check = false;
  return check;
}

// _________________FLAT MOVEMENT FUNCTIONS __________________________________

void push_leg1_nolift() { // scooting gait for leg 1
  switch (robot_state) {
  case 0:
    x_push = walkSPL_nolift[1];
    done = stand_check();
    if (done) {
      first_time = false;
      robot_state = 1;
    }
    break;
  case 1:
    done = true;
    if (!transition(LEG_ONE, ACTIVE_PUSH_NO_LIFT))
      done = false;
    if (!transition(LEG_TWO, INACTIVE_PUSH_NO_LIFT))
      done = false;
    if (!transition(LEG_THREE, INACTIVE_PUSH_NO_LIFT))
      done = false;
    if (done) {
      robot_state = 2;
      extendLeg((LEG_ONE + 1) % 3, 1, inactive_push_x_nolift, y_nolift, 1);
      extendLeg((LEG_ONE + 2) % 3, 1, inactive_push_x_nolift, y_nolift, 1);
    }
    break;
  case 2:
    extendLeg(LEG_ONE, 0, x_push, y_nolift, 0);
    x_push = x_push + 1;
    if (x_push >= walkSPL_nolift[0])
      robot_state = 3;
    break;
  case 3:
    extendLeg(LEG_ONE, 1, x_push, y_nolift, 0);
    x_push = x_push - 1;
    if (x_push <= walkSPL_nolift[1]) {
      robot_state = 2;
    }
    break;
  }
  writeAll();
}

void push_leg1_noball() { // shuffling gait for leg 1
  switch (robot_state) {
  case 0:
    done = true;
    x_push = walkSPL_noball[1];
    if (!transition(0, 0))
      done = false;
    if (!transition(1, 0))
      done = false;
    if (!transition(2, 0))
      done = false;
    if (done) {
      first_time = false;
      robot_state = 1;
    }
    break;
  case 1:
    done = true;
    if (!transition(0, 6))
      done = false;
    if (!transition(1, 5))
      done = false;
    if (!transition(2, 5))
      done = false;
    if (done) {
      robot_state = 2;
    }
    break;
  case 2:
    extendLeg(0, 0, x_push, y_noball, 0);
    x_push = x_push + 1;
    if (x_push >= walkSPL_noball[0])
      robot_state = 3;
    break;
  case 3:
    extendLeg(0, 1, x_push, y_noball, 0);
    x_push = x_push - 1;
    if (x_push <= walkSPL_noball[1]) {
      x_push = x_push - 5;
      robot_state = 2;
    }
    break;
  }
  writeAll();
}

void push_leg(int leg_index) {
  switch (robot_state) {
  case 0:
    x_push = walkSPL[1];
    y_push = y_ground;
    done = stand_check();
    if (done) {
      first_time = false;
      robot_state = 1;
    }
    break;
  case 1:
    done = true;
    if (!transition(leg_index, ACTIVE_PUSH))
      done = false;
    if (!transition((leg_index + 1) % 3, INACTIVE_PUSH))
      done = false;
    if (!transition((leg_index + 2) % 3, INACTIVE_PUSH))
      done = false;
    if (done) {
      robot_state = 2;
      extendLeg((leg_index + 1) % 3, 1, inactive_push_x, inactive_push_y, 1);
      extendLeg((leg_index + 2) % 3, 1, inactive_push_x, inactive_push_y, 1);
    }
    break;
  case 2:
    extendLeg(leg_index, 0, x_push, y_ground, 0);
    x_push = x_push + push_speed;
    if (x_push >= walkSPL[0])
      robot_state = 3;
    break;
  case 3:
    extendLeg(leg_index, 0, walkSPL[0], y_push, 0);
    y_push = y_push + prep_speed;
    if (y_push >= (y_ground + push_retract_height))
      robot_state = 4;
    break;
  case 4:
    extendLeg(leg_index, 0, x_push, y_ground + push_retract_height, 0);
    x_push = x_push - prep_speed;
    if (x_push <= walkSPL[1])
      robot_state = 5;
    break;
  case 5:
    extendLeg(leg_index, 0, walkSPL[1], y_push, 0);
    y_push = y_push - prep_speed;
    if (y_push <= (y_ground)) {
      robot_state = 2;
    }
    break;
  }
  writeAll();
}

void stand() { // default pose. raises all four feet
  switch (robot_state) {
  case 0:
    done = true;
    if (first_time) {
      for (int i = 0; i < 3; i++) {
        if (!transition(i, STAND))
          done = false;
        ;
      }
      if (done) {
        first_time = false;
        robot_state = 1;
      }
      break;
    } else {
      robot_state = 1;
      break;
    }
  case 1:
    for (int i = 0; i < 3; i++) {
      extendLeg(i, 1, x_stand, y_ground, 1);
    }
  }
  writeAll();
}

void stop() { // lowers all four feet
  switch (robot_state) {
  case 0:
    done = true;
    if (first_time) {
      for (int i = 0; i < 3; i++) {
        if (!transition(i, STOP))
          done = false;
        ;
      }
      if (done) {
        first_time = false;
        robot_state = 1;
      }
      break;
    } else {
      robot_state = 1;
      break;
    }
  case 1:
    for (int i = 0; i < 3; i++) {
      extendLeg(i, 0, x_stand, y_ground - stop_offset, 1);
    }
  }
  writeAll();
}

void pivot(int leg_index) { // pivots about the leg corresponding to index
  switch (robot_state) {
  case 0:
    done = true;
    if (first_time) {
      for (int i = 0; i < 3; i++) {
        if (!transition(i, STAND))
          done = false;
      }
      if (done) {
        first_time = false;
        robot_state = 1;
      }
      break;
    } else {
      robot_state = 1;
      break;
    }
  case 1:
    extendLeg(leg_index, 0, x_stand, y_ground - stop_offset, 1);
    extendLeg((leg_index + 1) % 3, 1, x_stand, y_ground - stop_offset, 1);
    extendLeg((leg_index + 2) % 3, 1, x_stand, y_ground - stop_offset, 1);
  }
  writeAll();
}

// INCLINE FUNCS
float y_to_y_incline(int leg_index, float x, float y,
                     float incline) { // solves for distance in the y direction
                                      // from a given upper robot joint to the
                                      // ground using IMU data and incline angle
  float yaw_leg_specific;
  if (leg_index == 0)
    yaw_leg_specific = yaw;
  else if (leg_index == 1)
    yaw_leg_specific = yaw - 120;
  else
    yaw_leg_specific = yaw + 120;
  float x_projected = x * cosd(yaw_leg_specific);
  float incline_offset = x_projected * tand(incline);
  float y_adjusted = y + incline_offset;
  return y_adjusted;
}

bool transition_incline(
    int leg_index, int movement_selected,
    float incline) { // same as transition but needs to account for incline
                     // using y_to_y_incline
  float x_goal;
  float y_goal;
  float footmode;
  bool override;
  // stand
  if (movement_selected == STAND) {
    x_goal = x_stand;
    y_goal = y_to_y_incline(leg_index, x_goal, y_ground, incline);
    footmode = 1;
    override = true;
  }
  // inactive push, not currently used
  else if (movement_selected == INACTIVE_PUSH) {
    x_goal = inactive_push_x;
    y_goal = y_to_y_incline(leg_index, x_goal, inactive_push_y, incline);
    footmode = 1;
    override = true;
  }
  // active push, not currently used
  else if (movement_selected == ACTIVE_PUSH) {
    x_goal = walkSPL[1];
    y_goal = y_to_y_incline(leg_index, x_goal, y_ground, incline);
    footmode = 0;
    override = false;
  }
  // stop
  else if (movement_selected == STOP) {
    x_goal = x_stand;
    y_goal = y_to_y_incline(leg_index, x_goal, y_ground - stop_offset, incline);
    footmode = 0;
    override = true;
  }

  float x = legs[leg_index]->x;
  float y = legs[leg_index]->y;
  if (abs(x - x_goal) <= 2 && abs(y - y_goal) <= 2) {
    return true;
  }

  if (x > x_goal) {
    if (y > y_goal)
      extendLeg(leg_index, footmode, x - 1, y - 1, override);
    else if (y < y_goal)
      extendLeg(leg_index, footmode, x - 1, y + 1, override);
    else
      extendLeg(leg_index, footmode, x - 1, y, override);
  } else if (x < x_goal) {
    if (y > y_goal)
      extendLeg(leg_index, footmode, x + 1, y - 1, override);
    else if (y < y_goal)
      extendLeg(leg_index, footmode, x + 1, y + 1, override);
    else
      extendLeg(leg_index, footmode, x + 1, y, override);
  } else {
    if (y > y_goal)
      extendLeg(leg_index, footmode, x, y - 1, override);
    else if (y < y_goal)
      extendLeg(leg_index, footmode, x, y + 1, override);
  }

  return false;
}

// all feet up on incline of angle incline.
void stand_incline(float incline) {
  switch (robot_state) {
  case 0:
    done = true;
    if (first_time) {
      for (int i = 0; i < 3; i++) {
        if (!transition_incline(i, 0, incline))
          done = false;
      }
      if (done) {
        first_time = false;
        robot_state = 1;
      }
      break;
    } else {
      robot_state = 1;
      break;
    }
  case 1:
    for (int i = 0; i < 3; i++) {
      extendLeg(i, 1, x_stand, y_to_y_incline(i, x_stand, y_ground, incline),
                1);
    }
  }
  writeAll();
}

// all feet down on incline of angle incline.
void stop_incline(float incline) {
  switch (robot_state) {
  case 0:
    done = true;
    if (first_time) {
      for (int i = 0; i < 3; i++) {
        if (!transition_incline(i, 2, incline))
          done = false;
      }
      if (done) {
        first_time = false;
        robot_state = 1;
      }
      break;
    } else {
      robot_state = 1;
      break;
    }
  case 1:
    for (int i = 0; i < 3; i++) {
      extendLeg(i, 0, x_stand,
                y_to_y_incline(i, x_stand, y_ground - stop_offset, incline), 1);
    }
  }
  writeAll();
}

// pivot about leg corresponding to index while level on incline of angle
// incline.
void pivot_incline(int leg_index, float incline) {
  switch (robot_state) {
  case 0:
    done = true;
    if (first_time) {
      if (!transition_incline(leg_index, 2, incline))
        done = false;
      if (!transition_incline((leg_index + 1) % 3, 0, incline))
        done = false;
      if (!transition_incline((leg_index + 2) % 3, 0, incline))
        done = false;
      if (done) {
        first_time = false;
        robot_state = 1;
      }
      break;
    } else {
      robot_state = 1;
      break;
    }
  case 1:
    // Serial.print("y: ");Serial.println(y_to_y_incline(leg_index, x_stand,
    // y_ground, incline));
    extendLeg(
        leg_index, 0, x_stand,
        y_to_y_incline(leg_index, x_stand, y_ground - stop_offset, incline), 1);
    extendLeg((leg_index + 1) % 3, 1, x_stand,
              y_to_y_incline((leg_index + 1) % 3, x_stand, y_ground, incline),
              1);
    extendLeg((leg_index + 2) % 3, 1, x_stand,
              y_to_y_incline((leg_index + 2) % 3, x_stand, y_ground, incline),
              1);
  }
  writeAll();
}

// uses a series of movements to traverse an extension cable or other small
// obstacle
void traverse_cable() {
  switch (robot_state) {
  case 0:
    done = true;
    x_push = x_stand;
    y_push = y_ground;
    if (!transition(LEG_ONE, STAND))
      done = false;
    if (!transition(LEG_TWO, INACTIVE_PUSH))
      done = false;
    if (!transition(LEG_THREE, INACTIVE_PUSH))
      done = false;
    if (done) {
      first_time = false;
      robot_state = 1;
    }
    break;
  case 1:
    extendLeg(LEG_ONE, 0, x_push, y_push - 12, 0); // robot forward
    x_push = x_push - prep_speed;
    if (x_push <= walkSPL[1]) {
      robot_state = 2;
      extendLeg(LEG_TWO, 0, inactive_push_x, inactive_push_y, 1);
      extendLeg(LEG_THREE, 0, inactive_push_x, inactive_push_y, 1);
    }
    break;
  case 2:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // leg up
    y_push = y_push + prep_speed;
    if (y_push >= y_ground + 40)
      robot_state = 3;
    break;
  case 3:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // leg out
    x_push = x_push + prep_speed;
    if (x_push >= walkSPL[0])
      robot_state = 4;
    break;
  case 4:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // leg down
    y_push = y_push - prep_speed;
    if (y_push <= y_ground - 10)
      robot_state = 5;
    break;
  case 5:
    extendLeg(LEG_ONE, 0, x_push, y_push - 20, 0); // robot forward
    x_push = x_push - prep_speed;
    if (x_push <= walkSPL_noball[0])
      robot_state = 6;
    break;
  case 6:
    done = true;
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // back legs down
    if (!transition(LEG_TWO, INACTIVE_PUSH_NO_BALL))
      done = false;
    if (!transition(LEG_THREE, INACTIVE_PUSH_NO_BALL))
      done = false;
    if (done) {
      robot_state = 7;
      y_push = y_ground;
    }
    break;
  case 7:
    done = true;
    if (!transition(LEG_ONE, ACTIVE_PUSH_NO_BALL)) // front leg down, robot up
      done = false;
    if (done) {
      robot_state = 8;
    }
    break;
  case 8:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // robot forward
    x_push = x_push - prep_speed;
    if (x_push <= 100)
      robot_state = 9;
    break;
  case 9:
    extendLeg(LEG_ONE, 1, x_push, y_push, 0); // leg out
    x_push = x_push + prep_speed;
    if (x_push >= 220)
      robot_state = 10;
    break;
  case 10:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // robot forward
    x_push = x_push - prep_speed;
    if (x_push <= 100)
      robot_state = 11;
    break;
  case 11:
    x_push = walkSPL_noball[1]; // robot down
    done = stand_check();
    if (done) {
      robot_state = 12; // doesn't loop
    }
    break;
  }
  writeAll();
}

// uses a series of movements to traverse an extension cable or other small
// obstacle
void stair() {
  switch (robot_state) {
  case 0:
    done = true;
    x_push = x_stand;
    y_push = y_ground;
    y_push_inactive = inactive_push_y;
    x_push_inactive = inactive_push_x;
    if (!transition(LEG_ONE, STAND))
      done = false;
    if (!transition(LEG_TWO, INACTIVE_PUSH))
      done = false;
    if (!transition(LEG_THREE, INACTIVE_PUSH))
      done = false;
    if (done) {
      first_time = false;
      robot_state = 1;
      delta = 20;
    }
    break;
  case 1:
    extendLeg(LEG_ONE, 0, x_push, y_push - 15, 0); // robot forward
    x_push = x_push - prep_speed;
    if (x_push <= walkSPL[1]) {
      robot_state = 2;
      extendLeg(LEG_TWO, 0, inactive_push_x, inactive_push_y, 1);
      extendLeg(LEG_THREE, 0, inactive_push_x, inactive_push_y, 1);
    }
    break;
  case 2:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // leg up
    y_push = y_push + prep_speed;
    if (y_push >= y_ground + 140)
      robot_state = 3;
    break;
  case 3:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // leg out
    x_push = x_push + prep_speed;
    if (x_push >= walkSPL[0])
      robot_state = 4;
    break;
  case 4:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // leg down
    y_push = y_push - prep_speed;
    if (y_push <= y_ground + stair_height - 25) {
      extendLeg(LEG_TWO, 1, inactive_push_x, inactive_push_y, 1);
      extendLeg(LEG_THREE, 1, inactive_push_x, inactive_push_y, 1);
      robot_state = 5;
    }
    break;
  case 5:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // rob forward
    x_push = x_push - prep_speed;
    if (x_push <= walkSPL[1]) {
      robot_state = 6;
    }
    break;
  case 6:
    extendLeg(LEG_TWO, 1, x_push_inactive, y_push_inactive, 1); // back legs in
    extendLeg(LEG_THREE, 1, x_push_inactive, y_push_inactive, 1);
    x_push_inactive = x_push_inactive - prep_speed;
    if (x_push_inactive <= 130) {
      robot_state = 7;
    }
    break;
  case 7:
    extendLeg(LEG_TWO, 1, x_push_inactive, y_push_inactive,
              1); // back legs down, robot up
    extendLeg(LEG_THREE, 1, x_push_inactive, y_push_inactive, 1);
    y_push_inactive = y_push_inactive - prep_speed;
    if (y_push_inactive <= y_ground - 120) {
      robot_state = 8;
      extendLeg(LEG_TWO, 0, x_push_inactive, y_push_inactive, 1);
      extendLeg(LEG_THREE, 0, x_push_inactive, y_push_inactive, 1);
      y_push = y_ground - 50;
    }
    break;
  case 8:
    extendLeg(LEG_ONE, 1, x_push, y_push + 10, 0); // leg out
    x_push = x_push + prep_speed;
    if (x_push >= 350) {
      robot_state = 9;
      extendLeg(LEG_TWO, 1, x_push_inactive, y_push_inactive, 1);
      extendLeg(LEG_THREE, 1, x_push_inactive, y_push_inactive, 1);
    }
    break;
  case 9:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // rob forward
    x_push = x_push - prep_speed;
    if (x_push <= 100) {
      robot_state = 10;
      extendLeg(LEG_TWO, 0, x_push_inactive, y_push_inactive, 1);
      extendLeg(LEG_THREE, 0, x_push_inactive, y_push_inactive, 1);
      y_push = y_ground;
    }
    break;
  case 10:
    extendLeg(LEG_ONE, 1, x_push, y_push + 5, 0); // leg out
    x_push = x_push + prep_speed;
    if (x_push >= 400) {
      robot_state = 11;
      extendLeg(LEG_ONE, 0, x_push, y_push, 0);
    }
    break;
  case 11:
    extendLeg(LEG_ONE, 0, x_push, y_push + 5, 0); // leg up
    y_push = y_push + prep_speed;
    if (y_push >= y_ground + 50) {
      robot_state = 12;
    }
    break;
  case 12:
    extendLeg(LEG_TWO, 0, x_push_inactive, y_push_inactive, 1); // back legs up
    extendLeg(LEG_THREE, 0, x_push_inactive, y_push_inactive, 1);
    y_push_inactive = y_push_inactive + 1;
    x_push_inactive = 100;
    if (y_push_inactive >= y_ground + 90) {
      robot_state = 13;
    }
    break;
  case 13:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // leg down
    y_push = y_push - 1;
    if (y_push <= y_ground - 20) {
      robot_state = 14;
      delta = 0;
    }
    break;
  case 14:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // rob forward
    x_push = x_push - 1;
    if (x_push <= 100) {
      robot_state = 15;
      delta = 20;
    }
    break;
  }
  writeAll();
}

void leg_up_down() { // uses a series of movements to traverse an extension
                     // cable or other small obstacle
  switch (robot_state) {
  case 0:
    done = true;
    x_push = x_stand;
    y_push = y_ground;
    y_push_inactive = inactive_push_y;
    x_push_inactive = inactive_push_x;
    if (!transition(LEG_ONE, STAND))
      done = false;
    if (!transition(LEG_TWO, INACTIVE_PUSH))
      done = false;
    if (!transition(LEG_THREE, INACTIVE_PUSH))
      done = false;
    if (done) {
      first_time = false;
      robot_state = 1;
      delta = 20;
    }
    break;
  case 1:
    extendLeg(LEG_ONE, 0, x_push, y_push, 0); // robot forward
    y_push = y_push + prep_speed;
    x_push = x_push + prep_speed;
    if (y_push >= 0) {
      robot_state = 2;
    }
    break;
    writeAll();
  }
}