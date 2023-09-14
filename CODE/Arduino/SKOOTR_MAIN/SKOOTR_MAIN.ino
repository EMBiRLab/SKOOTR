#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>

#define pi 3.14159265
// MPU vars
MPU6050 mpu;
#define INTERRUPT_PIN 2
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
float pitch;
float roll;
float yaw;
//end MPU vars

//various structs for organization
struct joint{
  Servo servo;
  int pin;
  float pos = 0;
};

struct leg{
  int index;
  joint * joint1;
  joint * joint2;
  joint * joint3;
  float x;
  float y;
  int offset1;
  int offset2;
};

struct q_struct{
  int q1;
  int q2;
};

// leg declaration w/ offsets
const int universaloffset1 = 50; // insert the values you used in servo_init.ino here
const int universaloffset2 = 16; 
joint j11;joint j12; joint j13; joint j21; joint j22;joint j23; joint j31; joint j32; joint j33;
leg leg1 = {0,&j11,&j12,&j13,0,0,-1,-19}; // insert individual joint offsets for each leg here
leg leg2 = {1,&j21,&j22,&j23,0,0,6,-9};   // (also from servo_init.ino)
leg leg3 = {2,&j31,&j32,&j33,0,0,6,-7};
leg * legs[3] = {&leg1,&leg2,&leg3};

//user input vars
char temp;
char which_func = '0';
bool newData = false;

//timing vars
unsigned long delta = 10; // time between IK recalculations
unsigned long timeToMove;
int c = 0;

//IK vars
const float L0 = 126; // link lengths
const float L1 = 136;
const float L2 = 265; 
const float d = 97;
const float Ln = 159; //sqrt(d*d+L0*L0);
const float psi_dif = 37.59; //atan(d/L0)*180/pi;
float L2_offset = 17; // difference in L2 length between foot modes

//movement vars
const float y_ground = -112; // can decrease if legs have insufficent traction with ground
float push_speed = 3; // speed for pushing motion
float prep_speed = 2; // speed between pushing motions
float stop_offset = 2; // additional distance in the y direction that the stop function digs into the ground
int x_stand = 200; // distance from the center of the robot that the leg extends during the stand function
int push_retract_height = 22; // height that active legs lift above y_ground during pushing functions
int walkSPL[2] = {320,130}; // x positions that active legs move between during pushing functions
int inactive_push_x = 220; // defines position of inactive legs during pushing functions
int inactive_push_y = y_ground + 35;
int walkSPL_nolift[2] = {320,130}; 
int y_nolift = y_ground;
int inactive_push_x_nolift = 220; 
int walkSPL_noball[2] = {220,80}; 
int inactive_push_x_noball = 120;
int y_noball = y_ground-50;

//state machines vars
float x_push = 0; // for push functions
float y_push = y_ground;
float x_push_inactive = 0; // for push functions
float y_push_inactive = y_ground;
bool done;
bool first_time;

void setup() {

  j11.pin = 4;  // make sure these values correctly represent the Arduino digital pins each servo control pin is connected to
  j12.pin = 5; 
  j13.pin = 6; 
  j21.pin = 7;
  j22.pin = 8;
  j23.pin = 9; 
  j31.pin = 10;
  j32.pin = 11;
  j33.pin = 12;
  /*j21.pin = 13;  // can be used to disable one or more legs for testing
  j22.pin = 14; 
  j23.pin = 15; */

  for (leg * l: legs){
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
    Wire.setWireTimeout(3000,true);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply gyro offsets here
  mpu.setXGyroOffset(258);
  mpu.setYGyroOffset(-70);
  mpu.setZGyroOffset(-180);
  mpu.setXAccelOffset(-4207);
  mpu.setYAccelOffset(-2441);
  mpu.setZAccelOffset(529);
  // make sure it workedx
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // turn on the DMP
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    //Serial.println("bad");
  }
  // ***** end mpu setup *****

  delay(100);
  timeToMove = millis();
  Serial.println("BEGIN LOOP");
}

void loop() {
  if (Serial.available() > 0) { // checks for user input
     temp = Serial.read();
     newData = true;
  }
  if (newData == true) { // collects user input
        Serial.print("Received char input: ");Serial.println(temp);
        which_func = temp;
        c = 0;
        first_time = true;
        newData = false;
  }
  if (millis() - timeToMove >= delta){
    switch (which_func){ // runs movement function corresponding to user input
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
        push_leg1();
        break;
      case '2':
        push_leg2();
        break;
      case '3':
        push_leg3();
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
        pivot_incline(0,20);
        break;
      case '8':
        pivot_incline(1,20);
        break;
      case '9':
        pivot_incline(2,20);
        break;
      default:
        stand();
        break;
    }
    timeToMove = millis();
    //Serial.print("yaw: ");Serial.println(yaw);//Serial.print(" pitch: ");Serial.println(pitch);
  }

  if (!dmpReady) return;
  //calc roll+pitch
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll = (ypr[2] * 180/M_PI);
    pitch = (ypr[1] * 180/M_PI);
    yaw = (ypr[0] * 180/M_PI);
   }
}

// _________________ FUNCTIONS __________________________________

void extendLeg(int index, int foot_mode, int x, int y, bool override){ // runs IK function and then uses joint offsets to turn IK return values into real servo input
  leg * l = *(legs+index);
  l->x = x;
  l->y = y;
  q_struct qs = IK(index, foot_mode, override);
  float posj1 = qs.q1+universaloffset1+l->offset1 + 90 - psi_dif;
  float posj2 = qs.q2+universaloffset2+l->offset2;
  if (foot_mode == 0)
    legs[index]->joint3->pos = 55;
  else
    legs[index]->joint3->pos = 0;
  l->joint1->pos = posj1;
  l->joint2->pos = posj2;
  //Serial.print("x: ");Serial.print(x);Serial.print(", y: ");Serial.print(y);Serial.print(", q1: ");Serial.print(qs.q1);Serial.print(", q2: ");Serial.println(qs.q2);
}

// ***Note: Several of the Inverse Kinematics variables in this code differ from that in our paper, and the correspondance is listed as follows:
// paper: phi, psi, roll, pitch, yaw
// code:  q0, gamma, yaw, pitch, roll
// Note also that the IMU is oriented differently from the global frame in the paper, 
// and so the matrix transforms are performed in a different order resulting in a different equation for phi than the equation for q0 in the paper.


struct q_struct IK(int index, int foot_mode, bool override){ // uses goal pose and getPhi() to perform inverse kinematics and solve for our two joint positions. Also controls the foot joints given foot_mode
  leg * l = *(legs+index);
  float x = l->x;
  float y = l->y;
  float phi;
  if (override)
    phi = 90;
  else
    phi = getPhi(index, roll, pitch);
  float psi = phi - psi_dif;
  float x1 = x - Ln*cosd(psi);
  float y1 = y - Ln*sind(psi);
  float L2_copy;
  if (foot_mode == 1)
    L2_copy = L2;
  else
    L2_copy = L2 - L2_offset;
  float q2 = acosd((pow(x1,2) + pow(y1,2) - pow(L1,2) - pow(L2_copy,2))/(2*L1*L2_copy)); // sign here is flipped in respect to the paper, because joint 2 is oriented "backwards"
  float q1 = atan2d(y1,x1) + atan2d((L2_copy*sind(q2)),(L1+L2_copy*cosd(q2))) - psi;
  if (index == 1){
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
  q_struct qs = {q1,q2};
  return qs;
}

float getPhi(int index, float roll, float pitch){ // calculates the angle between the vector normal to the plane of movement and the vector from the center of the sphere to the origin of a given joint
  float mag = acosd(cosd(pitch)*cosd(roll)/pow(pow(sind(pitch),2)+pow(cosd(pitch),2)*pow(sind(roll),2)+pow(cosd(pitch),2)*pow(cosd(roll),2),1/2)); // this expression differs from the one in the paper but is the same mathematically
  switch (index){
    case 0:
      if (roll>=0)
        return 90+mag;
      else
        return 90-mag;
    case 1:
      if (roll>=0)
        return 90-mag;
      else
        return 90+mag;
    default:
      if (roll>=0)
        return 90-mag;
      else
        return 90+mag;
  }
}

bool transition(int index, int which_func){ // this function is used to transition between movement functions smoothly. Returns true when the robot has arrived at its goal configuration
  float x_goal;
  float y_goal;
  float footmode;
  bool override;
  if (which_func == 0){ // stand
    x_goal = x_stand;
    y_goal = y_ground;
    footmode = 1;
    override = true;
  }
  else if (which_func == -1){ // inactive push
    x_goal = inactive_push_x;
    y_goal = inactive_push_y;
    footmode = 1;
    override = true;
  }
  else if (which_func == 1){ // active push
    x_goal = walkSPL[1];
    y_goal = y_ground;
    footmode = 0;
    override = false;
  }
  else if (which_func == 2){ // stop
    x_goal = x_stand;
    y_goal = y_ground-stop_offset;
    footmode = 0;
    override = true;
  }
  else if (which_func == 3){ // inactive push no lift
    x_goal = inactive_push_x_nolift;
    y_goal = y_nolift;
    footmode = 1;
    override = true;
  }
  else if (which_func == 4){ // active push no lift
    x_goal = walkSPL_nolift[1];
    y_goal = y_nolift;
    footmode = 0;
    override = false;
  }
  else if (which_func == 5){ // inactive push no ball
    x_goal = inactive_push_x_noball;
    y_goal = y_noball;
    footmode = 1;
    override = true;
  }
  else if (which_func == 6){ // active push no ball
    x_goal = walkSPL_noball[1];
    y_goal = y_noball;
    footmode = 0;
    override = false;
  }

  float x = legs[index]->x;
  float y = legs[index]->y;
  if (abs(x-x_goal) <= 2 && abs(y-y_goal) <= 2){
    return true;
  }
  if (x > x_goal){
    if (y > y_goal)
      extendLeg(index, footmode, x-1, y-1, override);
    else if (y < y_goal)
      extendLeg(index, footmode, x-1, y+1, override);
    else
      extendLeg(index, footmode, x-1, y, override);
  }
  else if (x < x_goal){
    if (y > y_goal)
      extendLeg(index, footmode, x+1, y-1, override);
    else if (y < y_goal)
      extendLeg(index, footmode, x+1, y+1, override);
    else
      extendLeg(index, footmode, x+1, y, override);
  }
  else{
    if (y > y_goal)
      extendLeg(index, footmode, x, y-1, override);
    else if (y < y_goal)
      extendLeg(index, footmode, x, y+1, override);
  }
  return false;
}

void writeAll(){ // movement functions set the positions of the joints in the servo structs, but this function needs to be called to actually actuate the joints.
  for (leg * l: legs){
    l->joint1->servo.write(l->joint1->pos);
    l->joint2->servo.write(l->joint2->pos); 
    l->joint3->servo.write(l->joint3->pos);
  }
}

float sind(float x){  // helper trig functions:
  return sin(x*pi/180.0);
}
float cosd(float x){
  return cos(x*pi/180.0);
}
float tand(float x){
  return tan(x*pi/180.0);
}
float acosd(float x){
  return acos(x)*180.0/pi;
}
float atan2d(float y, float x){
  return atan2(y,x)*180.0/pi;
}

// _________________FLAT MOVEMENT FUNCTIONS __________________________________

void push_leg1_nolift(){ // scooting gait for leg 1
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL_nolift[1];
      if (!transition(0,0))
        done = false;
      if (!transition(1,0))
        done = false;
      if (!transition(2,0))
        done = false;
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      done = true;
      if (!transition(0,4))
        done = false;
      if (!transition(1,3))
        done = false;
      if (!transition(2,3))
        done = false;
      if (done){
        c = 2;
        extendLeg(1, 1, inactive_push_x_nolift, y_nolift, 1);
        extendLeg(2, 1, inactive_push_x_nolift, y_nolift, 1);
      }
      break;
    case 2:
      extendLeg(0,0,x_push,y_nolift,0);
      x_push = x_push + 1;
      if (x_push>=walkSPL_nolift[0])
        c=3;
      break;
    case 3:
      extendLeg(0,1,x_push,y_nolift,0);
      x_push = x_push - 1;
      if (x_push<=walkSPL_nolift[1]){
        c=2;
      }
      break;
  }
  writeAll();
}

void push_leg1_noball(){ // shuffling gait for leg 1
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL_noball[1];
      if (!transition(0,0))
        done = false;
      if (!transition(1,0))
        done = false;
      if (!transition(2,0))
        done = false;
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      done = true;
      if (!transition(0,6))
        done = false;
      if (!transition(1,5))
        done = false;
      if (!transition(2,5))
        done = false;
      if (done){
        c = 2;
      }
      break;
    case 2:
      extendLeg(0,0,x_push,y_noball,0);
      x_push = x_push + 1;
      if (x_push>=walkSPL_noball[0])
        c=3;
      break;
    case 3:
      extendLeg(0,1,x_push,y_noball,0);
      x_push = x_push - 1;
      if (x_push<=walkSPL_noball[1]){
        x_push = x_push - 5;
        c=2;
      }
      break;
  }
  writeAll();
}

void push_leg1(){ // skating gait for leg 1
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL[1];
      y_push = y_ground;
      if (!transition(0,0))
        done = false;
      if (!transition(1,0))
        done = false;
      if (!transition(2,0))
        done = false;
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      done = true;
      if (!transition(0,1))
        done = false;
      if (!transition(1,-1))
        done = false;
      if (!transition(2,-1))
        done = false;
      if (done){
        c = 2;
        extendLeg(1, 1, inactive_push_x, inactive_push_y, 1);
        extendLeg(2, 1, inactive_push_x, inactive_push_y, 1);
      }
      break;
    case 2:
      extendLeg(0,0,x_push,y_ground,0);
      x_push = x_push + push_speed;
      if (x_push>=walkSPL[0])
        c=3;
      break;
    case 3:
      extendLeg(0,0,walkSPL[0],y_push,0);
      y_push = y_push + prep_speed;
      if (y_push>=(y_ground+push_retract_height))
        c=4;
      break;
    case 4:
      extendLeg(0,0,x_push,y_ground+push_retract_height,0);
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1])
        c=5;
      break;
    case 5:
      extendLeg(0,0,walkSPL[1],y_push,0);
      y_push = y_push - prep_speed;
      if (y_push<=(y_ground)){
        c=2;
      }
      break;
  }
  writeAll();
}

void push_leg2(){ //skating gait for leg 2
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL[1];
      y_push = y_ground;
      if (!transition(0,0))
        done = false;
      if (!transition(1,0))
        done = false;
      if (!transition(2,0))
        done = false;
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      done = true;
      if (!transition(1,1))
        done = false;
      if (!transition(0,-1))
        done = false;
      if (!transition(2,-1))
        done = false;
      if (done){
        c = 2;
        extendLeg(0, 1, inactive_push_x, inactive_push_y, 1);
        extendLeg(2, 1, inactive_push_x, inactive_push_y, 1);
      }
      break;
    case 2:
      extendLeg(1,0,x_push,y_ground,0);
      x_push = x_push + push_speed;
      if (x_push>=walkSPL[0])
        c=3;
      break;
    case 3:
      extendLeg(1,0,walkSPL[0],y_push,0);
      y_push = y_push + prep_speed;
      if (y_push>=(y_ground+push_retract_height))
        c=4;
      break;
    case 4:
      extendLeg(1,0,x_push,y_ground+push_retract_height,0);
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1])
        c=5;
      break;
    case 5:
      extendLeg(1,0,walkSPL[1],y_push,0);
      y_push = y_push - prep_speed;
      if (y_push<=(y_ground)){
        c=2;
      }
      break;
  }
  writeAll();
}

void push_leg3(){ // skating gait for leg 3
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL[1];
      y_push = y_ground;
      if (!transition(0,0))
        done = false;
      if (!transition(1,0))
        done = false;
      if (!transition(2,0))
        done = false;
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      done = true;
      if (!transition(2,1))
        done = false;
      if (!transition(0,-1))
        done = false;
      if (!transition(1,-1))
        done = false;
      if (done){
        c = 2;
        extendLeg(0, 1, inactive_push_x, inactive_push_y, 1);
        extendLeg(1, 1, inactive_push_x, inactive_push_y, 1);
      }
      break;
    case 2:
      extendLeg(2,0,x_push,y_ground,0);
      x_push = x_push + push_speed;
      if (x_push>=walkSPL[0])
        c=3;
      break;
    case 3:
      extendLeg(2,0,walkSPL[0],y_push,0);
      y_push = y_push + prep_speed;
      if (y_push>=(y_ground+push_retract_height))
        c=4;
      break;
    case 4:
      extendLeg(2,0,x_push,y_ground+push_retract_height,0);
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1])
        c=5;
      break;
    case 5:
      extendLeg(2,0,walkSPL[1],y_push,0);
      y_push = y_push - prep_speed;
      if (y_push<=(y_ground))
        c=2;
      break;
  }
  writeAll();
}

void stand(){ // default pose. raises all four feet
  switch (c){
    case 0:
      done = true;
      if (first_time){
        for (int i = 0; i < 3; i++){
          if (!transition(i,0))
            done = false;;
        }
        if (done){
          first_time = false;
          c = 1;
        }
        break;
      }
      else{
        c=1;
        break;
      }
    case 1:
      for (int i = 0; i < 3; i++){
        extendLeg(i, 1, x_stand, y_ground, 1);
      }
  }
  writeAll();
}

void stop(){ // lowers all four feet
  switch (c){
    case 0:
      done = true;
      if (first_time){
        for (int i = 0; i < 3; i++){
          if (!transition(i,2))
            done = false;;
        }
        if (done){
          first_time = false;
          c = 1;
        }
        break;
      }
      else{
        c=1;
        break;
      }
    case 1:
      for (int i = 0; i < 3; i++){
        extendLeg(i, 0, x_stand, y_ground - stop_offset, 1);
      }
  }
  writeAll();
}

void pivot(int index){ // pivots about the leg corresponding to index
  switch (c){
    case 0:
      done = true;
      if (first_time){
        for (int i = 0; i < 3; i++){
          if (!transition(i,0))
            done = false;;
        }
        if (done){
          first_time = false;
          c = 1;
        }
        break;
      }
      else{
        c=1;
        break;
      }
    case 1:
      extendLeg(index, 0, x_stand, y_ground - stop_offset, 1);
      extendLeg((index+1)%3, 1, x_stand, y_ground - stop_offset, 1);
      extendLeg((index+2)%3, 1, x_stand, y_ground - stop_offset, 1);
  }
  writeAll();
}


// INCLINE FUNCS

float y_to_y_incline(int index, float x, float y, float incline){ // solves for distance in the y direction from a given upper robot joint to the ground using IMU data and incline angle
  float yaw_leg_specific;
  if (index == 0)
    yaw_leg_specific = yaw;
  else if (index == 1)
    yaw_leg_specific = yaw-120;
  else
    yaw_leg_specific = yaw+120;
  float x_projected = x*cosd(yaw_leg_specific);
  float incline_offset = x_projected*tand(incline);
  float y_adjusted = y + incline_offset;
  return y_adjusted;
}

bool transition_incline(int index, int which_func, float incline){ // same as transition but needs to account for incline using y_to_y_incline
  float x_goal;
  float y_goal;
  float footmode;
  bool override;
  if (which_func == 0){ // stand
    x_goal = x_stand;
    y_goal = y_to_y_incline(index, x_goal, y_ground, incline);
    footmode = 1;
    override = true;
  }
  else if (which_func == -1){ // inactive push, not currently used
    x_goal = inactive_push_x;
    y_goal = y_to_y_incline(index, x_goal, inactive_push_y, incline);
    footmode = 1;
    override = true;
  }
  else if (which_func == 1){ // active push, not currently used
    x_goal = walkSPL[1];
    y_goal = y_to_y_incline(index, x_goal, y_ground, incline);
    footmode = 0;
    override = false;
  }
  else if (which_func == 2){ // stop
    x_goal = x_stand;
    y_goal = y_to_y_incline(index, x_goal, y_ground-stop_offset, incline);
    footmode = 0;
    override = true;
  }

  float x = legs[index]->x;
  float y = legs[index]->y;
  if (abs(x-x_goal) <= 2 && abs(y-y_goal) <= 2){
    return true;
  }

  if (x > x_goal){
    if (y > y_goal)
      extendLeg(index, footmode, x-1, y-1, override);
    else if (y < y_goal)
      extendLeg(index, footmode, x-1, y+1, override);
    else
      extendLeg(index, footmode, x-1, y, override);
  }
  else if (x < x_goal){
    if (y > y_goal)
      extendLeg(index, footmode, x+1, y-1, override);
    else if (y < y_goal)
      extendLeg(index, footmode, x+1, y+1, override);
    else
      extendLeg(index, footmode, x+1, y, override);
  }
  else{
    if (y > y_goal)
      extendLeg(index, footmode, x, y-1, override);
    else if (y < y_goal)
      extendLeg(index, footmode, x, y+1, override);
  }

  return false;
}

void stand_incline(float incline){ // all feet up on incline of angle incline.
  switch (c){
    case 0:
      done = true;
      if (first_time){
        for (int i = 0; i < 3; i++){
          if (!transition_incline(i,0, incline))
            done = false;;
        }
        if (done){
          first_time = false;
          c = 1;
        }
        break;
      }
      else{
        c=1;
        break;
      }
    case 1:
      for (int i = 0; i < 3; i++){
        extendLeg(i, 1, x_stand, y_to_y_incline(i, x_stand, y_ground, incline), 1);
      }
  }
  writeAll();
}

void stop_incline(float incline){ // all feet down on incline of angle incline.
  switch (c){
    case 0:
      done = true;
      if (first_time){
        for (int i = 0; i < 3; i++){
          if (!transition_incline(i,2, incline))
            done = false;;
        }
        if (done){
          first_time = false;
          c = 1;
        }
        break;
      }
      else{
        c=1;
        break;
      }
    case 1:
      for (int i = 0; i < 3; i++){
        extendLeg(i, 0, x_stand, y_to_y_incline(i, x_stand, y_ground - stop_offset, incline), 1);
      }
  }
  writeAll();
}

void pivot_incline(int index, float incline){ // pivot about leg corresponding to index while level on incline of angle incline.
  switch (c){
    case 0:
      done = true;
      if (first_time){
        if (!transition_incline(index,2, incline))
          done = false;
        if (!transition_incline((index+1)%3,0, incline))
          done = false;
        if (!transition_incline((index+2)%3,0, incline))
          done = false;
        if (done){
          first_time = false;
          c = 1;
        }
        break;
      }
      else{
        c=1;
        break;
      }
    case 1:
      //Serial.print("y: ");Serial.println(y_to_y_incline(index, x_stand, y_ground, incline));
      extendLeg(index, 0, x_stand, y_to_y_incline(index, x_stand, y_ground - stop_offset, incline), 1);
      extendLeg((index+1)%3, 1, x_stand, y_to_y_incline((index+1)%3, x_stand, y_ground, incline), 1);
      extendLeg((index+2)%3, 1, x_stand, y_to_y_incline((index+2)%3, x_stand, y_ground, incline), 1);
  }
  writeAll();
}

void traverse_cable(){ // uses a series of movements to traverse an extension cable or other small obstacle
  switch(c){
    case 0:
      done = true;
      x_push = x_stand;
      y_push = y_ground;
      if (!transition(0,0))
        done = false;
      if (!transition(1,-1))
        done = false;
      if (!transition(2,-1))
        done = false;
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      extendLeg(0,0,x_push,y_push-12,0); // robot forward
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1]){
        c=2;
        extendLeg(1,0,inactive_push_x,inactive_push_y,1);
        extendLeg(2,0,inactive_push_x,inactive_push_y,1);
      }
      break;
    case 2:
      extendLeg(0,0,x_push,y_push,0); // leg up
      y_push = y_push + prep_speed;
      if (y_push>=y_ground+40)
        c=3;
      break;
    case 3:
      extendLeg(0,0,x_push,y_push,0); // leg out
      x_push = x_push + prep_speed;
      if (x_push>=walkSPL[0])
        c=4;
      break;
    case 4:
      extendLeg(0,0,x_push,y_push,0); // leg down
      y_push = y_push - prep_speed;
      if (y_push<=y_ground-10)
        c=5;
      break;
    case 5:
      extendLeg(0,0,x_push,y_push-20,0); // robot forward
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL_noball[0])
        c=6;
      break;
    case 6:
      done = true;
      extendLeg(0,0,x_push,y_push,0); // back legs down
      if (!transition(1,5))
        done = false;
      if (!transition(2,5))
        done = false;
      if (done){
        c = 7;
        y_push = y_ground;
      }
      break;
    case 7:
      done = true;
      if (!transition(0,6)) // front leg down, robot up
        done = false;
      if (done){
        c = 8;
        //y_push = y_ground; 
      }
      break;
    case 8: 
      extendLeg(0,0,x_push,y_push,0); // robot forward
      x_push = x_push - prep_speed;
      if (x_push<=100)
        c=9;
      break;
     case 9: 
      extendLeg(0,1,x_push,y_push,0); // leg out
      x_push = x_push + prep_speed;
      if (x_push>=220)
        c=10;
      break;
    case 10: 
      extendLeg(0,0,x_push,y_push,0); // robot forward
      x_push = x_push - prep_speed;
      if (x_push<=100)
        c=11;
      break;
    case 11:
      done = true;
      x_push = walkSPL_noball[1]; // robot down
      if (!transition(0,0))
        done = false;
      if (!transition(1,0))
        done = false;
      if (!transition(2,0))
        done = false;
      if (done){
        c = 12; // doesn't loop
      }
      break;
  }
  writeAll();
}

int stair_height = 100;

void stair(){ // uses a series of movements to traverse an extension cable or other small obstacle
  switch(c){
    case 0:
      done = true;
      x_push = x_stand;
      y_push = y_ground;
      y_push_inactive = inactive_push_y;
      x_push_inactive = inactive_push_x;
      if (!transition(0,0))
        done = false;
      if (!transition(1,-1))
        done = false;
      if (!transition(2,-1))
        done = false;
      if (done){
        first_time = false;
        c = 1;
        delta = 20;
      }
      break;
    case 1:
      extendLeg(0,0,x_push,y_push-15,0); // robot forward
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1]){
        c=2;
        extendLeg(1,0,inactive_push_x,inactive_push_y,1);
        extendLeg(2,0,inactive_push_x,inactive_push_y,1);
      }
      break;
    case 2:
      extendLeg(0,0,x_push,y_push,0); // leg up
      y_push = y_push + prep_speed;
      if (y_push>=y_ground+140)
        c=3;
      break;
    case 3:
      extendLeg(0,0,x_push,y_push,0); // leg out
      x_push = x_push + prep_speed;
      if (x_push>=walkSPL[0])
        c=4;
      break;
    case 4:
      extendLeg(0,0,x_push,y_push,0); // leg down
      y_push = y_push - prep_speed;
      if (y_push<=y_ground+stair_height-25){
        extendLeg(1,1,inactive_push_x,inactive_push_y,1);
        extendLeg(2,1,inactive_push_x,inactive_push_y,1);
        c=5;
      }
      break;
    case 5:
      extendLeg(0,0,x_push,y_push,0); // rob forward
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1]){
        c=6;
      }
      break;
    case 6:
      extendLeg(1,1,x_push_inactive,y_push_inactive,1); // back legs in
      extendLeg(2,1,x_push_inactive,y_push_inactive,1);
      x_push_inactive = x_push_inactive - prep_speed;
      if (x_push_inactive<=130){
        c=7;
      }
      break;
    case 7:
      extendLeg(1,1,x_push_inactive,y_push_inactive,1); // back legs down, robot up
      extendLeg(2,1,x_push_inactive,y_push_inactive,1);
      y_push_inactive = y_push_inactive - prep_speed;
      if (y_push_inactive<=y_ground-120){
        c=8;
        extendLeg(1,0,x_push_inactive,y_push_inactive,1);
        extendLeg(2,0,x_push_inactive,y_push_inactive,1);
        y_push = y_ground-50;
      }
      break;
    case 8:
      extendLeg(0,1,x_push,y_push+10,0); // leg out
      x_push = x_push + prep_speed;
      if (x_push>=350){
        c=9;
        extendLeg(1,1,x_push_inactive,y_push_inactive,1);
        extendLeg(2,1,x_push_inactive,y_push_inactive,1);
      }
      break;
    case 9:
      extendLeg(0,0,x_push,y_push,0); // rob forward
      x_push = x_push - prep_speed;
      if (x_push<=100){
        c=10;
        extendLeg(1,0,x_push_inactive,y_push_inactive,1);
        extendLeg(2,0,x_push_inactive,y_push_inactive,1);
        y_push = y_ground;
      }
      break;
    case 10:
      extendLeg(0,1,x_push,y_push+5,0); // leg out
      x_push = x_push + prep_speed;
      if (x_push>=400){
        c=11;
        extendLeg(0,0,x_push,y_push,0);
      }
      break;
    case 11:
      extendLeg(0,0,x_push,y_push+5,0); // leg up
      y_push = y_push + prep_speed;
      if (y_push>=y_ground + 50){
        c=12;
      }
      break;

      break;
    case 12:
      extendLeg(1,0,x_push_inactive,y_push_inactive,1); // back legs up
      extendLeg(2,0,x_push_inactive,y_push_inactive,1);
      y_push_inactive = y_push_inactive + 1;
      x_push_inactive = 100;
      if (y_push_inactive>=y_ground+90){
        c=13;
      }
      break;
    case 13:
      extendLeg(0,0,x_push,y_push,0); // leg down
      y_push = y_push - 1;
      if (y_push<=y_ground-20){
        c=14;
        delta = 0;
      }
      break;
    case 14:
      extendLeg(0,0,x_push,y_push,0); // rob forward
      x_push = x_push - 1;
      if (x_push<=100){
        c=15;
        delta = 20;
      }
      break;
  }
  writeAll();
}

void leg_up_down(){ // uses a series of movements to traverse an extension cable or other small obstacle
  switch(c){
    case 0:
      done = true;
      x_push = x_stand;
      y_push = y_ground;
      y_push_inactive = inactive_push_y;
      x_push_inactive = inactive_push_x;
      if (!transition(0,0))
        done = false;
      if (!transition(1,-1))
        done = false;
      if (!transition(2,-1))
        done = false;
      if (done){
        first_time = false;
        c = 1;
        delta = 20;
      }
      break;
    case 1:
      extendLeg(0,0,x_push,y_push,0); // robot forward
      y_push = y_push + prep_speed;
      x_push = x_push + prep_speed;
      if (y_push>=0){
        c=2;
      }
      break;
  writeAll();
}