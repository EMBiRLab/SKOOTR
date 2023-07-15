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
//end

struct joint{
  Servo servo;
  int pin;
  float pos = 0;
  int flo = 0;
  int ciel = 130; 
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
joint j11;joint j12; joint j13; joint j21; joint j22;joint j23; joint j31; joint j32; joint j33;
leg leg1 = {0,&j11,&j12,&j13,0,0,-1,-19}; 
leg leg2 = {1,&j21,&j22,&j23,0,0,6,-9};
leg leg3 = {2,&j31,&j32,&j33,0,0,8,-7};
leg * legs[3] = {&leg1,&leg2,&leg3};

//user input
char temp;
char which_func = '0';
bool newData = false;

//timing
unsigned long delta = 10; // time between IK recalculations
unsigned long timeToMove;
int c = 0;

//offsets+movement
const float y_ground = -112;
const int universaloffset1 = 50; // don't change
const int universaloffset2 = 16; // don't change
float L0 = 126; // link lengths
float L1 = 97;
float L2 = 245; 
float d = 97;
float Ln = 159.01; //sqrt(L1*L1+L0*L0);
float psi_dif = 37.59; //atan(d/L0)*180/pi;

float x_push = 0; // globals for push functions
float y_push = y_ground;
float push_speed = 4; // speed for pushing motion
float prep_speed = 2; // speed between pushing motions
bool done;
bool first_time;
int inactive_push_x = 200; // vars for inactive leg positions
int inactive_push_y = y_ground + 30;
int push_retract_height = 22;
int walkSPL[2] = {260,90};

void setup() {

  j11.pin = 4;  
  j12.pin = 5; 
  j13.pin = 6; 
  j21.pin = 7;
  j22.pin = 8;
  j23.pin = 9; 
  j31.pin = 10;
  j32.pin = 11;
  j33.pin = 12;
  /*j21.pin = 13;  // disables leg for testing
  j22.pin = 14; 
  j23.pin = 15; */

  for (leg * l: legs){
    l->joint1->servo.attach(l->joint1->pin);
    l->joint2->servo.attach(l->joint2->pin);
    l->joint3->servo.attach(l->joint3->pin);
  }

  for (int i = 0; i < 180; i++)
    stand(1);

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
  // make sure it worked
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
  if (Serial.available() > 0) {
     temp = Serial.read();
     newData = true;
  }
  if (newData == true) {
        Serial.print("Received char input: ");Serial.println(temp);
        which_func = temp;
        c = 0;
        first_time = true;
        newData = false;
  }
  if (millis() - timeToMove >= delta){
    switch (which_func){
      case '1':
        push_leg1();
        break;
      case '2':
        push_leg2();
        break;
      case '3':
        push_leg3();
        break;
      default:
        stand(1);
        break;
    }
    timeToMove = millis();
    //Serial.print("roll: ");Serial.print(roll);Serial.print(" pitch: ");Serial.println(pitch);
  }

  if (!dmpReady) return;
  //calc roll+pitch
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll = (ypr[2] * 180/M_PI);
    pitch = (ypr[1] * 180/M_PI);
   }


}

void push_leg1(){
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL[1];
      y_push = y_ground;
      if (first_time){
        if (!transition(0,1))
          done = false;
        if (!transition(1,-1))
          done = false;
        if (!transition(2,-1))
          done = false;
      }
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      extendLeg(0,0,x_push,y_ground,0);
      x_push = x_push + push_speed;
      if (x_push>=walkSPL[0])
        c=2;
      break;
    case 2:
      extendLeg(0,0,walkSPL[0],y_push,0);
      y_push = y_push + prep_speed;
      if (y_push>=(y_ground+push_retract_height))
        c=3;
      break;
    case 3:
      extendLeg(0,0,x_push,y_ground+push_retract_height,0);
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1])
        c=4;
      break;
    case 4:
      extendLeg(0,0,walkSPL[1],y_push,0);
      y_push = y_push - prep_speed;
      if (y_push<=(y_ground))
        c=0;
      break;
  }
  if (c!=0){
    extendLeg(1, 1, inactive_push_x, inactive_push_y, 1);
    extendLeg(2, 1, inactive_push_x, inactive_push_y, 1);
  }
  writeAll();
}

void push_leg2(){
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL[1];
      y_push = y_ground;
      if (first_time){
        if (!transition(1,1))
          done = false;
        if (!transition(0,-1))
          done = false;
        if (!transition(2,-1))
          done = false;
      }
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      extendLeg(1,0,x_push,y_ground,0);
      x_push = x_push + push_speed;
      if (x_push>=walkSPL[0])
        c=2;
      break;
    case 2:
      extendLeg(1,0,walkSPL[0],y_push,0);
      y_push = y_push + prep_speed;
      if (y_push>=(y_ground+push_retract_height))
        c=3;
      break;
    case 3:
      extendLeg(1,0,x_push,y_ground+push_retract_height,0);
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1])
        c=4;
      break;
    case 4:
      extendLeg(1,0,walkSPL[1],y_push,0);
      y_push = y_push - prep_speed;
      if (y_push<=(y_ground))
        c=0;
      break;
  }
  if (c!=0){
    extendLeg(0, 1, inactive_push_x, inactive_push_y, 1);
    extendLeg(2, 1, inactive_push_x, inactive_push_y, 1);
  }
  writeAll();
}

void push_leg3(){
  switch(c){
    case 0:
      done = true;
      x_push = walkSPL[1];
      y_push = y_ground;
      if (first_time){
        if (!transition(2,1))
          done = false;
        if (!transition(0,-1))
          done = false;
        if (!transition(1,-1))
          done = false;
      }
      if (done){
        first_time = false;
        c = 1;
      }
      break;
    case 1:
      extendLeg(2,0,x_push,y_ground,0);
      x_push = x_push + push_speed;
      if (x_push>=walkSPL[0])
        c=2;
      break;
    case 2:
      extendLeg(2,0,walkSPL[0],y_push,0);
      y_push = y_push + prep_speed;
      if (y_push>=(y_ground+push_retract_height))
        c=3;
      break;
    case 3:
      extendLeg(2,0,x_push,y_ground+push_retract_height,0);
      x_push = x_push - prep_speed;
      if (x_push<=walkSPL[1])
        c=4;
      break;
    case 4:
      extendLeg(2,0,walkSPL[1],y_push,0);
      y_push = y_push - prep_speed;
      if (y_push<=(y_ground))
        c=0;
      break;
  }
  if (c!=0){
    extendLeg(0, 1, inactive_push_x, inactive_push_y, 1);
    extendLeg(1, 1, inactive_push_x, inactive_push_y, 1);
  }
  writeAll();
}

void stand(bool foot_mode){
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
        extendLeg(i, 1, 200, y_ground, 1);
      }
  }
  writeAll();
}

bool extendLeg(int index, int foot_mode, int x, int y, bool override){
  leg * l = *(legs+index);
  l->x = x;
  l->y = y;
  q_struct qs = IK(index, 1, override);
  float posj1 = qs.q1+universaloffset1+l->offset1 + 90-psi_dif;
  float posj2 = qs.q2+universaloffset2+l->offset2;
  if (foot_mode == 0)
    legs[index]->joint3->pos = 40;
  else
    legs[index]->joint3->pos = 0;
  l->joint1->pos = posj1;
  l->joint2->pos = posj2;
}

struct q_struct IK(int index, float ornt, bool override){
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
  
  /*Serial.print("x: ");Serial.println(x);
  Serial.print("phi: ");Serial.println(phi);
  Serial.print("psi: ");Serial.println(psi);
  Serial.print("x1: ");Serial.println(x1);
  Serial.print("y1: ");Serial.println(y1);
  Serial.print("L1: ");Serial.println(L1);
  Serial.print("L2: ");Serial.println(L2);*/

  float q2 = ornt*acosd((pow(x1,2) + pow(y1,2) - pow(L1,2) - pow(L2,2))/(2*L1*L2));
  float q1 = atan2d(y1,x1) + atan2d((L2*sind(q2)),(L1+L2*cosd(q2))) - psi;
  //Serial.print("q1: ");Serial.println(q1);
  //Serial.print("q2: ");Serial.println(q2);
  q_struct qs = {q1,q2};
  return qs;
}

float getPhi(int index, float roll, float pitch){
  float mag = acosd(cosd(pitch)*cosd(roll)/pow(pow(sind(pitch),2)+pow(cosd(pitch),2)*pow(sind(roll),2)+pow(cosd(pitch),2)*pow(cosd(roll),2),1/2));
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

bool transition(int index, int which_func){
  float x_goal;
  float y_goal;
  float footmode;
  bool override;
  if (which_func == 0){ // stand
    x_goal = 200;
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

void writeAll(){
  for (leg * l: legs){
    l->joint1->servo.write(constrain(l->joint1->pos, l->joint1->flo, l->joint1->ciel));
    l->joint2->servo.write(constrain(l->joint2->pos, l->joint2->flo, l->joint2->ciel)); 
    l->joint3->servo.write(constrain(l->joint3->pos, l->joint3->flo, l->joint3->ciel));
    if (l->index == 0 && which_func == '1'){
      //Serial.print("joint 1: ");Serial.println(l->joint1->pos);
      //Serial.print("joint 2: ");Serial.println(l->joint2->pos);
      //Serial.print("j3: ");Serial.println(l->joint3->pos);
    }
  }
}

float sind(float x){
  return sin(x*pi/180.0);
}
float cosd(float x){
  return cos(x*pi/180.0);
}
float acosd(float x){
  return acos(x)*180.0/pi;
}
float atan2d(float y, float x){
  return atan2(y,x)*180.0/pi;
}


