#include <Servo.h>

Servo s1;Servo s2;Servo s3;Servo s4;Servo s5;Servo s6;Servo s7;Servo s8;Servo s9;

const int universaloffset1 = 50; // set this to the approximate angle that link1 makes with servo_mount_60kg
const int universaloffset2 = 16; // set this to the approximate angle that link2p1 makes with servo_mount_60kg_slotted

// start by setting these all to 0 (these were my offsets), and change them until each leg 
// points straightout from the body parallel to the floor (remove center ball from robot)
const int leg1offset1 = -1; 
const int leg1offset2 = -19;
const int leg2offset1 = 6;
const int leg2offset2 = -9;
const int leg3offset1 = 8;
const int leg3offset2 = -7;

int q1 = 14; // After finding all offsets, set q1 to 14 and q2 to 110 and the robot should stand up and be able to roll around
int q2 = 110;
int q_foot = 0; // Also try change q_foot to 55 to verify that the 20kg servos are working. 
                // You should see the spherical bearings at the end of each leg move all the way up.

void setup() {
  s1.attach(4);s2.attach(5);s3.attach(6); //leg 1
  s4.attach(7);s5.attach(8);s6.attach(9); //leg 2
  s7.attach(10);s8.attach(11);s9.attach(12); //leg 3
}

void loop() {
  //leg 1
  s1.write(q1+universaloffset1+leg1offset1);
  s2.write(q2+universaloffset2+leg1offset2);
  s3.write(q_foot);
  //leg 2
  s4.write(q1+universaloffset1+leg2offset1);  
  s5.write(q2+universaloffset2+leg2offset2);
  s6.write(q_foot);
  //leg 3
  s7.write(q1+universaloffset1+leg3offset1);  
  s8.write(q2+universaloffset2+leg3offset2);
  s9.write(q_foot);
}
