// Könyvtárak tartalmazása
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

// Léptetőmotorok vezérlésére szolgáló pinek definiálása
// Joint 1
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

// Joint 2
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// Joint 3
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

// Joint 4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Léptetőmotorok és szervók definiálása
AccelStepper joint1(1, E0_STEP_PIN, E0_DIR_PIN); // 1 = Driver
AccelStepper joint2(1, E1_STEP_PIN, E1_DIR_PIN); // 1 = Driver
AccelStepper joint3(1, Y_STEP_PIN, Y_DIR_PIN); // 1 = Driver
AccelStepper joint4(1, X_STEP_PIN, X_DIR_PIN); // 1 = Driver
MultiStepper steppers;
Servo joint5;
Servo joint6;
Servo gripper;

// Program változóinak deklarálása és inicializálása
int stepsPerRevolution[4] = {3200,3200,3200,3200};
long joint_step[4];
long total_joint_step[4];
long positions[4];
double mod[4] = {6.0625,8.3125,7.875,5.18};
int joint5_angle = 90;
int joint6_angle = 90;
int gripper_angle = 90;
int stepper_speed = 12000;
int stepper_accel = 2000;

void gripper_on()
{
   gripper.write(50); // Megfogó szervó szöge 45-135    
}
void gripper_off()
{
   gripper.write(130); // Megfogó szervó szöge 45-135    
}

void setup(){
  Serial.begin(115200); // opens serial port, sets data rate to 115200 bps
  // Csomópontok inicializálása

  // Vezérlő pinek inicializálása
  pinMode(E0_STEP_PIN, OUTPUT);
  pinMode(E0_DIR_PIN, OUTPUT);
  pinMode(E0_ENABLE_PIN, OUTPUT);
  digitalWrite (E0_ENABLE_PIN, LOW);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  digitalWrite (X_ENABLE_PIN, LOW);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  digitalWrite (E1_ENABLE_PIN, LOW);
  pinMode(E1_STEP_PIN, OUTPUT);
  pinMode(E1_DIR_PIN, OUTPUT);
  pinMode(E1_ENABLE_PIN, OUTPUT);
  digitalWrite (E1_ENABLE_PIN, LOW);

  // Ventilátor bekapcsolása
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

  // Léptetőmotorok sebessége
  joint1.setMaxSpeed(stepper_speed);
  joint2.setMaxSpeed(stepper_speed);
  joint3.setMaxSpeed(stepper_speed);
  joint4.setMaxSpeed(stepper_speed);

  joint1.setAcceleration(stepper_accel);
  joint2.setAcceleration(stepper_accel);
  joint3.setAcceleration(stepper_accel);
  joint4.setAcceleration(stepper_accel);

  // Hozzáadás MultiStepper-hez
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);

  // Szervó vezérlő PWM pinek definiálása
  joint5.attach(5);
  joint6.attach(4); 
  gripper.attach(11);
  
  Serial.print(" \r\n****Init Complete:****\r\n");
}

int cnt = 0;
int dir = 1;
int jointNumber = 1;

int planId = 1;

//joint2 is not stabil
//joint3 has a broken cable

//joint 5 and 6 are the servos. They need a degree for 'dir'
//joint 7 is the gripper. It also need a value between 45 and 135 for 'dir'
int plan1_joint = 1;
int plan1_dir = 0;

int plan2_joint = 1;
int plan2_dir = 1;

int plan3_joint = 4;
int plan3_dir = 0;

int plan4_joint = 4;
int plan4_dir = 1;

int plan5_joint = 5;
int plan5_dir = 45;

int plan6_joint = 5;
int plan6_dir = 90;

int plan7_joint = 6;
int plan7_dir = 45;

int plan8_joint = 6;
int plan8_dir = 90;

int plan9_joint = 7;
int plan9_dir = 45;

int plan10_joint = 7;
int plan10_dir = 90;

void loop(){
  
  // Szervók mozgatása
  //joint5.write(joint5_angle);
  //joint6.write(joint6_angle);
  
  // Léptetőmotorok mozgatása
  //steppers.moveTo(positions);
  //steppers.runSpeedToPosition();

  digitalWrite(13, HIGH-digitalRead(13)); // Beépített LED villog

  switch (planId) {
    case 1:
      moveJoint(plan1_joint, cnt, plan1_dir);
      break;
    case 2:
      moveJoint(plan2_joint, cnt, plan2_dir);
      break;
    case 3:
      moveJoint(plan3_joint, cnt, plan3_dir);
      break;
    case 4:
      moveJoint(plan4_joint, cnt, plan4_dir);
      break;
    case 5:
      moveJoint(plan5_joint, cnt, plan5_dir);
      break;
    case 6:
      moveJoint(plan6_joint, cnt, plan6_dir);
      break;
    case 7:
      moveJoint(plan7_joint, cnt, plan7_dir);
      break;
    case 8:
      moveJoint(plan8_joint, cnt, plan8_dir);
      break;
    case 9:
      moveJoint(plan9_joint, cnt, plan9_dir);
      break;
    case 10:
      moveJoint(plan10_joint, cnt, plan10_dir);
      break;
    default:
      planId = 1;
  }  

  //switch planId
  if ((cnt > 4000) || (cnt < -4000))
  {
    planId++;
    delay(1000);
    cnt = 0;
    Serial.print("New PlanID: ");
    Serial.print(planId);
    Serial.print("\r\n");
  }
  
  if ((cnt % 1000) == 0)
  {
  Serial.print("cnt: ");
  Serial.print(cnt);
  Serial.print("\r\n");
  }
}

int changeCnt(int direction)
{
      if (direction == 0)
      {
        return cnt--;
      }
      else if (direction == 1)
      {
        return cnt++;
      }
      return 0;
}

void moveJoint(int jointId, int _cnt, int _dir)
{
  switch (jointId)
  {
    case 1:
        changeCnt(_dir);
        joint1.moveTo(_cnt);        
        joint1.run();
      break;
    case 2:
        changeCnt(_dir);
        joint2.moveTo(_cnt);        
        joint2.run();
      break;
    case 3:
        changeCnt(_dir);
        joint3.moveTo(_cnt);        
        joint3.run();
      break;
    case 4:
        changeCnt(_dir);
        joint4.moveTo(_cnt);        
        joint4.run();
      break;
    //servo
    case 5:
      if (_dir > 180)
      {
        _dir = 180;
      }
      if (_dir < -180)
      {
        _dir = -180;
      }
      joint5.write(_dir);
    planId++;
    delay(1000);
      break;
    //servo
    case 6:
      if (_dir > 180)
      {
        _dir = 180;
      }
      if (_dir < -180)
      {
        _dir = -180;
      }
      joint6.write(_dir);
    planId++;
    delay(1000);
      break;

      break;
    //servo, gripper
    case 7:
      if (_dir > 135)
      {
        _dir = 135;
      }
      if (_dir < 45)
      {
        _dir = 45;
      }
      cnt = 0;
      gripper.write(_dir); // Megfogó szervó szöge 45-135  
      
    planId++;
    delay(1000);  
      break;
  }
}
