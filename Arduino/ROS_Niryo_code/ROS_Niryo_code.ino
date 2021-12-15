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

int cnt = -100;
int dir = 1;
int jointNumber = 1;

void loop(){
  
  // Szervók mozgatása
  joint5.write(joint5_angle);
  joint6.write(joint6_angle);
  
  // Léptetőmotorok mozgatása
  //steppers.moveTo(positions);
  //steppers.runSpeedToPosition();

  digitalWrite(13, HIGH-digitalRead(13)); // Beépített LED villog

if ((dir == 0) && (cnt < -4000))
{
  dir = 1;
}
if ((dir == 1) && (cnt > 4000))
{
  
 /* Serial.print("jointNumber: ");
  Serial.print(jointNumber);
  Serial.print("\r\n");*/
  dir = 0; 
  if (jointNumber == 1)
  {
    jointNumber = 2;
    Serial.print("Change jointNumber from 1 to 2 ");
    Serial.print("\r\n");
  }
  else if (jointNumber == 2)
  {
    jointNumber = 1;
    
    Serial.print("Change jointNumber from 2 to 1 ");
    Serial.print("\r\n");
  }
  
  /*Serial.print("jointNumber: ");
  Serial.print(jointNumber);
  Serial.print("\r\n");*/
}

//Joint1
if (jointNumber == 1)
{
  if ((dir == 0) && (cnt > -2000))
  {
    joint1.moveTo(cnt);
    joint1.setMaxSpeed(stepper_speed);
    joint1.setAcceleration(stepper_accel);
    
    joint1.run();
    gripper_on();
  }
  if ((dir == 1) && (cnt < 2000))
  {
    joint1.moveTo(cnt);
    
    joint1.setMaxSpeed(stepper_speed);
    joint1.setAcceleration(stepper_accel);
    joint1.run();
    gripper_off();
  }
}

//Joint2
if (jointNumber == 2)
{
  if ((dir == 0) && (cnt > -2000))
  {
    joint2.moveTo(cnt);
    joint2.setMaxSpeed(stepper_speed);
    joint2.setAcceleration(stepper_accel);
    
    joint2.run();
    gripper_on();
  }
  if ((dir == 1) && (cnt < 2000))
  {
    joint2.moveTo(cnt);
    
    joint2.setMaxSpeed(stepper_speed);
    joint2.setAcceleration(stepper_accel);
    joint2.run();
    gripper_off();
  }
}

//Stop
if ((cnt > 3000 ) || cnt < -3000)
{
  joint1.setAcceleration(0);
  joint1.setMaxSpeed(0);
  joint2.setAcceleration(0);
  joint2.setMaxSpeed(0);
}

if (dir == 0)
{
  cnt--;
}
else
{
  cnt++;
}
  //joint1.run();

  if ((cnt % 100) == 0)
  {
  Serial.print("cnt: ");
  Serial.print(cnt);
  Serial.print("\r\n");
  }
  // Szervók mozgatása
  //joint5.write(180);
  //joint6.write(180);
  //delay(1000);
}
