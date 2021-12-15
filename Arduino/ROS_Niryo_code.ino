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

ros::NodeHandle  nh; // ROS csomópont kezelése

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
long goal_position[4];
long positions[4];
double prev_angle[4] = {0,0,-1.4,0};
double cur_angle[4];
double mod[4] = {6.0625,8.3125,7.875,5.18};
int joint5_angle = 90;
int joint6_angle = 90;
int gripper_angle = 90;
int stepper_speed = 4000;
int stepper_accel = 2000;

  int step0 = 1000; //J1
  int step1 = 1100;
  int step2 = 2100; //J2
  int step3 = 2200;
  int step4 = 3200; //J3
  int step5 = 3300;
  int step6 = 4300; //J4
  int step7 = 4400;
  int step8 = 5400; //S1
  int step9 = 5500;
  int step10 = 6500; //S2
  int step11 = 6600;
  int step12 = 7600;

String incomingString = ""; // for incoming serial data
String lastString = ""; // for incoming serial data

sensor_msgs::JointState cmd_msg;
int dir = 0;
int dirCnt = 0;

void cmd_cb(const sensor_msgs::JointState& _cmd_msg){

  // Szöghelyzetek beolvasása
  cur_angle[0] = _cmd_msg.position[0];
  cur_angle[1] = _cmd_msg.position[1];
  cur_angle[2] = _cmd_msg.position[2];
  cur_angle[3] = _cmd_msg.position[3];
  cur_angle[4] = _cmd_msg.position[4];
  cur_angle[5] = _cmd_msg.position[5];
  
  // Lépések számítása
  joint_step[0] = mod[0] * ((cur_angle[0]-prev_angle[0])*stepsPerRevolution[0]/(2*M_PI));
  joint_step[1] = mod[1] * ((cur_angle[1]-prev_angle[1])*stepsPerRevolution[1]/(2*M_PI));
  joint_step[2] = mod[2] * ((cur_angle[2]-prev_angle[2])*stepsPerRevolution[2]/(2*M_PI));
  joint_step[3] = mod[3] * ((cur_angle[3]-prev_angle[3])*stepsPerRevolution[3]/(2*M_PI));

  // Lépések összegzése
  total_joint_step[0] += joint_step[0];
  total_joint_step[1] += joint_step[1];
  total_joint_step[2] += joint_step[2];
  total_joint_step[3] += joint_step[3];

  // Feltétel, ami a végső pozíció elérését figyeli
  if (prev_angle[0] == cur_angle[0] &&
      prev_angle[1] == cur_angle[1] &&
      prev_angle[2] == cur_angle[2] &&
      prev_angle[3] == cur_angle[3]){

        goal_position[0] = total_joint_step[0];
        goal_position[1] = total_joint_step[1];
        goal_position[2] = total_joint_step[2];
        goal_position[3] = total_joint_step[3];

        // Szervók szögének átváltása radiánból fokba
        joint5_angle = (int) (90 - (cur_angle[4] * 180 / M_PI));
        joint6_angle = (int) (90 + (cur_angle[5] * 180 / M_PI));
        
  }
  // Szöghelyzetek mentése a következő ciklushoz
  prev_angle[0] = cur_angle[0];
  prev_angle[1] = cur_angle[1];
  prev_angle[2] = cur_angle[2];
  prev_angle[3] = cur_angle[3];

}

void gripper_cb( const std_msgs::UInt16& _cmd_msg){
   gripper.write(_cmd_msg.data); // Megfogó szervó szöge 45-135    
}

void gripper_on()
{
   gripper.write(50); // Megfogó szervó szöge 45-135    
}
void gripper_off()
{
   gripper.write(130); // Megfogó szervó szöge 45-135    
}

// ROS feliratkozás témákra, üzenetek lekérése
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", cmd_cb);
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper", gripper_cb);

void setup(){
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps
  // Csomópontok inicializálása, feliratkozás
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(gripper_sub);

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
  
  cmd_msg.position[0] = 0;
  cmd_msg.position[1] = 0;
  cmd_msg.position[2] = 0;
  cmd_msg.position[3] = 0;
  cmd_msg.position[4] = 0;
  cmd_msg.position[5] = 0;
    Serial.print(" \r\n****Init Complete:****\r\n");
}

void loop(){
  
  // Léptetőmotor pozíciók tömbje
  positions[0] = goal_position[0]; 
  positions[1] = goal_position[1]; 
  positions[2] = -goal_position[2]; // Negálni kell, ha a ROS-hoz képest másik irányba forog
  positions[3] = goal_position[3];

  // Szervók mozgatása
  joint5.write(joint5_angle);
  joint6.write(joint6_angle);
  
  // Léptetőmotorok mozgatása
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  digitalWrite(13, HIGH-digitalRead(13)); // Beépített LED villog

  // Visszahívások (cb) ciklusonkénti futtatása
  nh.spinOnce();

  // check if data is available
  if (Serial.available() > 0) {
    // read the incoming string:
    incomingString = Serial.readStringUntil('\n');

    // prints the received data
    Serial.print("I received: ");
    Serial.println(incomingString);
    lastString = incomingString;
  }
    cmd_msg.position[0] = 0;
    cmd_msg.position[1] = 0;
    cmd_msg.position[2] = 0;
    cmd_msg.position[3] = 0;
    cmd_msg.position[4] = 0;
    cmd_msg.position[5] = 0;
  if (lastString == "a")
  {
    cmd_msg.position[1] = 10;
  }
  if (lastString == "b")
  {
    cmd_msg.position[1] = 0;
  }

  dirCnt++;
  
      cmd_msg.position[0] = 0; 
      cmd_msg.position[1] = 0; 
      cmd_msg.position[2] = 0; 
      cmd_msg.position[3] = 0; 
      cmd_msg.position[4] = 0;
      cmd_msg.position[5] = 0;

    if ((dirCnt > 0) && (dirCnt < step0))
    {
      cmd_msg.position[0] = dirCnt % 10;
    }
     if (((dirCnt > step0) && (dirCnt < step1 )) || ((dirCnt > step4) && (dirCnt < step5 )) || ((dirCnt > step8) && (dirCnt < step9 )))
    { 
      gripper_on();
      cmd_msg.position[5] = 70;
      cmd_msg.position[6] = 70;
    }
    
     if (((dirCnt > step2) && (dirCnt < step3 )) || ((dirCnt > step6) && (dirCnt < step7 )) || ((dirCnt > step10) && (dirCnt < step11 )))
    { 
      gripper_off();
      cmd_msg.position[5] = 70;
      cmd_msg.position[6] = 70;
    }
    
    if ((dirCnt > step1) && (dirCnt < step2))
    {
      cmd_msg.position[1] = dirCnt % 100;
    }
    
    if ((dirCnt > step3) && (dirCnt < step4))
    {
      cmd_msg.position[2] = dirCnt % 100;
    }
    
    if ((dirCnt > step5) && (dirCnt < step6))
    {
      cmd_msg.position[3] = dirCnt % 100;
    }
    
    if ((dirCnt > step7) && (dirCnt < step8))
    {
      cmd_msg.position[4] = dirCnt % 100;
    }
    
    if (dirCnt > step9)
    {
      dirCnt = 0;
    }
    
    cmd_cb(cmd_msg);
  delay(1);
}
