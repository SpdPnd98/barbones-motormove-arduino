// change flipped to match robot movement
#define FLIPPED true

#if !FLIPPED
#define ENA 5
#define ENB 6
#define MOTORLA 13
#define MOTORLB 12
#define MOTORRA 8
#define MOTORRB 7

#elif FLIPPED
#define ENA 5
#define ENB 6
#define MOTORLA 12
#define MOTORLB 13
#define MOTORRA 7
#define MOTORRB 8
#endif

// rosserial libraries
#include <ros.h>
#include <std_msgs/Int32.h>

void moveRobot(byte ena, byte enb, boolean la, boolean lb, boolean ra, boolean rb){
  analogWrite(ENA, ena);
  analogWrite(ENB, enb);
  digitalWrite(MOTORLA, la);
  digitalWrite(MOTORLB, lb);
  digitalWrite(MOTORRA, ra);
  digitalWrite(MOTORRB, rb);
}

void moveForward(int delayTime){
  moveRobot(255, 255, 1, 0, 1, 0);
  delay(delayTime);
  stopMotors(1);
}

void moveBackwards(int delayTime){
  moveRobot(255, 255, 0, 1, 0, 1);
  delay(delayTime);
  stopMotors(1);
}

void stopMotors(int delayTime){
  moveRobot(0, 0, 0, 0, 0, 0);
  delay(delayTime);
}

void initializeMotors(){
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOTORLA, OUTPUT);
  pinMode(MOTORLB, OUTPUT);
  pinMode(MOTORRA, OUTPUT);
  pinMode(MOTORRB, OUTPUT);
  stopMotors(1);
}

void setSpeedLeftRight(byte ena, byte enb){
  analogWrite(ENA, ena);
  analogWrite(ENB, enb);
}


void setSpeedLeft(byte ena){
  analogWrite(ENA, ena);
}

void setSpeedRight(byte enb){
  analogWrite(ENB, enb);
}

// ros related codes
ros::NodeHandle nodeHandle;

void moveForwardCallback(const std_msgs::Int32& delayTime){
  moveForward(delayTime.data); // this action gets the contents which is int compatible
}

//ros subscriber
ros::Subscriber<std_msgs::Int32> moveForwardSub("move_forward", &moveForwardCallback);

// main codes
void setup() {
  initializeMotors();
  nodeHandle.initNode();
  nodeHandle.subscribe(moveForwardSub);
}

void loop() {
  // put your main code here, to run repeatedly:
  //moveForward(500);
  //stopMotors(1000);
 
  // comment the line below to test if the motor is flipped
  //moveBackwards(500);
  //stopMotors(1000);

  nodeHandle.spinOnce();
  delay(1);
}


  
  
  
