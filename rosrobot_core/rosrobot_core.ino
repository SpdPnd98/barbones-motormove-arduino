#define USE_USBCON

// change flipped to match robot movement
#define FLIPPED true

#if !FLIPPED
#define ENA 5
#define ENB 6
#define MOTORLA 8
#define MOTORLB 9
#define MOTORRA 19
#define MOTORRB 18

#elif FLIPPED
#define ENA 5
#define ENB 6
#define MOTORLA 8
#define MOTORLB 9
#define MOTORRA 18
#define MOTORRB 19
#endif

// rosserial libraries
#include <ros.h>
//#include <geometry_msgs/Twist.h> // has to be customized in order to work 
#include <std_msgs/String.h>

// IMU libraries
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// IMU variables
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
bool imu_mask = false;
std_msgs::String imu_msg;
long publisher_timer = millis() + 100;

// global variables
float w_r = 0, w_l = 0;

//wheel_rad is the wheel radius ,wheel_sep is the separation between the wheels
float wheel_rad = 0.0325, wheel_sep = 0.295;

byte lowSpeed = 200;
byte highSpeed = 50;
float speed_ang = 0, speed_lin = 0;

// ros related codes
ros::NodeHandle nodeHandle;

void cmd_vel_callback(const std_msgs::String& msg) {
  left_right_speed_algorithm(msg); // runs an algorithm to calculate what the command speed is in motor terms
}

//ros subscriber
ros::Subscriber<std_msgs::String> cmd_vel_sub("/cmd_vel_simplified", &cmd_vel_callback);

//ros publisher
ros::Publisher raw_imu("raw_imu", &imu_msg);

void moveRobot(byte ena, byte enb, boolean la, boolean lb, boolean ra, boolean rb) {
  analogWrite(ENA, ena);
  analogWrite(ENB, enb);
  digitalWrite(MOTORLA, la);
  digitalWrite(MOTORLB, lb);
  digitalWrite(MOTORRA, ra);
  digitalWrite(MOTORRB, rb);
}

void stopMotors(int delayTime) {
  moveRobot(0, 0, 0, 0, 0, 0);
  delay(delayTime);
}

void initializeMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOTORLA, OUTPUT);
  pinMode(MOTORLB, OUTPUT);
  pinMode(MOTORRA, OUTPUT);
  pinMode(MOTORRB, OUTPUT);
  stopMotors(1);
}

void setSpeedLeftRight() {
  // right motor
  if (w_r > 0) { //move forward
    digitalWrite(MOTORRA, HIGH);
    digitalWrite(MOTORRB, LOW);
  }
  else if (w_r < 0) { //move backward
    digitalWrite(MOTORRA, LOW);
    digitalWrite(MOTORRB, HIGH);
  } else { //don't move
    digitalWrite(MOTORRA, LOW);
    digitalWrite(MOTORRB, LOW);
  }
  if (abs(w_r) <= 25.5) {
    analogWrite(ENA, (int)abs(w_r * 10));
  } else {
    analogWrite(ENA, 255);
  }

  // left motor
  if (w_l > 0) { //move forward
    digitalWrite(MOTORLA, HIGH);
    digitalWrite(MOTORLB, LOW);
  }
  else if (w_l < 0) { //move backward
    digitalWrite(MOTORLA, LOW);
    digitalWrite(MOTORLB, HIGH);
  } else { //don't move
    digitalWrite(MOTORLA, LOW);
    digitalWrite(MOTORLB, LOW);
  }
  if (abs(w_l) <= 25.5) {
    analogWrite(ENB, (int)abs(w_l * 10));
  } else {
    analogWrite(ENB, 255);
  }
}

void left_right_speed_algorithm(std_msgs::String msg) {
  // the following are code found online, kept as reference (its probably ground truth)
  //speed_ang = msg.angular.z;
  //speed_lin = msg.linear.x;
  String str_msg = msg.data;
  speed_lin = str_msg.substring(0,str_msg.indexOf(' ')).toFloat();
  speed_ang = str_msg.substring(str_msg.indexOf(' ') + 1).toFloat();
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}

void setSpeedLeft(byte ena) {
  analogWrite(ENA, ena);
}

void setSpeedRight(byte enb) {
  analogWrite(ENB, enb);
}

// IMU functions
void setupIMU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  //Serial.begin(38400);
  // initialize device
  // Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  // Serial.println("Testing device connections...");
  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  if (!accelgyro.testConnection()) {
    // these are impossible values, we must check from nodes reading this value
    ax = 32751;
    ay = 32751;
    az = 32751;
    gx = 32751;
    gy = 32751;
    gz = 32751;
    imu_mask = true;
  } else {
    imu_mask = false;
    delay(100);
  }
}

void retryIMU() {
  setupIMU();
}

void readIMU() {
  if (!imu_mask) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    String data_string = "A" + String(ax) + "B" + String(ay) + "C" + String(az) + "D"
                         + String(gx) + "E" + String(gy) + "F" + String(gz) + "G";
    int arr_len = data_string.indexOf("G") + 2;
    char data_final[arr_len + 1];
    data_string.toCharArray(data_final, arr_len + 1);
    if (publisher_timer < millis()) {
      imu_msg.data = data_final;
      publisher_timer = millis() + 200;
      raw_imu.publish(&imu_msg);
    }
  } else if (imu_mask) {
    retryIMU();
  }
}

// main codes
void setup() {
  initializeMotors();
  setupIMU();
  nodeHandle.getHardware()->setBaud(115200);
  nodeHandle.initNode();
  nodeHandle.subscribe(cmd_vel_sub);
  nodeHandle.advertise(raw_imu);
}

void loop() {
  setSpeedLeftRight();
  readIMU();
  nodeHandle.spinOnce();
  delay(25); // maybe ROS cannot interpret fast enough
}
