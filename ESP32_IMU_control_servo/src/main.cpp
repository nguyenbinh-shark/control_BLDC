#include <Arduino.h>
#include "Wire.h"
#include "IMU_HUST.h"
#include <ESP32Servo.h>
IMU_HUST mpu6050(Wire);
const int SERVO_PINS[4] = {9, 10, 11, 12};
Servo servos[4];
int servoAngles[4] = {130, 130, 50, 50};//132;132;50;50

//PIDController pid_stb{.P = 30, .I = 100, .D = 1, .ramp = 100000, .limit = 7};

void setup() {
Serial.begin(500000); 
  Wire.begin(4, 5, 400000UL); 
  mpu6050.init();
pinMode(9,OUTPUT);
pinMode(10,OUTPUT);
pinMode(11,OUTPUT);
pinMode(12,OUTPUT);
for (int i = 0; i < 4; i++) {
    ESP32PWM::allocateTimer(i);
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].write(servoAngles[i]);
  }
}

void loop() {
 mpu6050.update();
float angle1=mpu6050.angle[1];
  // Hiển thị dạng đồ thị – mỗi giá trị cách nhau bởi TAB
  Serial.print("A");  // Roll góc y
  Serial.print(angle1);Serial.print('\n');
  // Roll góc y
  
  delay(50);  // Cập nhật mỗi 50ms (20 lần/giây)
}
