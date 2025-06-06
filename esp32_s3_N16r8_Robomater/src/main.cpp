
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
// #define   _MON_TARGET 0b1000000  // monitor target value
// #define   _MON_VOLT_Q 0b0100000  // monitor voltage q value
// #define   _MON_VOLT_D 0b0010000  // monitor voltage d value
// #define   _MON_CURR_Q 0b0001000  // monitor current q value - if measured
// #define   _MON_CURR_D 0b0000100  // monitor current d value - if measured
// #define   _MON_VEL    0b0000010  // monitor velocity value
// #define   _MON_ANGLE  0b0000001  // monitor angle value
// control algorithm parameters
// stabilisation pid
PIDController pid_stb{ 4, 0.3,  1,  1000,  9};
// velocity pid
PIDController pid_vel{ 0.01, 0.03, 0,  10000,  _PI / 10};
// velocity control filtering
LowPassFilter lpf_pitch_cmd{ 0.07};
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{ 0.5};
LowPassFilter lpf_steering{ 0.1};
float steering = 0,voltage_control;
float throttle = 0;
float max_throttle = 20; // 20 rad/s
float max_steering = 1; // 1 V
int state = 1; // 1 on / 0 off

// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(35, 36, 0, 20);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(40, 41, 42, 2);

float target0 = 0;
float target1 = 0;  
float angle1;
 Commander command = Commander(Serial);
void doTarget(char* cmd) { 
  command.scalar(&target0, cmd);
  Serial.print("Target Torque 0: ");
  Serial.println(target0);  // Debugging line to check the value
}
void cntStab(char* cmd) {  command.pid(&pid_stb, cmd);}

void doTarget1(char* cmd) { 
  command.scalar(&target1, cmd);
  Serial.print("Target Torque 1: ");
  Serial.println(target1);  // Debugging line to check the value
}
void doAngle(char* cmd) {
  command.scalar(&angle1, cmd); 
}
void setup() {
    Serial.begin(500000);
// initialise magnetic sensor hardware
  Wire.begin(16, 15, (uint32_t)400000);
  Wire.setClock(400000);
//Thêm một bộ mã hóa từ tính mới
  Wire1.setClock(400000); 
  Wire1.begin(17, 18, (uint32_t)400000);

  sensor.init(&Wire);
  sensor1.init(&Wire1);

  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // power supply voltage [V]
  driver.voltage_power_supply = 12;  
  driver.init();  
  // link the motor and the driver
  motor.linkDriver(&driver);  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;  
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.010;
  motor.PID_velocity.I = 0.005;  
//  motor.PID_velocity.D = 0.01;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 12;  
  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;  
  // angle P controller
  motor.P_angle.P = 30;  //Giá trị P của vị trí PID
  // maximal velocity of the position control
  motor.velocity_limit = 50;
  motor.PID_velocity.output_ramp = 1200;
  motor.LPF_velocity.Tf = 0.01f;//Bộ lọc này giúp loại bỏ tiếng ồn và rung động tần số cao từ động cơ, giúp kiểm soát tốc độ ổn định hơn
//Động cơ mới
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::torque;
  motor1.PID_velocity.P = 0.01;
  motor1.PID_velocity.I = 0.005;
//  motor1.PID_velocity.D = 0.0011;
  motor1.voltage_limit = 12;
  motor1.LPF_velocity.Tf = 0.01f;
  motor1.P_angle.P = 30;
//  motor1.P_angle.I = 0.08;
  motor1.velocity_limit = 50;
  motor1.PID_velocity.output_ramp = 1200;/* Việc điều chỉnh giá trị này có thể ảnh hưởng 
                                          đến hiệu suất tăng tốc và giảm tốc của động cơ. 
                                          Giá trị cao hơn sẽ khiến động cơ tăng tốc và giảm tốc nhanh hơn,
                                          nhưng có thể gây ra rung động hoặc dòng điện tăng đột biến.*/
  motor1.LPF_velocity.Tf = 0.01f;

  // use monitoring with serial
  // comment out if not needed
  
  //motor.useMonitoring(Serial);
  // motor.monitor_variables =  _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
  Serial.println("Initializing motor 0...");
  motor.init();
  motor.initFOC();
  Serial.println("Motor 0 initialized!");

  // initialize motor
  // align sensor and start FOC

  //motor1.useMonitoring(Serial);
  // motor1.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
  Serial.println("Initializing motor 1...");
  motor1.init();
  motor1.initFOC();
  Serial.println("Motor 1 initialized!");

  motor.monitor_downsample = 10; // default 10

      command.add('X', cntStab, "pid stab");
      command.add('A' , doAngle  , "set target angle");
      command.add('R', doTarget , "target voltage");  
      command.add('L', doTarget1, "target voltage1");  
  Serial.println(F("Động cơ đã sẵn sàng"));
  Serial.println(F("Gửi lệnh góc Axx, tốc độ Vxx, mô-men xoắn Txx:"));
  _delay(1000);
}
void loop() {
  // main FOC algorithm function
    motor.loopFOC();
    motor1.loopFOC();

    motor.move();
    motor1.move(); 
    //----------------stars------------------
    if (!state) { // if balancer disabled
    motor.target = 0;
    motor1.target = 0;
  } else { // when IMU has received the package
    // read pitch from the IMU
    float pitch =  angle1- 1.4;
    // calculate the target angle for throttle control
    //float target_pitch = lpf_pitch_cmd(pid_vel((motor.shaft_velocity + motor1.shaft_velocity) / 2 - lpf_throttle(throttle)));
    // calculate the target voltage
    voltage_control = pid_stb(0- pitch);
    
    // filter steering
    float steering_adj = lpf_steering(steering);
    // set the tergat voltage value
    motor.target = voltage_control + steering_adj;
    motor1.target = voltage_control - steering_adj;
  }
  //-------------------end----------------
    motor.monitor();
    command.run();
    // Serial.print("r");
//      Serial.print(motor.target);
//      Serial.print(" ");
      //Serial.println(voltage_control);
 }