
#include <SharpDistSensor.h>
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

// MOTOR1 RIGHT, MOTOR2 LEFT
/***********SENSOR SECTION*******************************/

#define nbSensors 3
#define leftsensorpin A2
#define middlesensorpin A3
#define rightsensorpin A4

//Array of distance sensor read by sensors
uint16_t distArray[nbSensors];
// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;
SharpDistSensor sensorArray[] = {
  SharpDistSensor(leftsensorpin, medianFilterWindowSize),    // First Sensor using pin A2
  SharpDistSensor(middlesensorpin, medianFilterWindowSize),  // Second Sensor using pin A3
  SharpDistSensor(rightsensorpin, medianFilterWindowSize)    // Third Sensor using pin A4
};

/***********Displacement Calculations**************************/
#define wheel_radius 0.016
#define wheel_distance 0.09
#define pulse_per_rev 8100

float cmd_linear = 0;
float cmd_angular = 0;

void cmd_vel(float &desired_linear, float &desired_angular) {
  desired_linear = cmd_linear;  // m/s
  desired_angular = cmd_angular;  // rad/sec
}

void convert_velocities_to_wheel(float linear_velocity, float angular_velocity, float &wheel_left, float &wheel_right) {
  wheel_left = (linear_velocity - ((wheel_distance / 2) * angular_velocity)) / wheel_radius;
  wheel_right = (linear_velocity + ((wheel_distance / 2) * angular_velocity)) / wheel_radius;
}

struct RobotState {
  float x_pose, y_pose, angle_pose, x_vel, y_vel, angle_vel;
};

/* This function calculate the velocities and pose of the robot using the encoders */
struct RobotState *const calcPose(float encoderValL, float encoderValR, float dt) {
  static RobotState oldstate;
  RobotState newstate;
  //newstate.x_vel = ((2.0 * PI * wheel_radius) * (encoderValL + encoderValR)) / (2.0 * dt *  pulse_per_rev);
  newstate.x_vel = 2.0 * PI * wheel_radius * (encoderValL + encoderValR) / (pulse_per_rev * 2.0 * dt); 
  newstate.angle_vel = 2.0 * PI * wheel_radius * (encoderValR - encoderValL) / (pulse_per_rev * wheel_distance * dt);
  newstate.angle_pose = atan2(sin(oldstate.angle_pose + newstate.angle_vel * dt), cos(oldstate.angle_pose + newstate.angle_vel * dt));
  newstate.x_pose = oldstate.x_pose + newstate.x_vel * cos(newstate.angle_pose) * dt;
  newstate.y_pose = oldstate.y_pose + newstate.x_vel * sin(newstate.angle_pose) * dt;
  oldstate = newstate;
  return &oldstate;
}

/*************MOTORS********************/
// Motor 1 pins
const int motor1DIR = 5;
const int motor1PWM = 4;

// Motor 2 pins
const int motor2DIR = 6;
const int motor2PWM = 9;

void initializeMotors() {
  pinMode(motor1DIR, OUTPUT);
  pinMode(motor1PWM, OUTPUT);

  pinMode(motor2DIR, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
}

void setMotorSpeed(int motorPWM, int speed) {
  analogWrite(motorPWM, speed);
}

void setMotorDirection(int motorDIR, bool forward) {
  digitalWrite(motorDIR, forward ? HIGH : LOW);
}

void rotateMotors(int speedleft, int speedright){
  if(speedleft > 0){
    setMotorDirection(motor2DIR, false);
    setMotorSpeed(motor2PWM, min(speedleft, 255));
  } else {
    setMotorDirection(motor2DIR, true);
    setMotorSpeed(motor2PWM, min(-speedleft, 255));
  }
  if(speedright > 0){
    setMotorDirection(motor1DIR, false);
    setMotorSpeed(motor1PWM, min(speedright, 255));
  } else {
    setMotorDirection(motor1DIR, true);
    setMotorSpeed(motor1PWM, min(-speedright, 255));
  }
}

/*************************ENCODERS***************************/
Encoder encR(2, 3);    // Right wheel encoder pins
Encoder encL(18, 19);  // Left wheel encoder pins

/* return the number of pulses since the last call of the function */
struct RobotState *const checkEncoderValues(unsigned long dt) {

  static long prevEncoderValueL = 0;  // Previous left wheel encoder value
  static long prevEncoderValueR = 0;  // Previous right wheel encoder value
  static bool first = 1;

  long encoderDiffL;
  long encoderDiffR;

  if (first) {
    first = 0;
    encoderDiffL = encL.read();
    encoderDiffR = -encR.read();
    prevEncoderValueL = encoderDiffL;
    prevEncoderValueR = encoderDiffR;

  } else {
    // Calculate the difference in encoder values
    long currentEncoderValueL = encL.read();
    long currentEncoderValueR = -encR.read();

    encoderDiffL = currentEncoderValueL - prevEncoderValueL;
    encoderDiffR = currentEncoderValueR - prevEncoderValueR;

    prevEncoderValueL = currentEncoderValueL;
    prevEncoderValueR = currentEncoderValueR;
  }
  RobotState *Robstate = calcPose(encoderDiffL, encoderDiffR, float(dt) * 1E-3);
  return Robstate;
}

struct Errors {
  float integral = 0;
  float last = 0;
};

void updateError(struct Errors* errors, float currentError){
  errors->integral += currentError;
  errors->last = currentError;
}

#define Kp 0.7f
#define Ki 0.1f

void PidControl(unsigned long deltaT){
  static Errors left_errors;
  static Errors right_errors;

  RobotState *robstate = checkEncoderValues(deltaT);
  float real_wheel_left, real_wheel_right, desired_wheel_left, desired_wheel_right, desired_linear, desired_angle;
  cmd_vel(desired_linear, desired_angle);
  convert_velocities_to_wheel(robstate->x_vel, robstate->angle_vel, real_wheel_left, real_wheel_right);
  convert_velocities_to_wheel(desired_linear, desired_angle, desired_wheel_left, desired_wheel_right);

  float left_error = desired_wheel_left - real_wheel_left;
  float right_error = desired_wheel_right - real_wheel_right;
  updateError(&left_errors, left_error);
  updateError(&right_errors, right_error);
  float Gleft = Kp * left_error + Ki * left_errors.integral * float(deltaT);
  float Gright = Kp * right_error + Ki * right_errors.integral * float(deltaT);
  rotateMotors(Gleft, Gright);
}

/***** ROS Integration *****/

ros::NodeHandle  nh;

void receive_cmd(const geometry_msgs::Twist& twist_msg){
  cmd_linear = twist_msg.linear.x;
  cmd_angular = twist_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("RobVel", receive_cmd);

void setup() {
  initializeMotors(); // Initialize motor pins
  rotateMotors(0, 0);
  // Serial.begin(57600);  // Initialize serial communication
  for (byte i = 0; i < nbSensors; i++) {
    sensorArray[i].setModel(SharpDistSensor::GP2Y0A21F_5V_DS);  // Set the sensor model
  }
  encL.readAndReset();
  encR.readAndReset();

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);
}

unsigned long interval = 1000; // Interval in milliseconds (100ms = 10Hz)

void loop() {
  
  // put your main code here, to run repeatedly:
  static unsigned long previousMillis = 0;  // Previous time of the last iteration
  unsigned long currentMillis = millis();
  unsigned long deltaT = currentMillis - previousMillis;
  if (deltaT < interval) return;
  previousMillis = currentMillis;

  // Executed every interval
  PidControl(deltaT);

  nh.spinOnce();
}
