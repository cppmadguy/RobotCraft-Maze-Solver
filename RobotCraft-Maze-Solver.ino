#include <Encoder.h>
// MOTOR1 RIGHT, MOTOR2 LEFT
/***********SENSOR SECTION*******************************/
#include <SharpDistSensor.h>
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
const float rad = 0.016;
const float axisL = 0.09;
const int Cpr = 8250;

void cmd_vel(float &desired_linear, float &desired_angular) {
  desired_linear = 0.0625;  // m/s
  desired_angular = 0.0;  // rad/sec
}

void convert_velocities_to_wheel(float linear_velocity, float angular_velocity, float &wheel_left, float &wheel_right) {
  wheel_left = (linear_velocity - ((axisL / 2) * angular_velocity)) / rad;
  wheel_right = (linear_velocity + ((axisL / 2) * angular_velocity)) / rad;
}

void convert_wheel_to_velocities(float wheel_left, float wheel_right, float &linear_velocity, float &angular_velocity) {
  linear_velocity = (rad / 2) * (wheel_left + wheel_right);
  angular_velocity = ((wheel_right - wheel_left) * rad) / axisL;
}

struct Robotstate {
  float x_pose, y_pose, angle_pose, x_vel, y_vel, angle_vel;
};

/* This function calculate the velocities and pose of the robot using the encoders */
struct Robotstate *const calcPose(int EncoderValL, int EncoderValR, float dt) {
  static Robotstate Rob_Oldstate{ 0 };
  Robotstate Rob_Newstate{ 0 };
  Rob_Newstate.x_vel = ((2.0 * PI * rad) * (EncoderValL + EncoderValR)) / (2.0 * dt *  Cpr);
  Rob_Newstate.angle_vel = ((2.0 * PI * rad) * (EncoderValR - EncoderValL)) / (axisL * dt * Cpr);
  Rob_Newstate.angle_pose = atan2(sin(Rob_Oldstate.angle_pose + Rob_Newstate.angle_vel * dt), cos(Rob_Oldstate.angle_pose + Rob_Newstate.angle_vel * dt));
  Rob_Newstate.x_pose = Rob_Oldstate.x_pose + Rob_Newstate.x_vel * cos(Rob_Newstate.angle_pose) * dt;
  Rob_Newstate.y_pose = Rob_Oldstate.y_pose + Rob_Newstate.x_vel * sin(Rob_Newstate.angle_pose) * dt;

  Rob_Oldstate = Rob_Newstate;
  return &Rob_Oldstate;
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

void rotateMotorsForward(int speed) {
  setMotorDirection(motor1DIR, false);
  setMotorDirection(motor2DIR, false);

  setMotorSpeed(motor1PWM, speed-15);
  setMotorSpeed(motor2PWM, speed);
}

void rotateMotorsBackwards(int speed) {
  setMotorDirection(motor1DIR, true);
  setMotorDirection(motor2DIR, true);

  setMotorSpeed(motor1PWM, speed);
  setMotorSpeed(motor2PWM, speed);
}
void turnLeft(int speed) {
  setMotorDirection(motor1DIR, false);
  setMotorDirection(motor2DIR, true);
  setMotorSpeed(motor1PWM, speed);
  setMotorSpeed(motor2PWM, speed);
}

void turnRight(int speed) {
  setMotorDirection(motor1DIR, true);
  setMotorDirection(motor2DIR, false);
  setMotorSpeed(motor1PWM, speed);
  setMotorSpeed(motor2PWM, speed);
}
/*************************ENCODERS***************************/
Encoder encL(2, 3);    // Left wheel encoder pins
Encoder encR(18, 19);  // Right wheel encoder pins

unsigned long interval = 1000;  // Interval in milliseconds (100ms = 10Hz)

/* return the number of pulses since the last call of the function */
struct Robotstate *const checkEncoderValues(unsigned long dt) {

  static long prevEncoderValueL = 0;  // Previous left wheel encoder value
  static long prevEncoderValueR = 0;  // Previous right wheel encoder value
  static bool first = 1;

  long encoderDiffL;
  long encoderDiffR;

  if (first) {
    first = 0;
    encoderDiffL = -encL.read();
    encoderDiffR = encR.read();
    prevEncoderValueL = encoderDiffL;
    prevEncoderValueR = encoderDiffR;

  } else {
    // Calculate the difference in encoder values
    long currentEncoderValueL = -encL.read();
    long currentEncoderValueR = encR.read();

    encoderDiffL = currentEncoderValueL - prevEncoderValueL;
    encoderDiffR = currentEncoderValueR - prevEncoderValueR;

    prevEncoderValueL = currentEncoderValueL;
    prevEncoderValueR = currentEncoderValueR;
  }

  Robotstate *Robstate = calcPose(encoderDiffL, encoderDiffR, float(dt) * 1E-3);
  Serial.print("DeltaT: ");
  Serial.println(dt);
  Serial.print("Robot position x:");
  Serial.println(Robstate->x_pose);
  Serial.print("Robot position y:");
  Serial.println(Robstate->y_pose);

  //Print the encoder differences in both wheels
  Serial.print("Left wheel encoder difference: ");
  Serial.println(encoderDiffL);

  Serial.print("Right wheel encoder difference: ");
  Serial.println(encoderDiffR);
  return Robstate;
}
void setup() {

  Serial.begin(9600);  // Initialize serial communication

  initializeMotors(); // Initialize motor pins
  
  rotateMotorsForward(200);

  for (byte i = 0; i < nbSensors; i++) {
    sensorArray[i].setModel(SharpDistSensor::GP2Y0A21F_5V_DS);  // Set the sensor model
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long previousMillis = 0;  // Previous time of the last iteration
  unsigned long currentMillis = millis();
  unsigned long deltaT = currentMillis - previousMillis;

  if(sensorArray[1].getDist() <= 200){
    rotateMotorsBackwards(0);
  } else {
    rotateMotorsForward(200);
  }

  if (deltaT < interval) return;
  previousMillis = currentMillis;
  // Executed every interval

  Robotstate *robstate = checkEncoderValues(deltaT);
  float real_wheel_left, real_wheel_right, desired_wheel_left, desired_wheel_right, desired_linear, desired_angle, linear_error, angle_error;
  cmd_vel(desired_linear, desired_angle);
  convert_velocities_to_wheel(robstate->x_vel, robstate->angle_vel, real_wheel_left, real_wheel_right);
  convert_velocities_to_wheel(desired_linear, desired_angle, desired_wheel_left, desired_wheel_right);
  convert_wheel_to_velocities(desired_wheel_left - real_wheel_left, desired_wheel_right - real_wheel_right, linear_error, angle_error);
  Serial.print("real linear speed: "); Serial.println(robstate->x_vel, 5);
  Serial.print("linear speed error: "); Serial.println(linear_error, 5);
  Serial.print("x position: "); Serial.println(robstate->x_pose);
  Serial.print("y position: "); Serial.println(robstate->y_pose);
}
