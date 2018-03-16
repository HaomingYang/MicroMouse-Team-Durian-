#include "pins.h"

#include <VL6180X.h>
#include <Wire.h>
#include <PID_v1.h>

// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = false;
const boolean INVERT_MOTOR_RIGHT = true;

// Loop count, used for print statements
int count = 0;

// Sensor states
float velocity_angular = 0;
float velocity_linear = 0;
float left_dist;
float right_dist;
float center_dist;

// PID control
double v, u_lin, u_ang, error, v_sp;
double zero = 0;
double P = .07;
double I = 0;
double D = 0;
PID linPID(&v, &u_lin, &v_sp, P, I, D, P_ON_M, DIRECT);
PID angPID(&error, &u_ang, &zero, P, I, D, P_ON_M, DIRECT);


void setup() {
  Serial.begin(9600);
  hardwareSetup();
  v_sp = 100;
  error = 0;
  angPID.SetOutputLimits(-1, 1);
  linPID.SetOutputLimits(0, 1);
  linPID.SetMode(AUTOMATIC);
  angPID.SetMode(AUTOMATIC);
}

void loop() {
  // Read sensor data
  left_dist = getDistanceLeft();
  right_dist = getDistanceRight();
  center_dist = getDistanceCenter();

  velocity_linear = getLinearVelocity();
  velocity_angular = getAngularVelocity();

  ////////////////////////////////////
  // Your changes should start here //
  ////////////////////////////////////
  error = velocity_angular;
  v = velocity_linear;

  linPID.Compute();
  angPID.Compute();
  //
  // if (error < 0) {
  //   u_ang *= -1;
  // }

  applyPowerLeft(.5 - u_ang);
  applyPowerRight(.5 + u_ang);

  // Print debug info every 500 loops
  if (count % 500 == 0) {
    Serial.print(velocity_linear / 100.0);
    Serial.print(" ");
    Serial.print(velocity_angular);
    Serial.print(" ");
    Serial.print(left_dist);
    Serial.print(" ");
    Serial.print(center_dist);
    Serial.print(" ");
    Serial.print(right_dist);
    Serial.println();
    Serial.print(u_ang);
    Serial.println();
    Serial.print(u_lin);
    Serial.println();
  }
  count++;

  checkEncodersZeroVelocity();
  updateDistanceSensors();
}
