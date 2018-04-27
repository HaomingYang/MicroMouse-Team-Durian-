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
double u_ang, error;
double zero = -0.43434343434;
double P = 0.25;
double I = 0.0;
double D = 0.0003;
PID angPID(&error, &u_ang, &zero, P, I, D, DIRECT);

// delay
int r_count = 0;
int l_count = 0;
int c_count = 0;

void setup() {
  Serial.begin(9600);
  hardwareSetup();
  error = 0;
  angPID.SetOutputLimits(-1.0, 1.0);
  angPID.SetSampleTime(10);
  angPID.SetMode(AUTOMATIC);
}

void loop() {
  // Read sensor data
  left_dist = getDistanceLeft();
  right_dist = getDistanceRight();
  center_dist = getDistanceCenter();

  velocity_linear = getLinearVelocity();
  velocity_angular = getAngularVelocity(); // > 0 when right > left

  ////////////////////////////////////
  // Your changes should start here //
  ////////////////////////////////////

  if (right_dist > 22.0) {
    r_count = 1;
  }
  if (left_dist > 22.0) {
    l_count = 1;
  }
  if (center_dist < 15.5) {
    c_count = 1;
  }
  if (c_count && l_count) {
    left_dist += 100;
  }
  if (c_count && r_count) {
    right_dist += 100;
  }
  if (c_count && center_dist > 18) {
    c_count = 0;
    l_count = 0;
    r_count = 0;
  }

  error = velocity_angular + right_dist / 5.5 - left_dist / 5.5;

  angPID.Compute();

  applyPowerLeft((center_dist - 2.0) / 100 - u_ang);
  applyPowerRight((center_dist - 2.0) / 100 + u_ang);

  // Print debug info every 500 loops
  if (count % 500 == 0 && millis() > 1000) {
    // Serial.print(" V: ");
    // Serial.print(velocity_linear / 100.0);
    Serial.print(" V_A: ");
    Serial.print(error);
    Serial.print(" LEFT: ");
    Serial.print(left_dist);
    Serial.print(" CENTER: ");
    Serial.print(center_dist);
    Serial.print(" RIGHT: ");
    Serial.print(right_dist);
    Serial.print(" CRCTION: ");
    Serial.print(u_ang);
    Serial.println();
  }
  count++;

  checkEncodersZeroVelocity();
  updateDistanceSensors();
}
