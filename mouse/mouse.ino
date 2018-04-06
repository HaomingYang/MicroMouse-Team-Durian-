#include "pins.h"

#include <VL6180X.h>
#include <Wire.h>

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

void setup() {
  Serial.begin(9600);
  hardwareSetup();
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

  float left_power = 0.7;
  float right_power = 0.7;

  float u_current = u(sr_error),
        pleft = left_power + u_current,
        pright = right_power - u_current;
        
  //applyPowerLeft(pleft);
  //applyPowerRight(pright);

  applyPowerLeft(pleft);
  applyPowerRight(pright);

  // Print debug info every 500 loops
  if (count % 500 == 0) {
    /*
    Serial.print(" vlin");
    Serial.print(velocity_linear / 100.0);
    Serial.print(" vang");
    Serial.print(velocity_angular);
    Serial.print(" ldist");
    Serial.print(left_dist);
    Serial.print(" cdist");
    Serial.print(center_dist);
    Serial.print(" rdist");
    Serial.print(right_dist);
    Serial.print(" ucurr");
    Serial.print(u_current);
    */

    if(millis()> 1000)
      Serial.print(velocity_angular);
    Serial.println();

  }
  count++;

  checkEncodersZeroVelocity();
  updateDistanceSensors();
  setlight();
}

//va+ = blue
//va- = yellow
void setlight()
{
  if(sr_error() < 0)
  {
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, LOW);
  }
  else
  {
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, HIGH);
  }

}


//u(t)
//return:
//- : add power to right engine, turn left
//+ : add power to left engine, turn right

const float P = 0.05, I = 0, D = 0.01;
const unsigned long MIN_FIX_INTERVAL = 100;

float u(float (*err_f)())
{
  static float current_correction_rate = 0.0, last_error = sr_error();
  
  static unsigned long last_fix = 0;
  unsigned long current_time;
  
  current_time = millis();
  float error = sr_error();
  
  if(current_time - last_fix > 100){
    /* update current correction rate */

    float pfix = P * error,
          dfix = D * (error - last_error);
    current_correction_rate += pfix + dfix;

    last_error = error;
    last_fix = current_time;
  }
  
  return current_correction_rate;
}

//return:
//-: need to go left
//+: need to go right
float sr_error()
{
  return velocity_angular;
}
