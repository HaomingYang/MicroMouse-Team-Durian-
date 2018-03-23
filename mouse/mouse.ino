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

  float left_power = 0.3;
  float right_power = 0.3;

  float u_current = u(sr_error),
        pleft = left_power + u_current,
        pright = right_power - u_current;
        
  //applyPowerLeft(pleft);
  //applyPowerRight(pright);

  applyPowerLeft(pleft);
  applyPowerRight(pright);

  // Print debug info every 500 loops
  if (count % 500 == 0) {
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
    Serial.print(" pl");
    Serial.print(pleft);
    Serial.print(" pr");
    Serial.print(pright);
    
    Serial.println();

  }
  count++;

  checkEncodersZeroVelocity();
  updateDistanceSensors();
  setlight();
}

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
float u(float (*err_f)())
{
  static float current_correction_rate = 0.0;
  
  static unsigned long last_fix = 0;
  unsigned long current_time;
  
  current_time = millis();
  if(current_time - last_fix > 100){
    current_correction_rate += sr_error() * 0.005;
    last_fix = current_time;
  }


  return current_correction_rate;
}

//return:
//-: need to go left
//+: need to go right
float sr_error()
{
  return velocity_angular+0.15;
}
