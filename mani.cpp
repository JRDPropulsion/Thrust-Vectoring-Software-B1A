/*
  JRD Propulsion thrust vector control software (flight version)
  By: Aryan Kapoor
  Last updated on: 12/2/22
*/


/*
  General information and instructions below(Please Read carefully!):

  State indicator:
  0 = IMU calibration
  1 = Startup functions
  2 = Pad idle, waiting for acceleration spike
  3 = Liftoff detected, controlled ascent and data logging
  4 = Descent detected and fire pyro channels
  5 = Nothing, just to get out of state 4

  Servo offsets and ratio:
  - For the servo offsets, change the X and Y values until the mount is straigt when the computer is vertical
  - To tune the servo ratio to tvc mount actuation, mount a MPU6050 on the motor tube with an arduino and calibrate the MPU6050 so that 
  when the computer is completely vertical, the orientation of the MPU6050 reads 0. Then, actuate the TVC mount by comanding to the move
  the servos until the motor tube hits the end of mount. Then, divide the servo actuation value (number of steps it took to reach the end)
  by the MPU6050 reading (at what angle was the TVC mount when it touched the end).

  Required libraries:
  - BNO055.h
  - Orientation.h
  - servo.h
*/


// Importing libraries
#include <Arduino.h>
#include <BNO055.h>
#include <Orientation.h>
//#include <servo.h>


// Initialization
BNO055_sensor BNO055;
Orientation orientation;
uint64_t this_loop = 0;
uint64_t last_update = 0;
uint8_t sys, gyro, accel, mag = 0;

double accel_x, accel_y, accel_z;
double gyro_x, gyro_y, gyro_z;
double orientation_x, orientation_y, orientation_z;


// the setup routine runs once when you press reset:
void setup() 
{
  Serial.begin(9600);
  BNO055.begin();
  this_loop = last_update = micros(); // Set starting time after init/calibration
}


// Main loop function
void loop()
{
  this_loop = micros(); // Get new microsecond timestamp for this loop
  double dt = (double)(this_loop - last_update) / 1000000; // Finds elapsed microseconds since last update, converts to float, and converts to seconds
  last_update = this_loop; // We have updated, set the new timestamp

  BNO055.get_gyro(&gyro_x, &gyro_y, &gyro_z);
  BNO055.get_accel(&accel_x, &accel_y, &accel_z);
  orientation.quaternion_update(gyro_x, gyro_y, gyro_z, dt, &orientation_x, &orientation_y, &orientation_z);

    BNO055.calibrate(&sys, &gyro, &accel, &mag);
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.println(mag, DEC);
  
  //Serial.print("X: ");
  //Serial.print(orientation_x);
  //Serial.print(" Y: ");
  //Serial.print(orientation_y);
  //Serial.print(" Z: ");
  //Serial.println(orientation_z);

  delay(50);
}
