/*
  JRD Propulsion RP2040-BNO055 Example
  By: Aryan Kapoor
  Last updated on: 12/21/22
*/


// Importing libraries
#include <Arduino.h>
#include <BNO055.h>


// Initialization and definitions
BNO055_sensor BNO055;
double gyro_x, gyro_y, gyro_z
   

// the setup routine runs once at startup
void setup() 
{
  Serial.begin(115200);
  BNO055.begin(); // Initialization function of IMU
}


// Main loop function
void loop()
{
  BNO055.get_gyro(&gyro_x, &gyro_y, &gyro_z);
  
  Serial.print("X: ");
  Serial.print(gyro_x, 8);
  Serial.print("  Y: ");
  Serial.print(gyro_y, 8);
  Serial.print("  Z: ");
  Serial.println(gyro_z, 8);
  delay(10);

