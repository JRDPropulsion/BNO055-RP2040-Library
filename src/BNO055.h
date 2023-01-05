/*
  BNO055.h - Library for retrieving data from the BNO055.
  Created by Aryan Kapoor, 11/24/22.
  Released into the public domain.
*/


#ifndef BNO055_h
#include "Arduino.h"
#include "hardware/i2c.h"


#define BNO055_I2C_ADDR  0x28
#define BNO055_CHIP_ID_ADDR  0x00


// Defines BNO055_sensor class
class BNO055_sensor
{
    public:
        void begin();
        double get_accel(double *px, double *py, double *pz);
        double get_gyro(double *px, double *py, double *pz);
        double get_euler(double *px, double *py, double *pz);

    private:

};

#endif




