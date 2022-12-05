/*
  BNO055.h - Library for retrieving data from the BNO055.
  Created by Aryan Kapoor, 11/24/22.
  Released into the public domain.
*/


#ifndef BNO055_h
#include "Arduino.h"
#include "hardware/i2c.h"


/*
  Defines register addresses for the BNO055.
  Some regesters are not defined to save space.
  Add register addresses down bellow if needed.
 */


#define BNO055_I2C_ADDR  0x28
#define BNO055_CHIP_ID_ADDR  0x00


// Defines BNO055_sensor class
class BNO055_sensor
{
    public:
        BNO055_sensor();
        void begin();
        void calibrate();

        double accel_x();
        double accel_y();
        double accel_z();

        double gyro_x();
        double gyro_y();
        double gyro_z();

        double mag_x();
        double mag_y();
        double mag_z();

        double ang_accel_x();
        double ang_accel_y();
        double ang_accel_z();

    private:

};

#endif




