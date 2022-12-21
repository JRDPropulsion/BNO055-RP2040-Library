/*
  BNO055.cpp - Library for retrieving data from the BNO055.
  Created by Aryan Kapoor, 11/25/22.
  Released into the public domain.
*/


#include "Arduino.h"
#include "hardware/i2c.h"
#include "BNO055.h"


// Initialization:
#define I2C_PORT i2c1   // I2C bus
int SDA = 26;   // SDA GPIO pin number
int SCL = 27;   // SCL GPIO pin number


void BNO055_sensor::begin()
{   
    // Configure I2C Communication with BNO055
    _i2c_init(I2C_PORT, 400 * 1000);    // Add an extra "_" at the front if using Arduino core and hardware/i2c
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SDA); 
    delay(5000);        // Add a short delay to help BNO005 boot up

    uint8_t chipID[1];

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, BNO055_CHIP_ID_ADDR, 1, true);
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, chipID, 1, false);

    if(chipID[0] != 0xA0)
    {
        while(1)
        {
            Serial.print("Chip ID Not Correct - Check Connection!");
            delay(5000);
        }
    }

    // Use internal oscillator
    uint8_t data[2];
    data[0] = 0x3F;
    data[1] = 0x40;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

    // Reset all interrupt status bits
    //data[0] = 0x3F;
    //data[1] = 0x01;
    //i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

    // Configure Power Mode
    data[0] = 0x3E;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
    delay(50);

    // Default Axis Configuration
    data[0] = 0x41;
    data[1] = 0x24;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

    // Default Axis Signs
    data[0] = 0x42;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

    // Set units to m/s^2
    data[0] = 0x3B;
    data[1] = 0b0001000;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
    delay(30);

    // Set operation to NDOF
    data[0] = 0x3D;
    data[1] = 0x03;
    //data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
    delay(100);
}

double BNO055_sensor::get_accel(double *px, double *py, double *pz)
{
    uint8_t accel[6];   // Store data from 6 acceleration registers
    int16_t accelX, accelY, accelZ; // Combined 3 axis data
    float accel_x, accel_y, accel_z; // Float type of acceleration data
    uint8_t accel_val = 0x08; // Start register address = 0X08;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &accel_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, accel, 6, false); // I2C read function

    accelX = ((accel[1]<<8) | accel[0]);
    accelY = ((accel[5]<<8) | accel[4]);
    accelZ = ((accel[3]<<8) | accel[2]);

    accel_x = accelX / 100.00;
    accel_y = accelY / 100.00;
    accel_z = accelZ / 100.00;

    *px = accel_x;
    *py = accel_y;
    *pz = accel_z;

    return 0;
}

double BNO055_sensor::get_gyro(double *px, double *py, double *pz)
{
    uint8_t gyro[6];   // Store data from 6 gyro registers
    int16_t gyroX, gyroY, gyroZ; // Combined 3 axis data
    float gyro_x, gyro_y, gyro_z; // Float type of gyro data
    uint8_t gyro_val = 0x14; // Start register address = 0X14;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &gyro_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, gyro, 6, false); // I2C read function

    gyroX = ((gyro[1]<<8) | gyro[0]);
    gyroY = ((gyro[3]<<8) | gyro[2]);
    gyroZ = ((gyro[5]<<8) | gyro[4]);

    gyro_x = (DEG_TO_RAD * (gyroX / 16.0));
    gyro_y = (DEG_TO_RAD * (gyroY / 16.0));
    gyro_z = (DEG_TO_RAD * (gyroZ / 16.0));

    *px = gyro_x;
    *py = gyro_y;
    *pz = gyro_z;

    return 0;
}

double BNO055_sensor::get_euler(double *px, double *py, double *pz)
{
    uint8_t gyro[6];   // Store data from 6 gyro registers
    int16_t gyroX, gyroY, gyroZ; // Combined 3 axis data
    float gyro_x, gyro_y, gyro_z; // Float type of gyro data
    uint8_t gyro_val = 0x1A; // Start register address = 0X14;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &gyro_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, gyro, 6, false); // I2C read function

    gyroX = ((gyro[1]<<8) | gyro[0]);
    gyroY = ((gyro[3]<<8) | gyro[2]);
    gyroZ = ((gyro[5]<<8) | gyro[4]);

    gyro_x = ((gyroX / 16.0));
    gyro_y = ((gyroY / 16.0));
    gyro_z = ((gyroZ / 16.0));

    *px = gyro_x;
    *py = gyro_y;
    *pz = gyro_z;

    return 0;
}
