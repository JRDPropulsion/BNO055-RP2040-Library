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


// Functions from the BNO055.h file.
BNO055_sensor::BNO055_sensor()
{

}

void BNO055_sensor::begin()
{   
    // Configure I2C Communication with BNO055
    _i2c_init(I2C_PORT, 100 * 1000);    // Add an extra "_" at the front if using Arduino core and hardware/i2c
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
    data[0] = 0x3F;
    data[1] = 0x01;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);

    // Configure Power Mode
    data[0] = 0x3E;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
    delay(50);

    // Defaul Axis Configuration
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

    // Set operation to acceleration only
    data[0] = 0x3D;
    data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, data, 2, true);
    delay(100);
}


double BNO055_sensor::accel_x()
{
    uint8_t accel[6];   // Store data from 6 acceleration registers
    uint8_t accel_val = 0x08; // Start register address = 0X08;
    int16_t accelX; // Combined 3 axis data
    double accelX1;
    double accel_x;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &accel_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, accel, 6, false); // I2C read function

    accelX = ((accel[1]<<8) | accel[0]);
    accelX1 = (accelX);
    accel_x = (accelX1 / 100);

    return accel_x;
}

double BNO055_sensor::accel_y()
{
    uint8_t accel[6];   // Store data from 6 acceleration registers
    uint8_t accel_val = 0x08; // Start register address = 0X08;
    int16_t accelY; // Y-axis data
    double accelY1;
    double accel_y;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &accel_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, accel, 6, false); // I2C read function

    accelY = ((accel[3]<<8) | accel[2]);
    accelY1 = (accelY);
    accel_y = (accelY1 / 100);

    return accel_y;
}

double BNO055_sensor::accel_z()
{
    uint8_t accel[6];   // Store data from 6 acceleration registers
    uint8_t accel_val = 0x08; // Start register address = 0X08;
    int16_t accelZ; // Z-axis data
    double accelZ1;
    double accel_z;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &accel_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, accel, 6, false); // I2C read function

    accelZ = ((accel[5]<<8) | accel[4]);
    accelZ1 = (accelZ);
    accel_z = (accelZ1 / 100);

    return accel_z;
}


double BNO055_sensor::gyro_x()
{
    uint8_t gyro[6];   // Store data from 6 acceleration registers
    uint8_t gyro_val = 0x14; // Start register address = 0X14;
    int16_t gyroX; // Z-axis data
    double gyroX1;
    double gyro_x;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &gyro_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, gyro, 6, false); // I2C read function

    gyroX = ((gyro[1]<<8) | gyro[0]);
    gyroX1 = (gyroX);
    gyro_x = ((3.14159 / 180) * (gyroX1 / 16));

    return gyro_x;
}

double BNO055_sensor::gyro_y()
{
    uint8_t gyro[6];   // Store data from 6 acceleration registers
    uint8_t gyro_val = 0x14; // Start register address = 0X14;
    int16_t gyroY; // Z-axis data
    double gyroY1;
    double gyro_y;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &gyro_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, gyro, 6, false); // I2C read function

    gyroY = ((gyro[3]<<8) | gyro[2]);
    gyroY1 = (gyroY);
    gyro_y = ((3.14159 / 180) * (gyroY1 / 16));

    return gyro_y;
}

double BNO055_sensor::gyro_z()
{
    uint8_t gyro[6];   // Store data from 6 acceleration registers
    uint8_t gyro_val = 0x14; // Start register address = 0X14;
    int16_t gyroZ; // Z-axis data
    double gyroZ1;
    double gyro_z;

    i2c_write_blocking(I2C_PORT, BNO055_I2C_ADDR, &gyro_val, 1, true);  // I2C write function
    i2c_read_blocking(I2C_PORT, BNO055_I2C_ADDR, gyro, 6, false); // I2C read function

    gyroZ = ((gyro[5]<<8) | gyro[4]);
    gyroZ1 = (gyroZ);
    gyro_z = ((3.14159 / 180) * (gyroZ1 / 16));

    return gyro_z;
}

