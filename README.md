# BNO055-Library-RP2040
This is a simple library that allows the communication from any RP2040 based board with the BNO055 sensor using I2C. This library
works best with the Arduino framework.

* Note that this library is under developement and will take time to be fully functional. *

* A known issue is with NDOF mode. Currently, all data from the BNO055 is obtained in NDOF sensor-fusion mode. This is bad because even raw data from the sensor has fused data that is used for filtering. For example, if you print the raw gyro data, you will see that it mysteriously does not drift and even if you move it a lot, it will always come back to zero. The data if very smooth. This is bad because it is using mysterious math and can mess up the gyro data. For example, if you were using this gyro data to determind orientation, after some movement, the gyros can deviate, sometimes to 50 or 100 degrees more than the ange is suppossed to be. NDOF mode also runs mysterious calibration of all three sensors in sensor-fusion mode which helps the gyros deviate from their correct orientation. In the future, AMG mode will be enabled instead of NDOF so that there is no sensor fusion enable. Data may not be as smooth as in NDOF mode but will be more correct and will not mess up when trying to determine orientation using only the gyroscopes. *

As of 12/5/22, this library functions to retrieve acceleration measurements and gyroscope measurements. Future updates will include:
- An easy to use user guide explaining how the library works and how to use it
- The addition to get more data from other sensors in the BNO055 IMU
- A calibration function to increase the accuracy

