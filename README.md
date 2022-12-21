# BNO055-Library-RP2040
This is a simple library that allows the communication from any RP2040 based board with the BNO055 sensor using I2C. This library
works best with the Arduino framework. *Note that this library is under developement and will take time to be fully functional.


HOW TO USE:
This library features an example file that you can use to obtain the raw gyro rates from the BNO055. In order to use this library, 
you must to the following things:
- Make sure you are using the RP2040. This library uses the hardware/i2c library that is meant for the RP2040. 
  This is most likely already installed by default if you are using the RP2040. Secondly, this works best with the Adafruit BNO055
  breakout board but others may work as well. The best RP2040 board to use is the Raspberry Pi pico as it has been tested. 
  You may need to do some configuration work with a custom RP2040 baised board.
- Go to the BNO055.cpp file and change the SDA and SCL pin numbers to the pin numbers you want to use. 
  By default, SDA is set to pin 26 and SCL is set to pin 27, both of which are the GPIO pin numbers onboard the Raspberry Pi pico.
 
 
Once you have completed the steps above, you are ready to move on. You can either use the Arduino IDE or Platform IO to upload to the board. 
There are alternatives that may work as well but these are the most popular. Then, it is as simple as uploading the sketch to the board!
You will recieve raw gyro rates but you can also retrieve raw acceleration data. The process is the exact same to that of the gyroscopes.
This library does not retrieve sensor fusion data. If you wish to retrieve sensor fusion, then go to the BNO055.cpp file and uncomment 
this line of code:  //data[1] = 0x0C; // Set operation to NDOF. Next, comment out thise: data[1] = 0x03;. If you make these changes, 
you can modify the library to retrieve sensor fusion data as well as acceleration data since the default mode only retrieves gyro data.


  

