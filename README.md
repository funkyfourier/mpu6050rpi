# mpu6050rpi

Pure Data external for the MPU6050 motion sensor.

![Logos](https://i.imgur.com/M2A2aPZ.png)

Based on code from [alex-mous' MPU6050 library](https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi) and [rfetick's MPU6050_light for Arduino](https://github.com/rfetick/MPU6050_light).

This 

Due to my limited knowledge of C, the code could probably use some restructuring. Please do refactor! School me!

#### Installation

1. Install the GY-521 as shown [in this video](https://www.youtube.com/watch?v=JTFa5l7zAA4). Also enable i2c.
2. Install libraries: `sudo apt-get install libi2c-dev i2c-tools libi2c0`.
3. Go to the mpu6050rpi directory and type `make`.
4. In Pure Data, go to Preferences->Path and add the path of the mpu6050rpi directory.

#### How to use

Add the `[mpu6050rpi] external` to your Pure Data patch. Right-click and select *help*.
