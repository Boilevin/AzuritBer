# AzuritBer
Ardumower full odometry version
see https://www.ardumower.de for more info.

Master branch is GY-88 version
See other branch for GY-87 or MPU-9250/9255 version


OPTIONAL RFID/RASPBERRY:
	Use for multiple area mowing, Fast start Entry point,Fast return to station,soft docking ,etc....
	Need PN5180 AS RFID READER
	Need ESP32 dev kit to manage SPI communication with PN5180 
	Need Raspberry Pi 3B+ or Pi Zero for setting RFID


GY87 version use a GY87 IMU connected on I2C1 to avoid i2c adress conflit with RTC .

RL1000 version is the code for robomow rl model platform.
	The 3 big mow motor use 3 BTS7960 motor driver and 3 INA226 with R010 instead R100 to manage motor sense over I2C

