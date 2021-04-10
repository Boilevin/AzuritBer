/******************************************************************************
arduino_mpu9250_i2c.cpp - MPU-9250 Digital Motion Processor Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This library implements motion processing functions of Invensense's MPU-9250.
It is based on their Emedded MotionDriver 6.12 library.
	https://www.invensense.com/developers/software-downloads/

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
******************************************************************************/
#include "arduino_mpu9250_i2c.h"
#include <Arduino.h>
#include <Wire.h>

int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	Wire1.beginTransmission(slave_addr);
	Wire1.write(reg_addr);
	for (unsigned char i = 0; i < length; i++)
	{
		Wire1.write(data[i]);
	}
	Wire1.endTransmission(true);
	
	return 0;
}

int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	Wire1.beginTransmission(slave_addr);
	Wire1.write(reg_addr);
	Wire1.endTransmission(false);
	Wire1.requestFrom(slave_addr, length);
	for (unsigned char i = 0; i < length; i++)
	{
		data[i] = Wire1.read();
	}
	
	return 0;
}
