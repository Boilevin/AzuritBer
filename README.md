# AzuritBer
Ardumower full odometry version
see https://www.ardumower.de  and https://wiki.ardumower.de/index.php?title=AzuritBer_Firmware_(English)# for more info.

Master branch need PCB1.3 or 1.4 ,ODOMETRY on drive motor and IMU : GY-521 / or GY-88  based on MPU6050 version



OPTIONAL:

        	RFID BOARD PN5180 AS RFID READER and ESP32 dev kit to manage SPI communication with PN5180 and WIFI/BT for arduremote
	
		RASPBERRY for vision or camera streaming:
	
See other branch for GY-87  or MPU-9250/9255 version

GY87 version use a GY87 IMU connected on I2C1 to avoid i2c adress conflit with RTC .

RL1000 version is the code for robomow rl model platform.
	The 3 big mow motor use 3 BTS7960 motor driver and 3 INA226 with R010 instead R100 to manage motor sense over I2C

