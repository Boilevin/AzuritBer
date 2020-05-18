# AzuritBer
Version for Robomow RL1000

GY-87 MANDATORY

OPTIONAL RFID
	Use for multiple area mowing, Fast start Entry point,Fast return to station,soft docking ,etc....
	Need PN5180 AS RFID READER
	Need ESP32 dev kit to manage SPI communication with PN5180 
	Need Raspberry Pi 3B+ or Pi Zero for setting RFID

	The 3 big mow motor use 3 BTS7960 motor driver and 3 INA226 with R010 instead R100 to manage motor sense over I2C
