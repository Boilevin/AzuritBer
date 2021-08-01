/*

  choose your robot type, PCB version, baud rates, pin definitions etc.

*/

#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include "robot.h"

#include "drivers.h"
#include "bt.h"


/*  This software requires:
        Ardumower PCB v0.5/1.2/1.3  ( https://www.marotronics.de/Ardumower-Board-Prototyp )
        Arduino Mega or Arduino Due (Due requires PCB1.3)
        Ardumower Chassis Kit 1.0  ( http://wiki.ardumower.de/index.php?title=Ardumower_chassis ) or Ardumower Mini
*/


// ------- Choose one Ardumower PCB revision (1.2, 1.3 etc.) ------------------
//#define PCB_1_2
#define PCB_1_3

// ------- Choose robot model (Ardumower or Ardumower Mini) -------------------
#define ROBOT_ARDUMOWER
//#define ROBOT_MINI

// ------- Choose motor driver (MC33926 is recommended) -----------------------
#define DRIVER_MC33926
//#define DRIVER_L298N

#define IOREF 3.3   // I/O reference voltage for Due





// ------ pins---------------------------------------
#define pinMotorEnable  37         // EN motors enable
#define pinMotorLeftPWM 5          // M1_IN1 left motor PWM pin
#define pinMotorLeftDir 31         // M1_IN2 left motor Dir pin
#define pinMotorLeftSense A1       // M1_FB  left motor current sense
#define pinMotorLeftFault 25       // M1_SF  left motor fault

#define pinMotorRightPWM  3        // M2_IN1 right motor PWM pin
#define pinMotorRightDir 33        // M2_IN2 right motor Dir pin
#define pinMotorRightSense A0      // M2_FB  right motor current sense
#define pinMotorRightFault 27      // M2_SF  right motor fault

#define pinMotorMowPWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define pinMotorMowDir 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
#define pinMotorMowSense A3        // M1_FB  mower motor current sense  
#define pinMotorMowFault 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowEnable 28       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowRpm A11

#define pinBumperLeft 39           // bumper pins
#define pinBumperRight 38

#define pinDropLeft 45           // drop pins                                                                                          Dropsensor - Absturzsensor
#define pinDropRight 23          // drop pins                                                                                          Dropsensor - Absturzsensor

#define pinSonarCenterTrigger 24   // ultrasonic sensor pins
#define pinSonarCenterEcho 22
#define pinSonarRightTrigger 30
#define pinSonarRightEcho 32
#define pinSonarLeftTrigger 34
#define pinSonarLeftEcho 36
#define pinPerimeterRight A4       // perimeter
#define pinPerimeterLeft A5
#define pinPerimeterCenter A6


#define pinGreenLED 6              // DuoLED green
#define pinRedLED 7                // DuoLED red
#define pinLED 13                  // LED
#define pinBuzzer 53               // Buzzer
#define pinTilt 35                 // Tilt sensor (required for TC-G158 board)
#define pinButton 51               // digital ON/OFF button
#define pinBatteryVoltage A2       // battery voltage sensor
#define pinBatterySwitch 4         // battery-OFF switch   
#define pinChargeVoltage A9        // charging voltage sensor
#define pinChargeCurrent A8        // charge current sensor
#define pinChargeRelay 50          // charge relay
#define pinRemoteMow 12            // remote control mower motor
#define pinRemoteSteer 11          // remote control steering 
#define pinRemoteSpeed 10          // remote control speed
#define pinRemoteSwitch 52         // remote control switch
#define pinVoltageMeasurement A7   // test pin for your own voltage measurements

#define pinOdometryLeft DAC0     // left odometry sensor
#define pinOdometryLeft2 DAC1    // left odometry sensor (optional two-wire)
#define pinOdometryRight CANRX   // right odometry sensor  
#define pinOdometryRight2 CANTX  // right odometry sensor (optional two-wire)  

#define pinLawnFrontRecv 40        // lawn sensor front receive
#define pinLawnFrontSend 41        // lawn sensor front sender 
#define pinLawnBackRecv 42         // lawn sensor back receive
#define pinLawnBackSend 43         // lawn sensor back sender 
#define pinUserSwitch1 46          // user-defined switch 1
#define pinUserSwitch2 47          // user-defined switch 2
#define pinUserSwitch3 48          // user-defined switch 3
#define pinRain 44                 // rain sensor

// ---------------- COMPASS Selection ---------------------------
//#define COMPASS_IS HMC5883L
#define COMPASS_IS QMC5883L



// ------ serial ports for console, Bluetooth, ESP8266 -----------------------------

// Due has two serial ports: Native (SerialUSB) and Programming (Serial) -
// redirect 'Console' to -->'SerialUSB' so the Raspberry PI receive all message console data
// redirect 'Console' to -->'Serial' so the Raspberry PI receive all message console data

//#define Console Serial
#define Console SerialUSB
#define CONSOLE_BAUDRATE    115200       // baudrate used for Raspberry PI console

#define Enable_DueWatchdog true
//#define Enable_DueWatchdog false

//#define Enable_Screen true
#define Enable_Screen false

#define RaspberryPIPort SerialUSB  //The PI is connected on NATIVE USB port over USB cable

#define ESP8266port Serial1  //esp01
#define ESP8266_BAUDRATE    115200      // baudrate used for communication with esp8266 Wifi module

#define Bluetooth Serial2  // Ardumower default OK for ESP32 or HC05
#define BLUETOOTH_BAUDRATE  19200      // baudrate used for communication with Bluetooth module (Ardumower default: 19200)
#define BLUETOOTH_PIN       1234

#define GpsPort Serial3  // GPS do not forget workarround if PCB1.3 use

// ------- RTC  and EEPROM I2C addresses --------------------------------------------------------------
#define DS1307_ADDRESS B1101000
#define AT24C32_ADDRESS B1010000 //0x50 //Standard PCB1.3 RTC ds1307 memory module
//#define AT24C32_ADDRESS B1010111 //0x57 //Simple PCB RTC ds3231 memory module

// ---- choose only one perimeter signal code ----
#define SIGCODE_1  // Ardumower default perimeter signal
//#define SIGCODE_2  // Ardumower alternative perimeter signal
//#define SIGCODE_3  // Ardumower alternative perimeter signal

// ------- ultrasonic config ---------------------------------------------------------
#define NO_ECHO 0

/*
  Ardumower robot chassis
*/

class Mower : public Robot
{
  public:
    Mower();
    virtual void setup(void);
    // virtual void resetMotorFault();
    //virtual int readSensor(char type);
    virtual void setActuator(char type, int value);
    virtual void configureBluetooth(boolean quick);
};


extern Mower robot;

#endif
