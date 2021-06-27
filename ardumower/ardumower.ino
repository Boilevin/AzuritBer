/*      
 *      DUE and ODOMETRY MANDATORY VERSION 
 *      PCB1.3 
 *      COMPATIBLE WITH THIS IMU 
 *      GY-88 CONNECTED ON I2C2 IMU
 *      GY-521 with compass deactivate CONNECTED ON I2C2 IMU
 *      GY-521 CONNECTED ON I2C2 IMU + GY-273 CONNECTED ON I2C3 DISPLAY with compass activate
 *      SO IT'S MPU6050 and HMC5883L or QMC5883L
 *      
 *      Connect DUE programming port to PC for programming and PC console
 *      Connect DUE Native USB port to Raspberry Pi 
 *      
 *      During all test and dev set Enable_DueWatchdog to false
 *      or increase the delay in robot.cpp line : watchdogEnable(3000);// Watchdog trigger after  3 sec if not reseted.
 *      
 *      -------------------------- COMPASS TYPE --------------------------------------------------
 *      Into mower.h line 109 select correct configuration
 *      // ---------------- COMPASS Selection ---------------------------
 *      //#define COMPASS_IS HMC5883L
 *      #define COMPASS_IS QMC5883L
 *      -------------------------- BT or ESP8266 --------------------------------------------------
 *      Into mower.cpp line 255 select correct configuration
 *       bluetoothUse      = 1;      // use Bluetooth module? It's Impossible to use Bluetooth and esp8266 at same time
 *       esp8266Use        = 0;       // use ESP8266 Wifi module?
 *      
 *      ------------------------- RTC CHIP -------------------------------------------------------
 *      Into mower.h line 163 Select the correct RTC.
 *      #define AT24C32_ADDRESS B1010000 //0x50 //Standard PCB1.3 RTC ds1307 memory module
 *      //#define AT24C32_ADDRESS B1010111 //0x57 //Simple PCB RTC ds3231 memory module
 *      
 *      
 *      ------------------------- OLED SCREEN -------------------------------------------------------
 *      Into mower.h line 128 Enable_Screen if you want to connect a 128*64 OLED screen to mower      
 *      #define Enable_Screen true
 *      //#define Enable_Screen false
 *          
 *     
 *      ------------------------------ RASPBERRY -------------------------------------------------
 *      If Raspberry PI is not connected you need to change into mower.h
 *      into mower.h line 121
 *          #define Console Serial
 *          //#define Console SerialUSB
 *      and into arduremote setting Raspberry set raspberryPiuse to NO
 *      then you have access to the Console on the PC 
 *      
 *      If Raspberry PI is connected you need to change into mower.h
 *          //#define Console Serial
 *          #define Console SerialUSB
 *      and into arduremote setting R/C set raspberryPiuse to YES
 *      ------------------------------------------------------------------------------------------ 
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri    
  Copyright (c) 2014-2015 by Stefan Manteuffel
  Copyright (c) 2015 by Uwe Zimprich
  
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)


  Documentation:  http://wiki.ardumower.de
    
 */
#include <Arduino.h>
#include <Wire.h>
#include "config.h"



void setup()  {     
  robot.setup();
} 

void loop()  {     
  robot.loop();    
}
