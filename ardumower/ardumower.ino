/*      
 *      DUE and ODOMETRY MANDATORY VERSION 
 *      PCB1.3 
 *      IMU MPU-9250 MANDATORY 
 *      
 *      Connect DUE programming port to PC for programming and PC console
 *      Connect DUE Native USB port to Raspberry Pi 
 *      
 *      During all test and dev set Enable_DueWatchdog to false
 *      or increase the delay in robot.cpp line : watchdogEnable(2000);// Watchdog trigger after  2 sec if not reseted.
 *      
 *      --------------------------------- GPS ----------------------------------------------------
 *      GPS You need to change speed into pfod setting according to your module and connected to P44
 *     
 *      ------------------------------ RASPBERRY -------------------------------------------------
 *      If Raspberry PI is not connected you need to change into mower.h
 *          #define Console Serial
 *          //#define Console SerialUSB
 *      and into arduremote setting R/C set raspberryPiuse to NO
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
