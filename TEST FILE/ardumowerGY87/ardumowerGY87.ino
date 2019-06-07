/*
 * 
 * 
 *      
 *      DUE and ODOMETRY MANDATORY VERSION 
 *      PCB1.3 
 *      FOR BY LANE USE, IMU GY-88 MANDATORY 
 *      
 *      Use DUE programming port for programming and console!
 *      
 *      If Raspberry PI is not use change into mower.h
 *          RaspberryPIUse = false or true
 *          #define Console Serial
 *          //#define Console SerialUSB
 *      to have access to the Console on the PC else  PI  received the Console Output 
 *      
 *      
 *      
 *      
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
