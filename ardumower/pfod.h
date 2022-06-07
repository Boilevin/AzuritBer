/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat

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

*/

// Android remote control (pfod App)
// For a detailed specification of the pfodApp protocol, please visit:  http://www.forward.com.au/pfod/

// example usage:
//   RemoteControl remote;
//   remote.initSerial(19200);
//   while (true){
//     remote.readSerial();
//     remote.run();
//  }


#ifndef PFOD_H
#define PFOD_H

#include "drivers.h"
#include "pid.h"

/* The Modifications are made by Fürst Ruprecht for azuritBer and TeensyMower -Software made by the great Softwareengineer Bernard from France 
 * See :  https://github.com/Boilevin
 * Visit us at : https://www.diy-robot-lawn-mower.com/
 * Select your setting by removing // at the appropriate place. 
 * Attention: the compiler only checks whether the keyword is defined/existing. 
 * It does not check the correctness. Therefore true / false has no meaning for the function but only for the syntax.
*/

//#define DUE true                //if you use DUE-CPU
//#define TEENSYDUINO true        //if you use Teensy4.1-CPU
#define STANDARD true           //if you are NOT Fürst Ruprecht
//#define FUERSTRUPRECHT true     //if you are Fürst Ruprecht
//#define USECONSOLE true         //if you want to send the arduino-console-Output to your mobile/tablet
#define USEPFOD true            //if you use the arduino-pfod-V3-app

#if not defined(TEENSYDUINO)  //(DUE) 
  #include <Arduino.h>    //(DUE) 
  
  #include "perimeter.h"  //(DUE)
  
#endif


// pfodApp state
enum { PFOD_OFF, PFOD_MENU, PFOD_CONSOLE,
       PFOD_PLOT_BAT, PFOD_PLOT_ODO2D, PFOD_PLOT_IMU, PFOD_PLOT_SENSOR_COUNTERS,
       PFOD_PLOT_SENSORS, PFOD_PLOT_PERIMETER, PFOD_PLOT_GPS, PFOD_PLOT_GPS2D,
       PFOD_PLOT_MOTOR
     };
class Robot;
class RemoteControl
{
  public:
    RemoteControl();
    void setRobot(Robot *aRobot);
    void initSerial(HardwareSerial* serialPort, uint32_t baudrate);
    boolean readSerial();
    //bb10
    void processPI(String RpiCmd, float v1, float v2, float v3) ;
    void run();
    byte pfodState;
    
  private:
    HardwareSerial* serialPort;
    Robot *robot;
    boolean pfodCmdComplete;
    String pfodCmd;
    String logFileNameArray [51][2];
    int rfidDetailIdx;
    int testmode;
    unsigned long nextPlotTime;
    //bb
    float value1;
    float value2;
    float value3;
    boolean dataFromPi;
	#if not defined(TEENSYDUINO)
		int8_t perimeterCapture[RAW_SIGNAL_SAMPLE_SIZE];
	#endif	
    int perimeterCaptureIdx;
    float stringToFloat(String &s);
    byte rfid_pos_into_list;
    
    // generic
    void sendYesNo(int value);
    void sendOnOff(int value);
    void sendLeftRight(int value);
    void printDouble( double val, byte precision);//FR: code from Thorsten-ac


    // PID slider
    void sendPIDSlider(String cmd, String title, PID &pid, double scale, float maxvalue);
    void processPIDSlider(String result, String cmd, PID &pid, double scale, float maxvalue);

    // generic slider
    void sendSlider(String cmd, String title, float value, String unit, double scale, float maxvalue, float minvalue = 0);
    void processSlider(String result, float &value, double scale);
    void processSlider(String result, long &value, double scale);
    void processSlider(String result, int &value, double scale);
    void processSlider(String result, byte &value, double scale);
    void processSlider(String result, short &value, double scale);	
		void processSlider(String result, unsigned long &value, double scale);
	

    // send timer menu details
    void sendTimer(ttimer_t timer);

    // main menu
    void sendMainMenu(boolean update);
    void sendErrorMenu(boolean update);
    void sendInfoMenu(boolean update);
    void sendCommandMenu(boolean update);
    void processCommandMenu(String pfodCmd);
    void sendManualMenu(boolean update);
    void sendCompassMenu(boolean update);
    void sendTestOdoMenu(boolean update);
    void sendMainTestMenu(boolean update);
    void processCompassMenu(String pfodCmd);
    void processTestOdoMenu(String pfodCmd);
    void processMainTestMenu(String pfodCmd);
    void processManualMenu(String pfodCmd);
    void processSettingsMenu(String pfodCmd);

    // plotting
    void sendPlotMenu(boolean update);

    // live data menu
    void sendLiveDataMenu(boolean update);
    void processLiveDataMenu(String pfodCmd);
    void processLiveDataIMUMenu(boolean update);
    void processLiveDataBatteryMenu(boolean update);
    void processLiveDataPerimeterMenu(boolean update);
    void processLiveDataSonarMenu(boolean update);
    void processLiveDataGPS2DMenu(boolean update);

    // settings
    void sendSettingsMenu(boolean update);
    void sendMotorMenu(boolean update);
    void sendMowMenu(boolean update);
    void sendBumperMenu(boolean update);
	#if not defined(TEENSYDUINO)
		void sendDropMenu(boolean update);
	#endif
    void sendSonarMenu(boolean update);
    void sendPerimeterMenu(boolean update);
    #if not defined(TEENSYDUINO)
		void sendLawnSensorMenu(boolean update);
	#endif	
    void sendImuMenu(boolean update);
    void sendRemoteMenu(boolean update);
    void sendBatteryMenu(boolean update);
    void sendStationMenu(boolean update);
    void sendOdometryMenu(boolean update);
    void sendRainMenu(boolean update);
    void sendGPSMenu(boolean update);
    void sendRFIDMenu(boolean update);
    void sendRfidDetailMenu(int rfidDetailIdx,boolean update);

    void sendDateTimeMenu(boolean update);
    void sendFactorySettingsMenu(boolean update);

    void sendByLaneMenu(boolean update);
    void processByLaneMenu(String pfodCmd);

    void processMotorMenu(String pfodCmd);
    void processErrorMenu(String pfodCmd);
    void processMowMenu(String pfodCmd);
    void processBumperMenu(String pfodCmd);
    void processSonarMenu(String pfodCmd);
    void processPerimeterMenu(String pfodCmd);
    #if not defined(TEENSYDUINO)
		void processLawnSensorMenu(String pfodCmd);
		void processDropMenu(String pfodCmd);
	#endif	
    void processRainMenu(String pfodCmd);    
    void processGPSMenu(String pfodCmd);
    void processRFIDMenu(String pfodCmd);
    void processRfidDetailMenu(int rfidDetailIdx,String pfodCmd);
    void processImuMenu(String pfodCmd);
    void processRemoteMenu(String pfodCmd);
    void processBatteryMenu(String pfodCmd);
    void processStationMenu(String pfodCmd);
    void processOdometryMenu(String pfodCmd);
    void processDateTimeMenu(String pfodCmd);
    void processFactorySettingsMenu(String pfodCmd);
    void processInfoMenu(String pfodCmd);
	#if defined(useStepperMenu)
		void sendStepperMenu(boolean update);
		void processStepperMenu(String pfodCmd);
	#endif	
	#if defined(USECONSOLE)		
		void sendConsoleMenu(boolean update);
  #endif

    // timer
    void sendTimerDetailMenu(int timerIdx, boolean update);
    void processTimerDetailMenu(String pfodCmd);
    void sendTimerMenu(boolean update);
    void processTimerMenu(String pfodCmd);

  #if defined(TEENSYDUINO)
    void sendSdCardLogMenu(boolean update);
    void processSdCardLogMenu(String pfodCmd);
    void sendSdCardLogDetailMenu(String logFileName);
    //void processSdCardLogDetailMenu(int rfidDetailIdx,String pfodCmd);
  #endif  
};



#endif
