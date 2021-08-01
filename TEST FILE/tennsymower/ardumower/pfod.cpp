/*  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri

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


#include "pfod.h"
#include "robot.h"
//#include "adcman.h"
#include "imu.h"
#include "perimeter.h"
#include "config.h"

RemoteControl::RemoteControl() {
  pfodCmdComplete = false;
  pfodCmd = "";
  pfodState = PFOD_OFF;
  testmode = 0;
  nextPlotTime = 0;
  perimeterCaptureIdx = 0;
}

void RemoteControl::setRobot(Robot *aRobot) {
  this->robot = aRobot;
}

void RemoteControl::initSerial(HardwareSerial* _serialPort, uint32_t baudrate) {
  this->serialPort = _serialPort;
  serialPort->begin(baudrate);
}

float RemoteControl::stringToFloat(String &s) {
  float v;
  char tmp[20];
  s.toCharArray(tmp, sizeof(tmp));
  v = atof(tmp);
  //v = strtod(tmp, NULL);
  /*Console.print(s);
    Console.print("=");
    Console.println(v, 6);   */
  return v;
}

void RemoteControl::sendYesNo(int value) {
  if (value == 1) serialPort->print("YES");
  else serialPort->print("NO");
}
void RemoteControl::sendLeftRight(int value) {
  if (value == 1) serialPort->print("LEFT");
  else serialPort->print("RIGHT");
}


void RemoteControl::sendOnOff(int value) {
  if (value == 1) serialPort->print("ON");
  else serialPort->print("OFF");
}

void RemoteControl::sendTimer(ttimer_t timer) {
  if (timer.active) serialPort->print(F("(X)  "));
  else serialPort->print(F("(   )  "));
  serialPort->print(time2str(timer.startTime));
  serialPort->print("-");
  serialPort->print(time2str(timer.stopTime));
  serialPort->println();
  if (timer.daysOfWeek == B1111111) {
    serialPort->print(F("every day"));
  } else {
    int counter = 0;
    for (int j = 0; j < 7; j++) {
      if ((timer.daysOfWeek >> j) & 1) {
        if (counter != 0) serialPort->print(",");
        serialPort->print(dayOfWeek[j]);
        counter++;
      }
    }
  }
}


// NOTE: pfodApp rev57 changed slider protocol:  displayValue = (sliderValue + offset) * scale
void RemoteControl::sendSlider(String cmd, String title, float value, String unit, double scale, float maxvalue, float minvalue) {
  serialPort->print("|");
  serialPort->print(cmd);
  serialPort->print("~");
  serialPort->print(title);
  serialPort->print(" `");
  serialPort->print(((int)(value / scale)));
  serialPort->print("`");
  serialPort->print(((int)(maxvalue / scale)));
  serialPort->print("`");
  serialPort->print(((int)(minvalue / scale)));
  serialPort->print("~ ~");
  if (scale == 10) serialPort->print("10");
  else if (scale == 1) serialPort->print("1");
  else if (scale == 0.1) serialPort->print("0.1");
  else if (scale == 0.01) serialPort->print("0.01");
  else if (scale == 0.001) serialPort->print("0.001");
  else if (scale == 0.0001) serialPort->print("0.0001");
}

void RemoteControl::sendPIDSlider(String cmd, String title, PID &pid, double scale, float maxvalue) {
  sendSlider(cmd + "p", title + "_P", pid.Kp, "", scale, maxvalue);
  sendSlider(cmd + "i", title + "_I", pid.Ki, "", scale, maxvalue);
  sendSlider(cmd + "d", title + "_D", pid.Kd, "", scale, maxvalue);
}



void RemoteControl::processPIDSlider(String result, String cmd, PID &pid, double scale, float maxvalue) {
  if (dataFromPi) {
    pid.Kp = value1;
    pid.Ki = value2;
    pid.Kd = value3;
  }
  else {
    int idx = result.indexOf('`');
    String s = result.substring(idx + 1);
    //Console.println(tmp);
    float v = stringToFloat(s);
    if (pfodCmd.startsWith(cmd + "p")) {
      pid.Kp = v * scale;
      if (pid.Kp < scale) pid.Kp = 0.0;
    }
    else if (pfodCmd.startsWith(cmd + "i")) {
      pid.Ki = v * scale;
      if (pid.Ki < scale) pid.Ki = 0.0;
    }
    else if (pfodCmd.startsWith(cmd + "d")) {
      pid.Kd = v * scale;
      if (pid.Kd < scale) pid.Kd = 0.0;
    }
  }
}

void RemoteControl::processSlider(String result, float &value, double scale) {
  if (dataFromPi) {
    value = value1;
  }
  else {
    int idx = result.indexOf('`');
    String s = result.substring(idx + 1);
    float v = stringToFloat(s);
    value = v * scale;
  }
}
void RemoteControl::processSlider(String result, long &value, double scale) {
  if (dataFromPi) {
    value = value1;
  }
  else {
    float v;
    processSlider(result, v, scale);
    value = v;
  }
}

void RemoteControl::processSlider(String result, int &value, double scale) {
  if (dataFromPi) {
    value = value1;
  }
  else {
    float v;
    processSlider(result, v, scale);
    value = v;
  }
}

void RemoteControl::processSlider(String result, byte &value, double scale) {
  if (dataFromPi) {
    value = value1;
  }
  else {
    float v;
    processSlider(result, v, scale);
    value = v;
  }
}

void RemoteControl::processSlider(String result, short &value, double scale) {
  if (dataFromPi) {
    value = value1;
  }
  else {
    float v;
    processSlider(result, v, scale);
    value = v;
  }
}





void RemoteControl::sendMainMenu(boolean update) {
  if (update) serialPort->print("{:"); else {
    serialPort->print(F("{.Ardumower"));
    serialPort->print(" (");
    serialPort->print(robot->name);
    serialPort->print(")");
  }
  serialPort->print(F("|r~Commands|n~Manual|s~Settings|in~Info|c~Test IMU|yt~Test ODO|m1~Console|yp~Plot"));
  //bb1
  serialPort->println(F("|y4~Error counters}"));

}

void RemoteControl::sendPlotMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Plot"));
  serialPort->print(F("|y7~Sensors|y5~Sensor counters|y3~IMU|y6~Perimeter|y8~GPS"));
  serialPort->println(F("|y1~Battery|y2~Odometry2D|y11~Motor control|y10~GPS2D}"));
}


void RemoteControl::sendSettingsMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Settings"));
  if ((robot->stateCurr == STATE_OFF) || (robot->stateCurr == STATE_STATION))  //deactivate the save setting if the mower is not OFF to avoid zombie
  {
    serialPort->print(F("|sz~Save settings|s1~Motor|s2~Mow|s3~Bumper/Button|s4~Sonar|s5~Perimeter|s6~Lawn sensor|s7~IMU|s8~Raspberry"));
    serialPort->println(F("|s9~Battery|s10~Station|s11~Odometry|s13~Rain Temp Humid|s15~Drop sensor|s14~GPS RFID|i~Timer|s12~Date/time|sx~Factory settings|s16~ByLane Setting}"));
  }
  else
  {
    serialPort->print(F("|s1~Motor|s2~Mow|s3~Bumper/Button|s4~Sonar|s5~Perimeter|s6~Lawn sensor|s7~IMU|s8~Raspberry"));
    serialPort->println(F("|s9~Battery|s10~Station|s11~Odometry|s13~Rain Temp Humid|s15~Drop sensor|s14~GPS RFID|i~Timer|s12~Date/time|sx~Factory settings|s16~ByLane Setting}"));
  }
}

void RemoteControl::processSettingsMenu(String pfodCmd) {
  if (pfodCmd == "s1") sendMotorMenu(false);
  else if (pfodCmd == "s2") sendMowMenu(false);
  else if (pfodCmd == "s3") sendBumperMenu(false);
  else if (pfodCmd == "s4") sendSonarMenu(false);
  else if (pfodCmd == "s5") sendPerimeterMenu(false);
  else if (pfodCmd == "s6") sendLawnSensorMenu(false);
  else if (pfodCmd == "s7") sendImuMenu(false);
  else if (pfodCmd == "s8") sendRemoteMenu(false);
  else if (pfodCmd == "s9") sendBatteryMenu(false);
  else if (pfodCmd == "s10") sendStationMenu(false);
  else if (pfodCmd == "s11") sendOdometryMenu(false);
  else if (pfodCmd == "s12") sendDateTimeMenu(false);
  else if (pfodCmd == "s13") sendRainMenu(false);
  else if (pfodCmd == "s15") sendDropMenu(false);
  else if (pfodCmd == "s14") sendGPSMenu(false);
  else if (pfodCmd == "s16") sendByLaneMenu(false);
  else if (pfodCmd == "sx") sendFactorySettingsMenu(false);
  else if (pfodCmd == "sz") {
    robot->saveUserSettings();
    sendSettingsMenu(true);
  }
  else sendSettingsMenu(true);
}

void RemoteControl::sendErrorMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Error counters`1000"));
  serialPort->print(F("|z00~Reset counters"));
  //serialPort->print(F("|zz~ADC calibration "));
  //serialPort->print(robot->errorCounterMax[ERR_ADC_CALIB]);
  serialPort->print(F("|zz~Charger "));
  serialPort->print(robot->errorCounterMax[ERR_CHARGER]);
  serialPort->print(F("|zz~Battery "));
  serialPort->print(robot->errorCounterMax[ERR_BATTERY]);
  serialPort->print(F("|zz~Motor left "));
  serialPort->print(robot->errorCounterMax[ERR_MOTOR_LEFT]);
  serialPort->print(F("|zz~Motor right "));
  serialPort->print(robot->errorCounterMax[ERR_MOTOR_RIGHT]);
  serialPort->print(F("|zz~Motor mow "));
  serialPort->print(robot->errorCounterMax[ERR_MOTOR_MOW]);
  serialPort->print(F("|zz~Mow sense "));
  serialPort->print(robot->errorCounterMax[ERR_MOW_SENSE]);
  serialPort->print(F("|zz~Odometry left "));
  serialPort->print(robot->errorCounterMax[ERR_ODOMETRY_LEFT]);
  serialPort->print(F("|zz~Odometry right "));
  serialPort->print(robot->errorCounterMax[ERR_ODOMETRY_RIGHT]);
  serialPort->print(F("|zz~Perimeter timeout "));
  serialPort->print(robot->errorCounterMax[ERR_PERIMETER_TIMEOUT]);
  serialPort->print(F("|zz~Perimeter tracking "));
  serialPort->print(robot->errorCounterMax[ERR_TRACKING]);
  serialPort->print(F("|zz~IMU comm "));
  serialPort->print(robot->errorCounterMax[ERR_IMU_COMM]);
  serialPort->print(F("|zz~IMU calibration "));
  serialPort->print(robot->errorCounterMax[ERR_IMU_CALIB]);
  serialPort->print(F("|zz~IMU tilt "));
  serialPort->print(robot->errorCounterMax[ERR_IMU_TILT]);
  serialPort->print(F("|zz~RTC comm "));
  serialPort->print(robot->errorCounterMax[ERR_RTC_COMM]);
  serialPort->print(F("|zz~RTC data "));
  serialPort->print(robot->errorCounterMax[ERR_RTC_DATA]);
  serialPort->print(F("|zz~GPS comm "));
  serialPort->print(robot->errorCounterMax[ERR_GPS_COMM]);
  serialPort->print(F("|zz~GPS data "));
  serialPort->print(robot->errorCounterMax[ERR_GPS_DATA]);
  serialPort->print(F("|zz~Robot stuck "));
  serialPort->print(robot->errorCounterMax[ERR_STUCK]);
  serialPort->print(F("|zz~EEPROM data "));
  serialPort->print(robot->errorCounterMax[ERR_EEPROM_DATA]);
  serialPort->println("}");
}

void RemoteControl::processErrorMenu(String pfodCmd) {
  if (pfodCmd == "z00") {
    robot->resetErrorCounters();
    robot->setNextState(STATE_OFF, 0);
  }
  sendErrorMenu(true);
}


void RemoteControl::sendMotorMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Motor`1000"));

  serialPort->println(F("|a01~Power in Watt l, r "));
  serialPort->print(robot->motorLeftPower);
  serialPort->print(", ");
  serialPort->print(robot->motorRightPower);
  serialPort->println(F("|a05~motor current in mA l, r "));
  serialPort->print(robot->motorLeftSenseCurrent);
  serialPort->print(", ");
  serialPort->print(robot->motorRightSenseCurrent);
  sendSlider("a02", F("Power max"), robot->motorPowerMax, "", 0.1, 100, 0);
  serialPort->println(F("|a03~RPM/PWM Speed L/R "));
  serialPort->print(robot->motorLeftRpmCurr);
  serialPort->print(", ");
  serialPort->print(robot->motorRightRpmCurr);
  serialPort->print(F(" / "));
  serialPort->print(robot->motorLeftPWMCurr);
  serialPort->print(", ");
  serialPort->print(robot->motorRightPWMCurr);
  serialPort->print(F("|a10~Slope Adjust Speed : "));
  sendYesNo(robot->autoAdjustSlopeSpeed);



  //bber400
  serialPort->print(F("|a04~Actual Slope coeff : "));
  serialPort->print(robot->motorRpmCoeff);

  sendSlider("a06", F("Speed max in rpm"), robot->motorSpeedMaxRpm, "", 1, 50, 20);
  sendSlider("a15", F("Speed max in pwm"), robot->motorSpeedMaxPwm, "", 1, 255, 150);
  sendSlider("a11", F("Accel"), robot->motorAccel, "", 1, 2000, 500);
  sendSlider("a18", F("Power ignore time"), robot->motorPowerIgnoreTime, "", 1, 8000, 1);
  sendSlider("a07", F("Roll Degrees max"), robot->motorRollDegMax, "", 1, 360, 1);
  sendSlider("a19", F("Roll Degrees min"), robot->motorRollDegMin, "", 1, 180, 1);
  sendSlider("a08", F("Rev Distance / Perimeter"), robot->DistPeriOutRev, "", 1, 100, 1);
  sendSlider("a09", F("Stop Distance / Perimeter"), robot->DistPeriOutStop, "", 1, 30, 1);
  sendPIDSlider("a14", "RPM", robot->motorLeftPID, 0.01, 3.0);

  //bb add
  if (robot->developerActive) {
    sendSlider("a20", F("MotorSenseLeftScale"), robot->motorSenseLeftScale, "", 0.01, 0.10, 3.00);
    sendSlider("a21", F("MotorSenseRightScale"), robot->motorSenseRightScale, "", 0.01, 0.10, 3.00);
  }
  //end add
  serialPort->print(F("|a14~for config file:"));
  serialPort->print(F("motorSenseScale l, r"));
  serialPort->print(robot->motorSenseLeftScale);
  serialPort->print(", ");
  serialPort->print(robot->motorSenseRightScale);
  //bb
  sendSlider("a22", F("PWM Right Forward offset in %"), robot->motorRightOffsetFwd, "", 1, 50, -50);
  sendSlider("a23", F("PWM Right Reverse offset in %"), robot->motorRightOffsetRev, "", 1, 50, -50);
  sendSlider("a30", F("Speed Odo Minimum"), robot->SpeedOdoMin, "", 1, 90, 0);
  sendSlider("a31", F("Speed Odo Maximum"), robot->SpeedOdoMax, "", 1, 254, 100);
  serialPort->print(F("|a32~Calib Motor"));  //to compute the ticks per second motor speed and the RPM speed according to PWM
  serialPort->println("}");
}

void RemoteControl::processMotorMenu(String pfodCmd) {
  //bb add
  if (robot->developerActive) {
    if (pfodCmd.startsWith("a20")) processSlider(pfodCmd, robot->motorSenseLeftScale, 0.01);
    if (pfodCmd.startsWith("a21")) processSlider(pfodCmd, robot->motorSenseRightScale, 0.01);
  }
  //end add

  if (pfodCmd.startsWith("a02")) {
    processSlider(pfodCmd, robot->motorPowerMax, 0.1);
    //Console.print("motorpowermax=");
    //Console.println(robot->motorPowerMax);
  }

  // else if (pfodCmd.startsWith("a03")) {
  //   processSlider(pfodCmd, robot->motorLeftSenseCurrent, 1);
  //bb change ???????????????????????????? warning possible DIV by 0 so 1.0 instead of 0
  //robot->motorSenseLeftScale = robot->motorLeftSenseCurrent / max(0, (float)robot->motorLeftSenseADC);
  //  robot->motorSenseLeftScale = robot->motorLeftSenseCurrent / max(0.01, (float)robot->motorLeftSenseADC);
  //}
  //else if (pfodCmd.startsWith("a04")) {
  // processSlider(pfodCmd, robot->motorRightSenseCurrent, 1);
  //bb change ???????????????????? warning possible DIV by 0
  //robot->motorSenseRightScale = robot->motorRightSenseCurrent / max(0, (float)robot->motorRightSenseADC);
  // robot->motorSenseRightScale = robot->motorRightSenseCurrent / max(0.01, (float)robot->motorRightSenseADC);
  //}
  else if (pfodCmd.startsWith("a06")) processSlider(pfodCmd, robot->motorSpeedMaxRpm, 1);
  else if (pfodCmd.startsWith("a15")) processSlider(pfodCmd, robot->motorSpeedMaxPwm, 1);
  else if (pfodCmd.startsWith("a07")) processSlider(pfodCmd, robot->motorRollDegMax, 1);
  else if (pfodCmd.startsWith("a19")) processSlider(pfodCmd, robot->motorRollDegMin, 1);
  else if (pfodCmd.startsWith("a08")) processSlider(pfodCmd, robot->DistPeriOutRev, 1);
  else if (pfodCmd.startsWith("a09")) processSlider(pfodCmd, robot->DistPeriOutStop, 1);
  else if (pfodCmd.startsWith("a10")) robot->autoAdjustSlopeSpeed = !robot->autoAdjustSlopeSpeed;
  else if (pfodCmd.startsWith("a11")) processSlider(pfodCmd, robot->motorAccel, 1);
  else if (pfodCmd.startsWith("a12")) processSlider(pfodCmd, robot->motorBiDirSpeedRatio1, 0.01);
  else if (pfodCmd.startsWith("a13")) processSlider(pfodCmd, robot->motorBiDirSpeedRatio2, 0.01);
  else if (pfodCmd.startsWith("a14")) processPIDSlider(pfodCmd, "a14", robot->motorLeftPID, 0.01, 3.0);
  else if (pfodCmd.startsWith("a18")) processSlider(pfodCmd, robot->motorPowerIgnoreTime, 1);
  else if (pfodCmd.startsWith("a22")) processSlider(pfodCmd, robot->motorRightOffsetFwd, 1);
  else if (pfodCmd.startsWith("a23")) processSlider(pfodCmd, robot->motorRightOffsetRev, 1);


  else if (pfodCmd.startsWith("a30")) processSlider(pfodCmd, robot->SpeedOdoMin, 1);
  else if (pfodCmd.startsWith("a31")) processSlider(pfodCmd, robot->SpeedOdoMax, 1);

  else if (pfodCmd == "a32") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->stateEndOdometryRight = robot->odometryRight + 6 * robot->odometryTicksPerRevolution; //test on 6 full rev
    robot->stateEndOdometryLeft = robot->odometryLeft + 6 * robot->odometryTicksPerRevolution;
    robot->motorLeftSpeedRpmSet = 100;//robot->motorSpeedMaxRpm;
    robot->motorRightSpeedRpmSet = 100;//robot->motorSpeedMaxRpm;
    robot->setNextState(STATE_CALIB_MOTOR_SPEED, robot->rollDir);
    sendTestOdoMenu(true);
  }


  sendMotorMenu(true);
}

void RemoteControl::sendMowMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Mow`1000"));
  serialPort->print(F("|o12~Force mowing off: "));
  sendYesNo(robot->motorMowForceOff);
  //serialPort->print(F("|o00~Overload Counter "));
  //serialPort->print(robot->motorMowSenseCounter);
  serialPort->print(F("|o01~Power in Watt "));
  serialPort->print(robot->motorMowPower);
  serialPort->print(F("|o11~current in mA "));
  serialPort->print(robot->motorMowSenseCurrent);
  sendSlider("o02", F("Power max"), robot->motorMowPowerMax, "", 0.1, 100, 1);
  sendSlider("o05", F("PWM Max Speed "), robot->motorMowSpeedMaxPwm, "", 1, 255, 50);
  sendSlider("o08", F("PWM Min Speed "), robot->motorMowSpeedMinPwm, "", 1, 255, 50);
  if (robot->developerActive) {

    sendSlider("o03", F("calibrate mow motor "), robot->motorMowSenseScale, "", 0.01, 3, 0);
  }
  serialPort->print(F("|o07~PWM Coeff "));
  serialPort->print(robot->motorMowPwmCoeff);
  serialPort->print(F("|o04~Actual PWM "));
  serialPort->print(robot->motorMowPWMCurr);

  sendSlider("o13", F("Mow Pattern Max Duration Minutes"), robot->mowPatternDurationMax, "", 1, 255, 10);
  //sendPIDSlider("o09", "RPM", robot->motorMowPID, 0.01, 1.0);

  serialPort->println(F("|o10~Testing is"));
  switch (testmode) {
    case 0: serialPort->print(F("OFF")); break;
    case 1: serialPort->print(F("Motor ON")); break;
  }
  /*serialPort->println(F("|o04~for config file:"));
    serialPort->println(F("motorMowSenseScale:"));
    serialPort->print(robot->motorMowSenseScale);
  */
  serialPort->println("}");
}

void RemoteControl::processMowMenu(String pfodCmd) {
  if (pfodCmd.startsWith("o02")) processSlider(pfodCmd, robot->motorMowPowerMax, 0.1);
  else if (pfodCmd.startsWith("o12")) robot->motorMowForceOff = !robot->motorMowForceOff;
  else if (pfodCmd.startsWith("o03")) processSlider(pfodCmd, robot->motorMowSenseScale, 0.01);
  else if (pfodCmd.startsWith("o05")) processSlider(pfodCmd, robot->motorMowSpeedMaxPwm, 1);
  else if (pfodCmd.startsWith("o08")) processSlider(pfodCmd, robot->motorMowSpeedMinPwm, 1);
  else if (pfodCmd.startsWith("o13")) processSlider(pfodCmd, robot->mowPatternDurationMax, 1);
  //else if (pfodCmd.startsWith("o09")) processPIDSlider(pfodCmd, "o09", robot->motorMowPID, 0.01, 1.0);
  else if (pfodCmd == "o10") {
    testmode = (testmode + 1) % 2;
    switch (testmode) {
      case 0: robot->setNextState(STATE_OFF, 0); robot->motorMowPwmCoeff = 100; robot->motorMowEnable = false; break;
      case 1: robot->setNextState(STATE_MANUAL, 0); robot->motorMowEnable = true; break;
    }
  }
  sendMowMenu(true);
}

void RemoteControl::sendBumperMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Bumper`1000"));
  serialPort->print(F("|b00~Bumper Use "));
  sendYesNo(robot->bumperUse);
  serialPort->println(F("|b01~Counter l, r "));
  serialPort->print(robot->bumperLeftCounter);
  serialPort->print(", ");
  serialPort->print(robot->bumperRightCounter);
  serialPort->println(F("|b02~Value l, r "));
  serialPort->print(robot->bumperLeft);
  serialPort->print(", ");
  serialPort->print(robot->bumperRight);
  serialPort->println(F("|b03~Button Use "));
  sendYesNo(robot->buttonUse);
  serialPort->println("}");
}

void RemoteControl::processBumperMenu(String pfodCmd) {
  if (pfodCmd == "b00") robot->bumperUse = !robot->bumperUse;
  if (pfodCmd == "b03") robot->buttonUse = !robot->buttonUse;
  sendBumperMenu(true);
}

void RemoteControl::sendDropMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Drop`1000"));
  serialPort->print(F("|u00~Use "));
  sendYesNo(robot->dropUse);
  serialPort->println(F("|u01~Counter l, r "));
  serialPort->print(robot->dropLeftCounter);
  serialPort->print(", ");
  serialPort->print(robot->dropRightCounter);
  serialPort->println(F("|u02~Value l, r "));
  serialPort->print(robot->dropLeft);
  serialPort->print(", ");
  serialPort->print(robot->dropRight);
  serialPort->println("}");
}

void RemoteControl::processDropMenu(String pfodCmd) {
  if (pfodCmd == "u00") robot->dropUse = !robot->dropUse;
  sendDropMenu(true);
}


void RemoteControl::sendSonarMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Sonar`1000"));
  serialPort->print(F("|d00~Use "));
  sendYesNo(robot->sonarUse);
  serialPort->print(F("|d04~Use left "));
  sendYesNo(robot->sonarLeftUse);
  serialPort->print(F("|d05~Use center "));
  sendYesNo(robot->sonarCenterUse);
  serialPort->print(F("|d06~Use right "));
  sendYesNo(robot->sonarRightUse);
  // serialPort->print(F("|d01~Counter "));
  // serialPort->print(robot->sonarDistCounter);
  serialPort->println(F("|d02~Value l, c, r"));
  serialPort->print(robot->sonarDistLeft);
  serialPort->print(", ");
  serialPort->print(robot->sonarDistCenter);
  serialPort->print(", ");
  serialPort->print(robot->sonarDistRight);
  sendSlider("d03", F("Sonar Brake below"), robot->sonarTriggerBelow, "", 1, 150, 20);
  sendSlider("d09", F("Sonar To Front Dist"), robot->sonarToFrontDist, "", 1, 100, 0);

  serialPort->println("}");
}

void RemoteControl::processSonarMenu(String pfodCmd) {
  if (pfodCmd == "d00") robot->sonarUse = !robot->sonarUse;
  else if (pfodCmd.startsWith ("d03")) processSlider(pfodCmd, robot->sonarTriggerBelow, 1);
  else if (pfodCmd == "d04") robot->sonarLeftUse = !robot->sonarLeftUse;
  else if (pfodCmd == "d05") robot->sonarCenterUse = !robot->sonarCenterUse;
  else if (pfodCmd == "d06") robot->sonarRightUse = !robot->sonarRightUse;
  else if (pfodCmd.startsWith ("d09")) processSlider(pfodCmd, robot->sonarToFrontDist, 1);




  sendSonarMenu(true);
}

void RemoteControl::sendPerimeterMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Perimeter`1000"));
  serialPort->print(F("|e00~Use "));
  sendYesNo(robot->perimeterUse);
  serialPort->print(F("|e09~Actual Mowing Area"));
  serialPort->print(robot->areaInMowing);
  serialPort->println(F("|e02~Mag / Smag"));
  serialPort->print(robot->perimeterMag);
  serialPort->print(F("/"));
  serialPort->print(robot->perimeter.getSmoothMagnitude(0));
  //serialPort->print(robot->perimeterMagRight);
  //if (robot->perimeterMag < 0) serialPort->print(" (inside)");
  //else serialPort->print(" (outside)");
  sendSlider("e08", F("Mini Smag"), robot->perimeter.timedOutIfBelowSmag, "", 1, 200, 1);
  sendSlider("e14", F("Timeout (s) if Outside"), robot->perimeter.timeOutSecIfNotInside, "", 1, 20, 1);
  sendSlider("e04", F("Big AREA Smag Center"), robot->perimeterTriggerMinSmag, "", 1, 600, 100);
  sendSlider("e18", F("Tracking Max Speed PWM"), robot->MaxSpeedperiPwm, "", 1, 255, 80);
  sendSlider("e20", F("Circle Arc disance (cm) Obstacle while tracking"), robot->DistPeriObstacleAvoid, "", 1 , 250, 1);
  sendSlider("e21", F("Perimeter MAG MAX VALUE"), robot->perimeterMagMaxValue, "", 1 , 2500, 500);
  sendSlider("e11", F("Transition timeout"), robot->trackingPerimeterTransitionTimeOut, "", 1, 5000, 1);
  sendSlider("e12", F("Track error timeout"), robot->trackingErrorTimeOut, "", 1, 10000, 1);
  sendPIDSlider("e07", F("Track"), robot->perimeterPID, 0.1, 52);
  serialPort->print(F("|e10~Swap Left coil polarity "));
  sendYesNo(robot->perimeter.swapCoilPolarityLeft);
  serialPort->print(F("|e22~Swap Right coil polarity "));
  sendYesNo(robot->perimeter.swapCoilPolarityRight);
  serialPort->print(F("|e23~Read The 2 Coils "));
  sendYesNo(robot->perimeter.read2Coil);
  serialPort->print(F("|e13~Block inner wheel  "));
  sendYesNo(robot->trakBlockInnerWheel);
  serialPort->print(F("|e15~Reduce Speed near Wire "));
  sendYesNo(robot->reduceSpeedNearPerimeter);
  serialPort->print(F("|e24~Actual Speed coeff "));
  serialPort->print(robot->perimeterSpeedCoeff);
  serialPort->println("}");
}

void RemoteControl::processPerimeterMenu(String pfodCmd) {
  if (pfodCmd == "e00") robot->perimeterUse = !robot->perimeterUse;
  else if (pfodCmd.startsWith("e04")) processSlider(pfodCmd, robot->perimeterTriggerMinSmag, 1);
  else if (pfodCmd.startsWith("e18")) {
    processSlider(pfodCmd, robot->MaxSpeedperiPwm, 1);
    robot->ActualSpeedPeriPWM = robot->MaxSpeedperiPwm; //immediatly see the speed change without resetting
  }
  else if (pfodCmd.startsWith("e20")) processSlider(pfodCmd, robot->DistPeriObstacleAvoid, 1);
  else if (pfodCmd.startsWith("e21")) processSlider(pfodCmd, robot->perimeterMagMaxValue, 1);
  else if (pfodCmd.startsWith("e07")) processPIDSlider(pfodCmd, "e07", robot->perimeterPID, 0.1, 100);
  else if (pfodCmd.startsWith("e08")) processSlider(pfodCmd, robot->perimeter.timedOutIfBelowSmag, 1);
  else if ((robot->developerActive) && (pfodCmd.startsWith("e09"))) {
    robot->areaInMowing = robot->areaInMowing + 1;
    if (robot->areaInMowing > 3) robot->areaInMowing = 1;
    robot->perimeter.changeArea(robot->areaInMowing);
    robot->perimeter.begin(pinPerimeterLeft, pinPerimeterRight);
  }

  else if (pfodCmd.startsWith("e10")) robot->perimeter.swapCoilPolarityLeft = !robot->perimeter.swapCoilPolarityLeft;
  else if (pfodCmd.startsWith("e22")) robot->perimeter.swapCoilPolarityRight = !robot->perimeter.swapCoilPolarityRight;
  else if (pfodCmd.startsWith("e23")) robot->perimeter.read2Coil = !robot->perimeter.read2Coil;

  else if (pfodCmd.startsWith("e11")) processSlider(pfodCmd, robot->trackingPerimeterTransitionTimeOut, 1);
  else if (pfodCmd.startsWith("e12")) processSlider(pfodCmd, robot->trackingErrorTimeOut, 1);
  else if (pfodCmd.startsWith("e13")) robot->trakBlockInnerWheel = !robot->trakBlockInnerWheel;
  else if (pfodCmd.startsWith("e15")) robot->reduceSpeedNearPerimeter = !robot->reduceSpeedNearPerimeter;


  else if (pfodCmd.startsWith("e14")) processSlider(pfodCmd, robot->perimeter.timeOutSecIfNotInside, 1);

  sendPerimeterMenu(true);
}

void RemoteControl::sendLawnSensorMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Lawn sensor`1000"));
  serialPort->print(F("|f00~Use "));
  sendYesNo(robot->lawnSensorUse);
  serialPort->print(F("|f01~Counter "));
  serialPort->print(robot->lawnSensorCounter);
  serialPort->println(F("|f02~Value f, b"));
  serialPort->print(robot->lawnSensorFront);
  serialPort->print(", ");
  serialPort->print(robot->lawnSensorBack);
  serialPort->println("}");
}

void RemoteControl::processLawnSensorMenu(String pfodCmd) {
  if (pfodCmd == "f00") robot->lawnSensorUse = !robot->lawnSensorUse;
  sendLawnSensorMenu(true);
}


void RemoteControl::sendRainMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Rain`1000"));
  serialPort->print(F("|m00~Rain Use "));
  sendYesNo(robot->rainUse);
  serialPort->print(F("|m01~Counter "));
  serialPort->print(robot->rainCounter);
  serialPort->println(F("|m02~Value"));
  serialPort->print(robot->rain);

  serialPort->println(F("|m03~DHT22 Use "));
  sendYesNo(robot->DHT22Use);
  serialPort->println(F("|m04~Temperature "));
  serialPort->print(robot->temperatureDht);
  serialPort->println(F("|m05~Humidity "));
  serialPort->print(robot->humidityDht);
  sendSlider("m06", F("Maximum Temperature"), robot->maxTemperature, "", 1 , 80, 1);

  serialPort->println("}");
}

void RemoteControl::processRainMenu(String pfodCmd) {
  if (pfodCmd == "m00") robot->rainUse = !robot->rainUse;
  else if (pfodCmd == "m03") robot->DHT22Use = !robot->DHT22Use;
  else if (pfodCmd.startsWith("m06")) processSlider(pfodCmd, robot->maxTemperature, 1);
  sendRainMenu(true);
}
void RemoteControl::sendGPSMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.GPS RFID`1000"));
  serialPort->print(F("|q00~GPS Use(Need Reboot) "));
  sendYesNo(robot->gpsUse);
  sendSlider("q03", F("GPS Baudrate"), robot->gpsBaudrate, "", 1 , 38400, 9600);
  serialPort->print(F("|q01~RFID Use : "));
  sendYesNo(robot->rfidUse);
  serialPort->print(F("|q02~Last Rfid : "));
  serialPort->print(robot->rfidTagFind);
  serialPort->println("}");
}

void RemoteControl::processGPSMenu(String pfodCmd) {
  if (pfodCmd == "q00") robot->gpsUse = !robot->gpsUse;
  else if (pfodCmd.startsWith("q01")) robot->rfidUse = !robot->rfidUse;
  else if (pfodCmd.startsWith("q03")) processSlider(pfodCmd, robot->gpsBaudrate, 1);
  sendGPSMenu(true);
}

void RemoteControl::sendByLaneMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.ByLane`500"));

  serialPort->print(F("|w01~Change Lane in use Actual: "));
  serialPort->print(robot->laneUseNr);

  serialPort->println(F("|w20~Change Next Roll Direction "));
  sendLeftRight(robot->rollDir);
  serialPort->println(F("|w02~Actual YAW and cible"));
 // serialPort->print(robot->imu.ypr.yaw / PI * 180);
  serialPort->print(" <--> ");
 // serialPort->print(robot->imuDriveHeading);
  serialPort->print(F(" deg"));

  if (robot->laneUseNr == 1)  {
    if (robot->imuDriveHeading < 0) {
      if (robot->rollDir == RIGHT) sendSlider("w04", F("yaw Opposite 1 Roll Right"), robot->yawOppositeLane1RollRight, "Deg", 1 , -110, -165);
      else sendSlider("w05", F("yaw Opposite 1 Roll Left"), robot->yawOppositeLane1RollLeft, "Deg", 1 , -110, -165);
    }
    else sendSlider("w03", F("Fixed yaw1"), robot->yawSet1, "Deg", 1, 60, 20);
  }

  if (robot->laneUseNr == 2)  {
    if (robot->imuDriveHeading < 0) {
      if (robot->rollDir == RIGHT) sendSlider("w07", F("yaw Opposite 2 Roll Right"), robot->yawOppositeLane2RollRight, "Deg", 1 , -60, -110);
      else sendSlider("w08", F("yaw Opposite 2 Roll Left"), robot->yawOppositeLane2RollLeft, "Deg", 1 , -60, -110);
    }
    else  sendSlider("w06", F("Fixed yaw2"), robot->yawSet2, "Deg", 1, 110, 60);
  }
  if (robot->laneUseNr == 3)  {
    if (robot->imuDriveHeading < 0) {
      if (robot->rollDir == RIGHT) sendSlider("w10", F("yaw Opposite 3 Roll Right"), robot->yawOppositeLane3RollRight, "Deg", 1 , -15, -80);
      else sendSlider("w11", F("yaw Opposite 3 Roll Left"), robot->yawOppositeLane3RollLeft, "Deg", 1 , -15, -80);
    }
    else sendSlider("w09", F("Fixed yaw3"), robot->yawSet3, "Deg", 1, 165, 110);
  }
  if (robot->developerActive)  sendSlider("w00", F("Dist between Lane CM"), robot->DistBetweenLane , "CM", 1, 200, 10);
  if (robot->developerActive)  sendSlider("w12", F("Lane Max Lenght M"), robot->maxLenghtByLane , "M", 1, 50, 3);


  serialPort->println("}");
}

void RemoteControl::processByLaneMenu(String pfodCmd) {
  if (pfodCmd == "w01") {
    robot->justChangeLaneDir = true;
    robot->laneUseNr = robot->laneUseNr + 1 ;
    robot->findedYaw = 999;
    robot->imuDirPID.reset();
    if (robot->laneUseNr > 3) robot->laneUseNr = 1;
  }
  if (pfodCmd == "w20") {
    if (robot->rollDir == RIGHT ) robot->rollDir = LEFT;
    else robot->rollDir = RIGHT;
  }
  else if (pfodCmd.startsWith("w03") && (robot->developerActive)) processSlider(pfodCmd, robot->yawSet1, 1);
  else if (pfodCmd.startsWith("w04")) processSlider(pfodCmd, robot->yawOppositeLane1RollRight, 1);
  else if (pfodCmd.startsWith("w05")) processSlider(pfodCmd, robot->yawOppositeLane1RollLeft, 1);
  else if (pfodCmd.startsWith("w06") && (robot->developerActive)) processSlider(pfodCmd, robot->yawSet2, 1);
  else if (pfodCmd.startsWith("w07")) processSlider(pfodCmd, robot->yawOppositeLane2RollRight, 1);
  else if (pfodCmd.startsWith("w08")) processSlider(pfodCmd, robot->yawOppositeLane2RollLeft, 1);
  else if (pfodCmd.startsWith("w09") && (robot->developerActive)) processSlider(pfodCmd, robot->yawSet3, 1);
  else if (pfodCmd.startsWith("w10")) processSlider(pfodCmd, robot->yawOppositeLane3RollRight, 1);
  else if (pfodCmd.startsWith("w11")) processSlider(pfodCmd, robot->yawOppositeLane3RollLeft, 1);
  else if (pfodCmd.startsWith("w00") && (robot->developerActive)) processSlider(pfodCmd, robot->DistBetweenLane, 1);
  else if (pfodCmd.startsWith("w12") && (robot->developerActive)) {
    processSlider(pfodCmd, robot->maxLenghtByLane, 1);
    robot->actualLenghtByLane = robot->maxLenghtByLane;
  }
  sendByLaneMenu(true);
}


void RemoteControl::sendImuMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.IMU`1000"));
  serialPort->println(F("|g00~ IMU Use :(Need Restart) "));
  sendYesNo(robot->imuUse);
  serialPort->println(F("|g11~ Compass Use : "));
  sendYesNo(robot->CompassUse);
  serialPort->println(F("|g01~Gyro Yaw/Compass Yaw"));
  serialPort->print(robot->imu.ypr.yaw / PI * 180);
  serialPort->print(F(" / "));
  serialPort->print(robot->imu.comYaw / PI * 180);
  serialPort->print(F(" deg"));
  serialPort->print(F("|g09~DriveHeading "));
  serialPort->print(robot->imuDriveHeading);
  serialPort->print(F(" deg"));
  serialPort->print(F("|g02~Pitch "));
  serialPort->print(robot->imu.ypr.pitch / PI * 180);
  serialPort->print(F(" deg"));
  serialPort->print(F("|g03~Roll "));
  serialPort->print(robot->imu.ypr.roll / PI * 180);
  serialPort->print(F(" deg"));
  serialPort->print(F("|g04~Stop mow motor during Calib  "));
  sendYesNo(robot->stopMotorDuringCalib);
  sendPIDSlider("g05", F("Dir"), robot->imuDirPID, 0.1, 20);
  sendSlider("g06", F("Calibration Max Duration in Sec"), robot->maxDurationDmpAutocalib, "Sec", 1, 100, 10);
  sendSlider("g07", F("Delay between 2 Calib in Sec"), robot->delayBetweenTwoDmpAutocalib, "Sec", 1, 600, 60);
  sendSlider("g08", F("Drift Maxi in Deg Per Second "), robot->maxDriftPerSecond, "Deg", 0.01, 0.3, 0);
  serialPort->print(F("|g20~Delete IMU calibration"));
  serialPort->print(F("|g18~Accel Gyro Initial Calibration more than 30sec duration"));
  if (robot->CompassUse) {
    serialPort->print(F("|g19~Compass calibration click to start and again to stop"));
    sendSlider("g10", F("Speed to find ComYaw % of motorSpeedMaxRpm "), robot->compassRollSpeedCoeff, "Deg", 1, 80, 30);
  }

  serialPort->println("}");
}

void RemoteControl::processImuMenu(String pfodCmd) {
  if (pfodCmd == "g00" ) {
    robot->nextTimeImuLoop = millis() + 120000; //don't read the Imu immediatly need time to save the setting and reset
    robot->imuUse = !robot->imuUse;
  }
  else if (pfodCmd == "g11" ) robot->CompassUse = !robot->CompassUse;
  else if (pfodCmd == "g04" ) robot->stopMotorDuringCalib = !robot->stopMotorDuringCalib;
  else if (pfodCmd.startsWith("g05")) processPIDSlider(pfodCmd, "g05", robot->imuDirPID, 0.1, 20);
  else if (pfodCmd.startsWith("g06")) processSlider(pfodCmd, robot->maxDurationDmpAutocalib, 1);
  else if (pfodCmd.startsWith("g07")) processSlider(pfodCmd, robot->delayBetweenTwoDmpAutocalib, 1);
  else if (pfodCmd.startsWith("g08")) processSlider(pfodCmd, robot->maxDriftPerSecond, 0.01);
  else if (pfodCmd.startsWith("g10")) processSlider(pfodCmd, robot->compassRollSpeedCoeff, 1);
  else if (pfodCmd == "g18") {
    robot->imu.deleteAccelGyroCalib();
    robot->imu.calibGyro();
  }
  else if (pfodCmd == "g19") robot->imu.calibComStartStop();

  //bber18
  else if (pfodCmd == "g20") robot->imu.deleteCompassCalib();

  sendImuMenu(true);
}

void RemoteControl::sendRemoteMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Remote R/C or Pi`1000"));
  //serialPort->print(F("|h00~Use RC "));
  //sendYesNo(robot->remoteUse);
  serialPort->print(F("|h01~Use Rasberry(Need Reboot)"));
  sendYesNo(robot->RaspberryPIUse);


  serialPort->println("}");
}

void RemoteControl::processRemoteMenu(String pfodCmd) {
  //if (pfodCmd == "h00" ) robot->remoteUse = !robot->remoteUse;
  if (pfodCmd == "h01" ) robot->RaspberryPIUse = !robot->RaspberryPIUse;
  if (pfodCmd == "h02" ) robot->printSettingSerial();  //use by pi to show all the variable in the console
  if (pfodCmd == "h03" ) robot->consoleMode = (robot->consoleMode + 1) % 5;  //use by pi to change the console mode
  if (pfodCmd == "h04" ) robot->autoReboot();  //use by pi to reset due and pi

  sendRemoteMenu(true);
}

void RemoteControl::sendBatteryMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Battery`1000"));
  serialPort->print(F("|j00~Battery "));
  serialPort->print(robot->batVoltage);
  serialPort->print(" V");
  serialPort->print(F("|j01~Monitor "));
  sendYesNo(robot->batMonitor);
  //bb add
  if (robot->developerActive) {
    sendSlider("j09", F("Charge Factor"), robot->batChgFactor, "", 0.01, 12, 9);
    sendSlider("j05", F("Battery Factor "), robot->batFactor, "", 0.01, 12, 9);
    sendSlider("j08", F("Sense factor"), robot->batSenseFactor, "", 0.01, 12, 9);
  }
  //end add
  //Console.print("batFactor=");
  //Console.println(robot->batFactor);
  sendSlider("j02", F("Go home if below Volt"), robot->batGoHomeIfBelow, "", 0.1, robot->batFull, (robot->batFull * 0.72)); // for Sony Konion cells 4.2V * 0,72= 3.024V which is pretty safe to use
  sendSlider("j12", F("Switch off if idle minutes"), robot->batSwitchOffIfIdle, "", 1, 300, 1);
  sendSlider("j03", F("Switch off if below Volt"), robot->batSwitchOffIfBelow, "", 0.1, robot->batFull, (robot->batFull * 0.72));
  serialPort->print(F("|j04~Charge "));
  serialPort->print(robot->chgVoltage);
  serialPort->print("V ");
  serialPort->print(robot->chgCurrent);
  serialPort->print("A");

  sendSlider("j06", F("Charge sense zero"), robot->chgSenseZero, "", 1, 600, 400);

  sendSlider("j10", F("charging starts if Voltage is below"), robot->startChargingIfBelow, "", 0.1, robot->batFull, (robot->batFull * 0.72));
  sendSlider("j11", F("Battery is fully charged if current is below"), robot->batFullCurrent, "", 0.1, robot->batChargingCurrentMax, 0);
  serialPort->println("}");
}

void RemoteControl::processBatteryMenu(String pfodCmd) {
  if (pfodCmd == "j01") robot->batMonitor = !robot->batMonitor;
  else if (pfodCmd.startsWith("j02")) {
    processSlider(pfodCmd, robot->batGoHomeIfBelow, 0.1);
    //Console.print("gohomeifbelow=");
    //Console.println(robot->batGoHomeIfBelow);
  }
  else if (pfodCmd.startsWith("j03")) processSlider(pfodCmd, robot->batSwitchOffIfBelow, 0.1);
  else if (pfodCmd.startsWith("j05")) processSlider(pfodCmd, robot->batFactor, 0.01);
  else if (pfodCmd.startsWith("j06")) processSlider(pfodCmd, robot->chgSenseZero, 1);
  else if (pfodCmd.startsWith("j08")) processSlider(pfodCmd, robot->batSenseFactor, 0.01);
  else if (pfodCmd.startsWith("j09")) processSlider(pfodCmd, robot->batChgFactor, 0.01);
  else if (pfodCmd.startsWith("j10")) processSlider(pfodCmd, robot->startChargingIfBelow, 0.1);
  else if (pfodCmd.startsWith("j11")) processSlider(pfodCmd, robot->batFullCurrent, 0.1);
  else if (pfodCmd.startsWith("j12")) processSlider(pfodCmd, robot->batSwitchOffIfIdle, 1);
  sendBatteryMenu(true);
}

void RemoteControl::sendStationMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Station`1000"));
  serialPort->println(F("|k05~ Bumper pressed on dock "));
  sendYesNo(robot->UseBumperDock);
  serialPort->println(F("|k07~ Reboot after charging "));
  sendYesNo(robot->autoResetActive);
  sendSlider("k00", F("Reverse Distance (CM)"), robot->stationRevDist, "", 1, 200, 0);
  sendSlider("k01", F("Roll Angle (Deg)"), robot->stationRollAngle, "", 1, 180, 0);
  sendSlider("k02", F("Accel Distance after Roll"), robot->stationForwDist, "", 1, 200, 0);
  sendSlider("k03", F("Station check Distance"), robot->stationCheckDist, "", 1, 20, 0);
  sendSlider("k06", F("Docking Speed % of MaxSpeed"), robot->dockingSpeed, "", 1, 100, 20);
  serialPort->println("}");
}

void RemoteControl::processStationMenu(String pfodCmd) {
  if (pfodCmd == "k05" ) robot->UseBumperDock = !robot->UseBumperDock;
  else if (pfodCmd.startsWith("k07")) robot->autoResetActive = !robot->autoResetActive;
  else if (pfodCmd.startsWith("k00")) processSlider(pfodCmd, robot->stationRevDist, 1);
  else if (pfodCmd.startsWith("k01")) processSlider(pfodCmd, robot->stationRollAngle, 1);
  else if (pfodCmd.startsWith("k02")) processSlider(pfodCmd, robot->stationForwDist, 1);
  else if (pfodCmd.startsWith("k03")) processSlider(pfodCmd, robot->stationCheckDist, 1);
  else if (pfodCmd.startsWith("k06")) processSlider(pfodCmd, robot->dockingSpeed, 1);
  sendStationMenu(true);
}

void RemoteControl::sendOdometryMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Odometry2D`1000"));
  //serialPort->print(F("|l00~Use "));
  //sendYesNo(robot->odometryUse);
  serialPort->print(F("|l01~Value l, r "));
  serialPort->print(robot->odometryLeft);
  serialPort->print(", ");
  serialPort->println(robot->odometryRight);
  sendSlider("l04", F("Ticks per one full revolution"), robot->odometryTicksPerRevolution, "", 1, 2800, 500);
  sendSlider("l01", F("Ticks per cm"), robot->odometryTicksPerCm, "", 0.1, 60, 10);
  sendSlider("l02", F("Wheel base cm"), robot->odometryWheelBaseCm, "", 0.1, 50, 5);
  serialPort->println("}");
}

void RemoteControl::processOdometryMenu(String pfodCmd) {
  // if (pfodCmd == "l00") robot->odometryUse = !robot->odometryUse;
  if (pfodCmd.startsWith("l01")) processSlider(pfodCmd, robot->odometryTicksPerCm, 0.1);
  else if (pfodCmd.startsWith("l02")) processSlider(pfodCmd, robot->odometryWheelBaseCm, 0.1);
  else if (pfodCmd.startsWith("l04")) processSlider(pfodCmd, robot->odometryTicksPerRevolution, 1);


  sendOdometryMenu(true);
}

void RemoteControl::sendDateTimeMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Date/time"));
  serialPort->print("|t00~");
  serialPort->print(date2str(robot->datetime.date));
  serialPort->print(", ");
  serialPort->print(time2str(robot->datetime.time));
  sendSlider("t01", dayOfWeek[robot->datetime.date.dayOfWeek], robot->datetime.date.dayOfWeek, "", 1, 6, 0);
  sendSlider("t02", "Day ", robot->datetime.date.day, "", 1, 31, 1);
  sendSlider("t03", "Month ", robot->datetime.date.month, "", 1, 12, 1);
  sendSlider("t04", "Year ", robot->datetime.date.year, "", 1, 2030, 2019);
  sendSlider("t05", "Hour ", robot->datetime.time.hour, "", 1, 23, 0);
  sendSlider("t06", "Minute ", robot->datetime.time.minute, "", 1, 59, 0);
  serialPort->println("}");
}

void RemoteControl::processDateTimeMenu(String pfodCmd) {
  if (pfodCmd.startsWith("t01")) processSlider(pfodCmd, robot->datetime.date.dayOfWeek, 1);
  else if (pfodCmd.startsWith("t02")) processSlider(pfodCmd, robot->datetime.date.day, 1);
  else if (pfodCmd.startsWith("t03")) processSlider(pfodCmd, robot->datetime.date.month, 1);
  else if (pfodCmd.startsWith("t04")) processSlider(pfodCmd, robot->datetime.date.year, 1);
  else if (pfodCmd.startsWith("t05")) processSlider(pfodCmd, robot->datetime.time.hour, 1);
  else if (pfodCmd.startsWith("t06")) processSlider(pfodCmd, robot->datetime.time.minute, 1);
  sendDateTimeMenu(true);
  //Console.print(F("setting RTC datetime: "));
  //Console.println(date2str(robot->datetime.date));
  robot->setActuator(ACT_RTC, 0);
}

void RemoteControl::sendTimerDetailMenu(int timerIdx, boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Details"));
  serialPort->print("|p0");
  serialPort->print(timerIdx);
  serialPort->print("~Use ");
  sendYesNo(robot->timer[timerIdx].active);
  int startm = time2minutes(robot->timer[timerIdx].startTime);
  int stopm = time2minutes(robot->timer[timerIdx].stopTime);
  String sidx = String(timerIdx);
  sendSlider("p1" + sidx, F("Start hour "), robot->timer[timerIdx].startTime.hour, "", 1, 23, 0);
  sendSlider("p2" + sidx, F("Start minute "), robot->timer[timerIdx].startTime.minute, "", 1, 59, 0);
  sendSlider("p3" + sidx, F("Stop hour "), robot->timer[timerIdx].stopTime.hour, "", 1, 23, 0);
  sendSlider("p4" + sidx, F("Stop minute "), robot->timer[timerIdx].stopTime.minute, "", 1, 59, 0);
  sendSlider("p6" + sidx, F("Start Dist/Station: "), robot->timer[timerIdx].startDistance, "", 1, 255, 0);
  sendSlider("pc" + sidx, F("Use Beacon: "), robot->timer[timerIdx].rfidBeacon, "", 1, 40, 0);


  serialPort->print("|p7");
  serialPort->print(timerIdx);
  serialPort->println(F("~Start Pattern  "));
  serialPort->print(robot->mowPatternNameList(robot->timer[timerIdx].startMowPattern));
  serialPort->print("|pd");
  serialPort->print(timerIdx);
  serialPort->println(F("~Start Area  "));
  serialPort->print(robot->timer[timerIdx].startArea);


  serialPort->print("|p8");
  serialPort->print(timerIdx);
  serialPort->println(F("~Start Lane NR  "));
  serialPort->print(robot->timer[timerIdx].startNrLane);

  serialPort->print("|pa");
  serialPort->print(timerIdx);
  serialPort->print("~Start Roll Dir : ");
  sendLeftRight(robot->timer[timerIdx].startRollDir);


  sendSlider("pb" + sidx, F("Start Max Lane lenght : "), robot->timer[timerIdx].startLaneMaxlengh, "", 1, 50, 0);



  for (int i = 0; i < 7; i++) {
    serialPort->print("|p5");
    serialPort->print(timerIdx);
    serialPort->print(i);
    serialPort->print("~");
    if ((robot->timer[timerIdx].daysOfWeek >> i) & 1) serialPort->print("(X)  ");
    else serialPort->print("(  )  ");
    serialPort->print(dayOfWeek[i]);
  }
  serialPort->print("|p9");
  serialPort->print(timerIdx);
  serialPort->print(F("~Set to current time"));

  /*
    serialPort->print("|pc");
    serialPort->print(timerIdx);
    serialPort->print("~ TEST THIS SETTING NOW ");
  */
  serialPort->println("}");
}

void RemoteControl::processTimerDetailMenu(String pfodCmd) {
  timehm_t time;
  boolean checkStop = false;
  boolean checkStart = false;
  int startmin, stopmin;
  int timerIdx = pfodCmd[2] - '0';
  if (pfodCmd.startsWith("p0")) {
    robot->timer[timerIdx].active = !robot->timer[timerIdx].active;
    robot->nextTimeTimer = millis() + 30000; //reset this if the mower is in station from the home command and want to start again.
  }
  else if (pfodCmd.startsWith("p1")) {
    processSlider(pfodCmd, robot->timer[timerIdx].startTime.hour, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p2")) {
    processSlider(pfodCmd, robot->timer[timerIdx].startTime.minute, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p3")) {
    processSlider(pfodCmd, robot->timer[timerIdx].stopTime.hour, 1);
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p4")) {
    processSlider(pfodCmd, robot->timer[timerIdx].stopTime.minute, 1);
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p6")) {
    processSlider(pfodCmd, robot->timer[timerIdx].startDistance, 1);
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p7")) {
    robot->timer[timerIdx].startMowPattern = (robot->timer[timerIdx].startMowPattern + 1 ) % 3;
    checkStart = true;
  }
  else if (pfodCmd.startsWith("pd")) {

    robot->timer[timerIdx].startArea = robot->timer[timerIdx].startArea + 1 ;
    if (robot->timer[timerIdx].startArea > 3) robot->timer[timerIdx].startArea = 1;
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p8")) {

    robot->timer[timerIdx].startNrLane = robot->timer[timerIdx].startNrLane + 1 ;
    if (robot->timer[timerIdx].startNrLane > 3) robot->timer[timerIdx].startNrLane = 1;
    checkStart = true;
  }
  else if (pfodCmd.startsWith("pa")) {
    robot->timer[timerIdx].startRollDir = !robot->timer[timerIdx].startRollDir;
    checkStart = true;
  }
  else if (pfodCmd.startsWith("pb")) {
    processSlider(pfodCmd, robot->timer[timerIdx].startLaneMaxlengh, 1);
    checkStart = true;
  }

  else if (pfodCmd.startsWith("pc")) {
    // adjust rfid beacon value if 0 when start the mower never read the rfid tag
    processSlider(pfodCmd, robot->timer[timerIdx].rfidBeacon, 1);
    checkStart = true;

  }

  else if (pfodCmd.startsWith("p9")) {
    robot->timer[timerIdx].startTime = robot->datetime.time; checkStop = true;
    robot->timer[timerIdx].daysOfWeek = (1 << robot->datetime.date.dayOfWeek);
  }
  else if (pfodCmd.startsWith("p5")) {
    int day = pfodCmd[3] - '0';
    robot->timer[timerIdx].daysOfWeek = robot->timer[timerIdx].daysOfWeek ^ (1 << day);
  }
  if (checkStop) {
    // adjust start time
    startmin = min(1434, time2minutes(robot->timer[timerIdx].startTime));
    minutes2time(startmin, time);
    robot->timer[timerIdx].startTime = time;
    // check stop time
    stopmin  = time2minutes(robot->timer[timerIdx].stopTime);
    stopmin = max(stopmin, startmin + 5);
    minutes2time(stopmin, time);
    robot->timer[timerIdx].stopTime = time;
    robot->startByTimer = false;
    robot->whereToStart = 0;
    robot->actualLenghtByLane = robot->maxLenghtByLane;
  } else if (checkStart) {
    // adjust stop time
    stopmin = max(5, time2minutes(robot->timer[timerIdx].stopTime));
    minutes2time(stopmin, time);
    robot->timer[timerIdx].stopTime = time;
    // check start time
    startmin = time2minutes(robot->timer[timerIdx].startTime);
    startmin = min(startmin, stopmin - 5);
    minutes2time(startmin, time);
    robot->timer[timerIdx].startTime = time;
    robot->findedYaw = 999;
    robot->imuDirPID.reset();
    robot->mowPatternCurr = robot->timer[timerIdx].startMowPattern;
    robot->laneUseNr = robot->timer[timerIdx].startNrLane;
    robot->rollDir = robot->timer[timerIdx].startRollDir;
    robot->whereToStart = robot->timer[timerIdx].startDistance;
    robot->areaToGo = robot->timer[timerIdx].startArea;
    robot->areaInMowing = 1; // we are in the station so the actual area is 1
    robot->actualLenghtByLane = robot->timer[timerIdx].startLaneMaxlengh;
    robot->startByTimer = true;
    robot->totalDistDrive = 0;


  }
  sendTimerDetailMenu(timerIdx, true);
}

void RemoteControl::sendTimerMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Timer"));
  serialPort->print(F("|i99~Use "));
  sendYesNo(robot->timerUse);
  for (int i = 0; i < MAX_TIMERS; i++) {
    serialPort->print("|i0");
    serialPort->print(i);
    serialPort->print("~");
    sendTimer(robot->timer[i]);
  }
  serialPort->println("}");
}

void RemoteControl::processTimerMenu(String pfodCmd) {
  if (pfodCmd.startsWith("i0")) {
    int timerIdx = pfodCmd[2] - '0';
    sendTimerDetailMenu(timerIdx, false);
  } else {
    if (pfodCmd.startsWith("i99")) robot->timerUse = !robot->timerUse;
    sendTimerMenu(true);
  }
}



void RemoteControl::sendFactorySettingsMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->println(F("{.Factory settings"));
  serialPort->print(F("|x0~Set factory settings (requires reboot)"));
  serialPort->println("}");
}

void RemoteControl::processFactorySettingsMenu(String pfodCmd) {
  if (pfodCmd == "x0") robot->deleteUserSettings();
  sendFactorySettingsMenu(true);
}

void RemoteControl::sendInfoMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Info`1000"));
  serialPort->print(F("|v00~Ardumower "));
  serialPort->print(VER);
  serialPort->print(F("|v01~Developer "));
  sendYesNo(robot->developerActive);
  serialPort->print(F("|v04~Stats override "));
  sendYesNo(robot->statsOverride);
  serialPort->print(F("|v02~Mowing time trip (min) "));
  serialPort->print(robot->statsMowTimeMinutesTrip);
  serialPort->print(F("|v03~Mowing time total (hrs) "));
  serialPort->print(robot->statsMowTimeHoursTotal);
  serialPort->print(F("|v05~Battery charging cycles "));
  serialPort->print(robot->statsBatteryChargingCounterTotal);
  serialPort->print(F("|v06~Battery recharged capacity trip (mAh)"));
  serialPort->print(robot->statsBatteryChargingCapacityTrip);
  serialPort->print(F("|v07~Battery recharged capacity total (Ah)"));
  serialPort->print(robot->statsBatteryChargingCapacityTotal / 1000);
  serialPort->print(F("|v08~Battery recharged capacity average (mAh)"));
  serialPort->print(robot->statsBatteryChargingCapacityAverage);
  //serialPort->print("|d01~Perimeter v");
  //serialPort->print(verToString(readPerimeterVer()));
  //serialPort->print("|d02~IMU v");
  //serialPort->print(verToString(readIMUver()));
  //serialPort->print("|d02~Stepper v");
  //serialPort->print(verToString(readStepperVer()));
  serialPort->println("}");
}

void RemoteControl::processInfoMenu(String pfodCmd) {
  if (pfodCmd == "v01") robot->developerActive = !robot->developerActive;
  if (pfodCmd == "v04") robot->statsOverride = !robot->statsOverride;


  sendInfoMenu(true);
}

void RemoteControl::sendCommandMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->print(F("{.Commands`5000"));
  //serialPort->print(F("|ro~OFF|ra~Start Now in Auto mode|rc~RC mode|"));

  serialPort->print(F("|ro~OFF"));
  serialPort->print(F("|ra~Start Now in Auto mode"));
  serialPort->println(F("|rb~Actual Status is "));
  serialPort->print(robot->statusName());
  serialPort->println(F("|rs~Actual State is "));
  serialPort->print(robot->stateName());
  serialPort->println(F("|rm~Mowing is "));
  sendOnOff(robot->motorMowEnable);
  serialPort->println(F("|rp~Pattern is "));
  serialPort->print(robot->mowPatternName());
  serialPort->print(F("|rh~Go to Station"));
  serialPort->print(F("|rk~Start Tracking"));
  serialPort->print(F("|rt~Power OFF PCB"));
  serialPort->print(F("|r1~User switch 1 is "));
  sendOnOff(robot->userSwitch1);
  serialPort->print(F("|r2~User switch 2 is "));
  sendOnOff(robot->userSwitch2);
  serialPort->print(F("|r3~User switch 3 is "));
  sendOnOff(robot->userSwitch3);
  serialPort->print("}");
  serialPort->println();
}

void RemoteControl::processCommandMenu(String pfodCmd) {
  if (pfodCmd == "ro") {
    // cmd: off
    robot->nextTimeTimer = millis() + 10000; //reset this if the mower is in station from the home command and want to start again.
    robot->setNextState(STATE_OFF, 0);
    sendCommandMenu(true);
  } else if (pfodCmd == "rh") {
    // cmd: home
    robot->periFindDriveHeading = scalePI(robot->imu.ypr.yaw);
    robot->areaToGo = 1;
    robot->whereToStart = 99999;
    robot->nextTimeTimer = millis() + 3600000;
    robot->statusCurr = BACK_TO_STATION;
    robot->setNextState(STATE_PERI_FIND, 0);
    sendCommandMenu(true);
  } else if (pfodCmd == "rr") { //coming from pi
    //use to change the consoleMode
    robot->consoleMode = (robot->consoleMode + 1 ) % 5;
    sendCommandMenu(true);
  } else if (pfodCmd == "ry") { //coming from pi
    // cmd: find other tag for new area
    robot->setNextState(STATE_PERI_STOP_TO_NEWAREA, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "ru") { //coming from pi when find a tag to help find a faster start entry (skip part of the tracking wire)
    // cmd: find  tag for fast start
    if (robot->areaToGo != 1) { // if a distance is set for start point we can't use the fast start
      robot->setNextState(STATE_PERI_STOP_TO_FAST_START, 0);
    }
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rv") { //coming from pi starttimer mqtt addon
    robot->ShowMessageln("MQTT START FROM STATION");
    robot->ActualRunningTimer = 99;
    robot->findedYaw = 999;
    robot->imuDirPID.reset();
    //robot->mowPatternCurr = 1;
    robot->startByTimer = true;
    robot->mowPatternDuration = 0;
    robot->totalDistDrive = 0;
    robot->setActuator(ACT_CHGRELAY, 0);
    robot->setNextState(STATE_STATION_REV, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rz") { //coming from pi
    // cmd: find other tag for station
    robot->setNextState(STATE_PERI_STOP_TOROLL, 0);
    sendCommandMenu(true);

  } else if (pfodCmd == "rk") {
    // cmd: track perimeter
    robot->periFindDriveHeading = scalePI(robot->imu.ypr.yaw);
    robot->statusCurr = TRACK_TO_START;
    robot->setNextState(STATE_PERI_FIND, 0);
    sendCommandMenu(true);
  } else if (pfodCmd == "ra") {
    robot->statusCurr = NORMAL_MOWING;
    robot->mowPatternDuration = 0;
    robot->motorMowEnable = true;
    if ((robot->stateCurr == STATE_STATION) || (robot->stateCurr == STATE_STATION_CHARGING)) {
      //bber40
      robot->ShowMessageln("MANUAL START FROM STATION");
      robot->ActualRunningTimer = 99;
      robot->findedYaw = 999;
      robot->imuDirPID.reset();
      //robot->mowPatternCurr = 1;
      robot->laneUseNr = 1;
      robot->rollDir = 1;
      robot->whereToStart = 1;
      robot->areaToGo = 1;
      robot->actualLenghtByLane = 40;
      robot->beaconToStart = 0;
      robot->startByTimer = true;
      robot->mowPatternDuration = 0;
      robot->totalDistDrive = 0;
      robot->setActuator(ACT_CHGRELAY, 0);
      robot->setNextState(STATE_STATION_REV, 0);
    }
    else {
      if (robot->mowPatternName() == "WIRE") {
        robot->totalDistDrive = 0;
        robot->statusCurr = TRACK_TO_START; //status change later into STATE_PERI_STOP_TOTRACK
        robot->setNextState(STATE_PERI_FIND, 0);
      }
      else {
        robot->setNextState(STATE_ACCEL_FRWRD, 0);
      }
    }
    sendCommandMenu(true);
  } else if (pfodCmd == "rc") {
    // cmd: start remote control (RC)
    /*
      robot->motorMowEnable = true;

      robot->setNextState(STATE_REMOTE, 0);
      sendCommandMenu(true);
    */
  } else if (pfodCmd == "rm") {
    // cmd: mower motor on/off
    robot->motorMowEnable = !robot->motorMowEnable;
    sendCommandMenu(true);
  } else if (pfodCmd == "rs") {
    // cmd: state
    sendCommandMenu(true);

    //only for debug remove after
  } else if (pfodCmd == "rp") {
    // cmd: pattern
    robot->mowPatternCurr = (robot->mowPatternCurr + 1 ) % 3;
    robot->setNextState(STATE_OFF, 0);
    sendCommandMenu(true);
  } else if (pfodCmd == "rt") {
    robot->batSwitchOffIfIdle = 0; // to stop immediatly the PCB
    robot->setNextState(STATE_OFF, 0);
    sendCommandMenu(true);
  } else if (pfodCmd == "r1") {
    robot->userSwitch1 = !robot->userSwitch1;
    robot->setUserSwitches();
    sendCommandMenu(true);
  } else if (pfodCmd == "r2") {
    robot->userSwitch2 = !robot->userSwitch2;
    robot->setUserSwitches();
    sendCommandMenu(true);
  } else if (pfodCmd == "r3") {
    robot->userSwitch3 = !robot->userSwitch3;
    robot->setUserSwitches();
    sendCommandMenu(true);
  }
}

void RemoteControl::sendManualMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->println(F("{^Manual navigation`1000"));
  serialPort->print(F("|nl~Left|nr~Right|nf~Forward"));
  if (   ((robot->motorLeftSpeedRpmSet  < 5)  && (robot->motorLeftSpeedRpmSet  > -5))
         &&  ((robot->motorRightSpeedRpmSet < 5)  && (robot->motorRightSpeedRpmSet > -5))  ) {
    serialPort->print(F("|nb~Reverse"));
  } else serialPort->print(F("|ns~Stop"));
  serialPort->print(F("|nm~Mow is "));
  sendOnOff(robot->motorMowEnable);
  serialPort->println("}");
}

void RemoteControl::processManualMenu(String pfodCmd) {
  if (pfodCmd == "nl") {
    // manual: left
    robot->setNextState(STATE_MANUAL, 0);
    float sign = 1.0;
    if (robot->motorLeftSpeedRpmSet < 0) sign = -1.0;
    if (sign * robot->motorLeftSpeedRpmSet >= sign * robot->motorRightSpeedRpmSet) robot->motorLeftSpeedRpmSet  = sign * robot->motorSpeedMaxRpm / 2;
    else robot->motorLeftSpeedRpmSet /= 2;
    robot->motorRightSpeedRpmSet = sign * robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  } else if (pfodCmd == "nr") {
    // manual: right
    robot->setNextState(STATE_MANUAL, 0);
    float sign = 1.0;
    if (robot->motorRightSpeedRpmSet < 0) sign = -1.0;
    if (sign * robot->motorRightSpeedRpmSet >= sign * robot->motorLeftSpeedRpmSet) robot->motorRightSpeedRpmSet  = sign * robot->motorSpeedMaxRpm / 2;
    else robot->motorRightSpeedRpmSet /= 2;
    robot->motorLeftSpeedRpmSet  = sign * robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  } else if (pfodCmd == "nf") {
    // manual: forward
    robot->setNextState(STATE_MANUAL, 0);
    robot->motorLeftSpeedRpmSet  = robot->motorSpeedMaxRpm;
    robot->motorRightSpeedRpmSet = robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  } else if (pfodCmd == "nb") {
    // manual: reverse
    robot->setNextState(STATE_MANUAL, 0);
    robot->motorLeftSpeedRpmSet  = -robot->motorSpeedMaxRpm;
    robot->motorRightSpeedRpmSet = -robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  } else if (pfodCmd == "nm") {
    // manual: mower ON/OFF
    //bber13
    robot->setNextState(STATE_MANUAL, 0);
    robot->motorMowEnable = !robot->motorMowEnable;
    sendManualMenu(true);
  } else if (pfodCmd == "ns") {
    // manual: stop
    //setNextState(STATE_OFF, 0);
    robot->motorLeftSpeedRpmSet  =  robot->motorRightSpeedRpmSet = 0;
    sendManualMenu(true);
  }
}

void RemoteControl::sendTestOdoMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->println(F("{.TestOdo`1000"));

  serialPort->println(F("|yt10~Ticks Right / Left "));
  serialPort->print(robot->odometryRight);
  serialPort->print(F(" / "));
  serialPort->print(robot->odometryLeft);
  serialPort->println();
  //serialPort->print(F("|yt8~Calib Ticks/Second"));  //to compute the ticks per second motor speed
  serialPort->print(F("|yt0~1 turn Wheel Fwd"));  //to verify and adjust the TicksPerRevolution
  serialPort->print(F("|yt1~5 turns Wheel Fwd"));  //to verify and adjust the TicksPerRevolution  and PWM right OFFSET the 2 wheel need to stop at the same time
  serialPort->print(F("|yt2~1 turn Wheel Rev"));
  serialPort->print(F("|yt3~5 turns Wheel Rev"));  //to verify the TicksPerRevolution  and PWM right OFFSET the 2 wheel stop at the same time
  serialPort->print(F("|yt4~3 meter Forward")); //to verify and adjust the TicksPerCM
  serialPort->print(F("|yt6~Rotate 180Deg"));  //to verify and adjust the odometryWheelBaseCm
  serialPort->print(F("|yt5~Rotate 360Deg"));  //to verify and adjust the odometryWheelBaseCm
  serialPort->println(F("|yt7~Rotate Non Stop"));


  serialPort->println("}");

}

void RemoteControl::processTestOdoMenu(String pfodCmd) {
  if (pfodCmd == "yt0") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorRightSpeedRpmSet = robot->motorSpeedMaxRpm ;
    robot->stateEndOdometryRight = robot->odometryRight + robot->odometryTicksPerRevolution;
    robot->stateEndOdometryLeft = robot->odometryLeft + robot->odometryTicksPerRevolution;
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }
  else if (pfodCmd == "yt1") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorRightSpeedRpmSet = robot->motorSpeedMaxRpm;
    robot->stateEndOdometryRight = robot->odometryRight + 5 * robot->odometryTicksPerRevolution;
    robot->stateEndOdometryLeft = robot->odometryLeft + 5 * robot->odometryTicksPerRevolution;
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }
  else if (pfodCmd == "yt2") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorRightSpeedRpmSet = -robot->motorSpeedMaxRpm ;
    robot->stateEndOdometryRight = robot->odometryRight - robot->odometryTicksPerRevolution;
    robot->stateEndOdometryLeft = robot->odometryLeft - robot->odometryTicksPerRevolution;
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }
  else if (pfodCmd == "yt3") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorRightSpeedRpmSet = -robot->motorSpeedMaxRpm;
    robot->stateEndOdometryRight = robot->odometryRight - 5 * robot->odometryTicksPerRevolution;
    robot->stateEndOdometryLeft = robot->odometryLeft - 5 * robot->odometryTicksPerRevolution;
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }
  else if (pfodCmd == "yt4") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorRightSpeedRpmSet = robot->motorSpeedMaxRpm;
    robot->stateEndOdometryRight = robot->odometryRight + 300.00 * robot->odometryTicksPerCm;
    robot->stateEndOdometryLeft = robot->odometryLeft + 300.00 * robot->odometryTicksPerCm;
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }
  else if (pfodCmd == "yt6") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorSpeedMaxRpm / 2;
    robot->motorRightSpeedRpmSet = -robot->motorSpeedMaxRpm / 2;
    robot->stateEndOdometryRight = robot->odometryRight - (int)18000 * (robot->odometryTicksPerCm * PI * robot->odometryWheelBaseCm / 36000);
    robot->stateEndOdometryLeft = robot->odometryLeft + (int)18000 * (robot->odometryTicksPerCm * PI * robot->odometryWheelBaseCm / 36000);
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }
  else if (pfodCmd == "yt5") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorSpeedMaxRpm / 2;
    robot->motorRightSpeedRpmSet = -robot->motorSpeedMaxRpm / 2;
    robot->stateEndOdometryRight = robot->odometryRight - (int)36000 * (robot->odometryTicksPerCm * PI * robot->odometryWheelBaseCm / 36000);
    robot->stateEndOdometryLeft = robot->odometryLeft + (int)36000 * (robot->odometryTicksPerCm * PI * robot->odometryWheelBaseCm / 36000);
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }
  else if (pfodCmd == "yt7") {
    robot->odometryRight = robot->odometryLeft = 0;
    robot->motorLeftSpeedRpmSet = robot->motorSpeedMaxRpm / 3;
    robot->motorRightSpeedRpmSet = -robot->motorSpeedMaxRpm / 3;
    robot->stateEndOdometryRight = robot->odometryRight - (int)36000 * (robot->odometryTicksPerCm * PI * robot->odometryWheelBaseCm / 360);
    robot->stateEndOdometryLeft = robot->odometryLeft + (int)36000 * (robot->odometryTicksPerCm * PI * robot->odometryWheelBaseCm / 360);
    robot->setNextState(STATE_TEST_MOTOR, robot->rollDir);
    sendTestOdoMenu(true);
  }

}


void RemoteControl::sendCompassMenu(boolean update) {
  if (update) serialPort->print("{:"); else serialPort->println(F("{^Compass`1000"));
  serialPort->print(F("|cw~West|ce~East|cn~North "));
  serialPort->print(robot->imu.ypr.yaw / PI * 180);
  serialPort->println(F("|cs~South|cm~Mow"));
  serialPort->println("}");
}

void RemoteControl::processCompassMenu(String pfodCmd) {
  if (pfodCmd == "cm") {
    robot->motorMowEnable = !robot->motorMowEnable;
    sendCompassMenu(true);
  } else if (pfodCmd == "cn") {
    robot->yawToFind = 0;
    robot->setNextState(STATE_TEST_COMPASS, robot->rollDir);
    sendCompassMenu(true);
  } else if (pfodCmd == "cs") {
    robot->yawToFind = 179;
    robot->setNextState(STATE_TEST_COMPASS, robot->rollDir);
    sendCompassMenu(true);
  } else if (pfodCmd == "cw") {
    robot->yawToFind = -90;
    robot->setNextState(STATE_TEST_COMPASS, robot->rollDir);
    sendCompassMenu(true);
  } else if (pfodCmd == "ce") {
    robot->yawToFind = 90;
    robot->setNextState(STATE_TEST_COMPASS, robot->rollDir);
    sendCompassMenu(true);
  }

}

// process pfodState
void RemoteControl::run() {
  if (pfodState == PFOD_PLOT_BAT) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 60000;
      serialPort->print(((unsigned long)millis() / 60000));
      serialPort->print(",");
      serialPort->print(robot->batVoltage);
      serialPort->print(",");
      serialPort->print(robot->chgVoltage);
      serialPort->print(",");
      serialPort->print(robot->chgCurrent);
      serialPort->print(",");
      serialPort->println(robot->batCapacity);
    }
  } else if (pfodState == PFOD_PLOT_ODO2D) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 500;
      serialPort->print(robot->odometryX);
      serialPort->print(",");
      serialPort->println(robot->odometryY);
    }
  } else if (pfodState == PFOD_PLOT_IMU) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 200;
      serialPort->print((millis() / 1000));
      serialPort->print(",");
      serialPort->print(robot->imu.ypr.yaw / PI * 180);
      serialPort->print(",");
      serialPort->print(robot->imu.ypr.pitch / PI * 180);
      serialPort->print(",");
      serialPort->print(robot->imu.ypr.roll / PI * 180);
      serialPort->print(",");
      serialPort->println(robot->imu.comYaw / PI * 180);
    }
  } else if (pfodState == PFOD_PLOT_SENSOR_COUNTERS) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 200;
      serialPort->print((float(millis()) / 1000.0f));
      serialPort->print(",");
      serialPort->print(robot->stateCurr);
      serialPort->print(",");
      serialPort->print(robot->motorLeftSenseCounter);
      serialPort->print(",");
      serialPort->print(robot->motorRightSenseCounter);
      serialPort->print(",");
      serialPort->print(robot->motorMowSenseCounter);
      serialPort->print(",");
      serialPort->print(robot->bumperLeftCounter);
      serialPort->print(",");
      serialPort->print(robot->bumperRightCounter);
      //serialPort->print(",");
      //serialPort->print(robot->sonarDistCounter);
      serialPort->print(",");
      serialPort->print(robot->perimeterCounter);
      serialPort->print(",");
      serialPort->print(robot->lawnSensorCounter);
      serialPort->print(",");
      serialPort->print(robot->rainCounter);
      serialPort->print(",");
      serialPort->print(robot->dropLeftCounter);
      serialPort->print(",");
      serialPort->println(robot->dropRightCounter);
    }
  } else if (pfodState == PFOD_PLOT_SENSORS) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 200;
      serialPort->print((float(millis()) / 1000.0f));
      serialPort->print(",");
      serialPort->print(robot->stateCurr);
      serialPort->print(",");
      serialPort->print(robot->motorLeftPower);
      serialPort->print(",");
      serialPort->print(robot->motorRightPower);
      serialPort->print(",");
      serialPort->print(robot->motorMowPower);
      serialPort->print(",");
      serialPort->print(robot->sonarDistLeft);
      serialPort->print(",");
      serialPort->print(robot->sonarDistCenter);
      serialPort->print(",");
      serialPort->print(robot->sonarDistRight);
      serialPort->print(",");
      serialPort->print(robot->perimeter.isInside(0));
      serialPort->print(",");
      serialPort->print(robot->lawnSensor);
      serialPort->print(",");
      serialPort->print(robot->rain);
      serialPort->print(",");
      serialPort->print(robot->dropLeft);
      serialPort->print(",");
      serialPort->println(robot->dropRight);
    }
  } else if (pfodState == PFOD_PLOT_PERIMETER) {
    if (millis() >= nextPlotTime) {

      if (perimeterCaptureIdx >= RAW_SIGNAL_SAMPLE_SIZE * 3)
        perimeterCaptureIdx = 0;

      if (perimeterCaptureIdx == 0) {
        // Get new Perimeter sample to plot
        memcpy(perimeterCapture, robot->perimeter.getRawSignalSample(0), RAW_SIGNAL_SAMPLE_SIZE);
      }

      nextPlotTime = millis() + 200;
      serialPort->print(perimeterCapture[perimeterCaptureIdx / 3]);
      serialPort->print(",");
      serialPort->print(robot->perimeterMag);
      serialPort->print(",");
      serialPort->print(robot->perimeter.getSmoothMagnitude(0));
      serialPort->print(",");
      serialPort->print(robot->perimeter.isInside(0));
      serialPort->print(",");
      serialPort->print(robot->perimeterCounter);
      serialPort->print(",");
      serialPort->print(!robot->perimeter.signalTimedOut(0));
      serialPort->print(",");
      serialPort->println(robot->perimeter.getFilterQuality(0));
      perimeterCaptureIdx++;
    }
  } else if (pfodState == PFOD_PLOT_GPS) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 200;
      float lat, lon;
      unsigned long age;

    }
  } else if (pfodState == PFOD_PLOT_GPS2D) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 500;
      serialPort->print(robot->gpsX);
      serialPort->print(",");
      serialPort->println(robot->gpsY);
    }
  } else if (pfodState == PFOD_PLOT_MOTOR) {
    if (millis() >= nextPlotTime) {
      nextPlotTime = millis() + 50;
      serialPort->print((float(millis()) / 1000.0f));
      serialPort->print(",");
      serialPort->print(robot->motorLeftRpmCurr);
      serialPort->print(",");
      serialPort->print(robot->motorRightRpmCurr);
      serialPort->print(",");
      serialPort->print(robot->motorLeftSpeedRpmSet);
      //serialPort->print(robot->motorLeftPID.w);
      serialPort->print(",");
      serialPort->print(robot->motorRightSpeedRpmSet);
      //serialPort->print(robot->motorRightPID.w);
      serialPort->print(",");
      serialPort->print(robot->motorLeftPWMCurr);
      serialPort->print(",");
      serialPort->print(robot->motorRightPWMCurr);
      serialPort->print(",");
      serialPort->print(robot->motorLeftPID.eold);
      serialPort->print(",");
      serialPort->println(robot->motorRightPID.eold);
    }
  }
}

// process serial input from pfod App
boolean RemoteControl::readSerial() {
  dataFromPi = false;
  boolean res = false;
  while (serialPort->available() > 0) {
    res = true;
    if (serialPort->available() > 0) {
      char ch = serialPort->read();
      //Console.print("pfod ch=");
      //Console.println(ch);
      if (ch == '}') pfodCmdComplete = true;
      else if (ch == '{') pfodCmd = "";
      else pfodCmd += ch;
    }
    if (pfodCmdComplete) {
      //Console.print("pfod cmd=");
      //Console.println(pfodCmd);
      pfodState = PFOD_MENU;

      if (pfodCmd == ".") {
        robot->ConsoleToPfod = false;
        sendMainMenu(false);
      }
      else if (pfodCmd == "m1") {
        // Console
        serialPort->println(F("{=Console}"));
        pfodState = PFOD_CONSOLE;
        robot->ConsoleToPfod = true;
      }
      else if (pfodCmd == "y1") {
        // plot battery
        serialPort->println(F("{=battery|time min`0|battery V`1|charge V`1|charge A`2|capacity Ah`3}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_BAT;
      }
      else if (pfodCmd == "y2") {
        // plot odometry 2d
        serialPort->println(F("{=odometry2d|position`0~~~x|`~~~y}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_ODO2D;
      }
      else if (pfodCmd == "y3") {
        // plot IMU
        serialPort->print(F("{=IMU|time s`0|yaw`1~180~-180|pitch`2~90~-90|roll`3~90~-90|ComYaw`4~180~-180}"));
        //serialPort->println(F("|comX`4~2~-2|comY`4|comZ`4}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_IMU;
      }
      else if (pfodCmd == "y5") {
        // plot sensor counters
        serialPort->print(F("{=Sensor counters`300|time s`0|state`1|motL`2|motR`3|motM`4|bumL`5|bumR`6"));
        serialPort->println(F("|son`7|peri`8|lawn`9|rain`10|dropL`11|dropR`12}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_SENSOR_COUNTERS;
      }
      else if (pfodCmd == "y6") {
        // plot perimeter spectrum
        /*serialPort->print(F("{=perimeter spectrum`"));
          serialPort->print(perimeter.getFilterBinCount());
          serialPort->print(F("|freq (Hz)`0|magnitude`0~60~-1|selected band`0~60~-1}"));*/
        serialPort->println(F("{=Perimeter`128|sig`1|mag`2|smag`3|in`4|cnt`5|on`6|qty`7}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_PERIMETER;
      }
      else if (pfodCmd == "y7") {
        // plot sensor values
        serialPort->print(F("{=Sensors`300|time s`0|state`1|motL`2|motR`3|motM`4|sonL`5|sonC`6"));
        serialPort->println(F("|sonR`7|peri`8|lawn`9|rain`10|dropL`11|dropR`12}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_SENSORS;
      }
      else if (pfodCmd == "y8") {
        // plot GPS
        serialPort->println(F("{=GPS`300|time s`0|hdop`1|sat`2|spd`3|course`4|alt`5|lat`6|lon`7}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_GPS;
      }
      else if (pfodCmd == "y10") {
        // plot GPS 2d
        serialPort->println(F("{=gps2d|position`0~~~x|`~~~y}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_GPS2D;
      }
      else if (pfodCmd == "y11") {
        // motor control
        serialPort->println(F("{=Motor control`300|time s`0|lrpm_curr`1|rrpm_curr`2|lrpm_set`3|rrpm_set`4|lpwm`5|rpwm`6|lerr`7|rerr`8}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_MOTOR;
      }
      else if (pfodCmd == "yp") sendPlotMenu(false);
      else if (pfodCmd == "y4") sendErrorMenu(false);
      else if (pfodCmd == "yt") sendTestOdoMenu(false);
      else if (pfodCmd == "w") sendByLaneMenu(false);
      else if (pfodCmd == "n") sendManualMenu(false);
      else if (pfodCmd == "s") sendSettingsMenu(false);
      else if (pfodCmd == "r") sendCommandMenu(false);
      else if (pfodCmd == "c") sendCompassMenu(false);
      else if (pfodCmd == "t") sendDateTimeMenu(false);
      else if (pfodCmd == "i") sendTimerMenu(false);
      else if (pfodCmd == "in") sendInfoMenu(false);


      else if (pfodCmd.startsWith("a")) processMotorMenu(pfodCmd);
      else if (pfodCmd.startsWith("b")) processBumperMenu(pfodCmd);
      else if (pfodCmd.startsWith("c")) processCompassMenu(pfodCmd);
      else if (pfodCmd.startsWith("d")) processSonarMenu(pfodCmd);
      else if (pfodCmd.startsWith("e")) processPerimeterMenu(pfodCmd);
      else if (pfodCmd.startsWith("f")) processLawnSensorMenu(pfodCmd);
      else if (pfodCmd.startsWith("g")) processImuMenu(pfodCmd);
      else if (pfodCmd.startsWith("h")) processRemoteMenu(pfodCmd);
      else if (pfodCmd.startsWith("i")) processTimerMenu(pfodCmd);
      else if (pfodCmd.startsWith("j")) processBatteryMenu(pfodCmd);
      else if (pfodCmd.startsWith("k")) processStationMenu(pfodCmd);
      else if (pfodCmd.startsWith("l")) processOdometryMenu(pfodCmd);
      else if (pfodCmd.startsWith("m")) processRainMenu(pfodCmd);
      else if (pfodCmd.startsWith("n")) processManualMenu(pfodCmd);
      else if (pfodCmd.startsWith("o")) processMowMenu(pfodCmd);
      else if (pfodCmd.startsWith("p")) processTimerDetailMenu(pfodCmd);
      else if (pfodCmd.startsWith("q")) processGPSMenu(pfodCmd);
      else if (pfodCmd.startsWith("r")) processCommandMenu(pfodCmd);
      else if (pfodCmd.startsWith("s")) processSettingsMenu(pfodCmd);
      else if (pfodCmd.startsWith("t")) processDateTimeMenu(pfodCmd);
      else if (pfodCmd.startsWith("u")) processDropMenu(pfodCmd);
      else if (pfodCmd.startsWith("v")) processInfoMenu(pfodCmd);
      else if (pfodCmd.startsWith("w")) processByLaneMenu(pfodCmd);
      else if (pfodCmd.startsWith("x")) processFactorySettingsMenu(pfodCmd);
      else if (pfodCmd.startsWith("yt")) processTestOdoMenu(pfodCmd);
      else if (pfodCmd.startsWith("z")) processErrorMenu(pfodCmd);
      else if (pfodCmd.startsWith("RFID")) {
        robot->rfidTagFind = pfodCmd.substring(4);
        robot->newTagFind();
      }

      else {
        // no match
        serialPort->println("{}");
      }
      serialPort->flush();
      pfodCmd = "";
      pfodCmdComplete = false;
    }
  }
  return res;
}

void RemoteControl::processPI(String RpiCmd, float v1, float v2, float v3) {
  pfodCmd = RpiCmd;
  value1 = v1;
  value2 = v2;
  value3 = v3;
  dataFromPi = true;
  if (pfodCmd.startsWith("a")) processMotorMenu(pfodCmd);
  else if (pfodCmd.startsWith("b")) processBumperMenu(pfodCmd);
  else if (pfodCmd.startsWith("c")) processCompassMenu(pfodCmd);
  else if (pfodCmd.startsWith("d")) processSonarMenu(pfodCmd);
  else if (pfodCmd.startsWith("e")) processPerimeterMenu(pfodCmd);
  else if (pfodCmd.startsWith("f")) processLawnSensorMenu(pfodCmd);
  else if (pfodCmd.startsWith("g")) processImuMenu(pfodCmd);
  else if (pfodCmd.startsWith("h")) processRemoteMenu(pfodCmd);
  else if (pfodCmd.startsWith("i")) processTimerMenu(pfodCmd);
  else if (pfodCmd.startsWith("j")) processBatteryMenu(pfodCmd);
  else if (pfodCmd.startsWith("k")) processStationMenu(pfodCmd);
  else if (pfodCmd.startsWith("l")) processOdometryMenu(pfodCmd);
  else if (pfodCmd.startsWith("m")) processRainMenu(pfodCmd);
  else if (pfodCmd.startsWith("n")) processManualMenu(pfodCmd);
  else if (pfodCmd.startsWith("o")) processMowMenu(pfodCmd);
  else if (pfodCmd.startsWith("p")) processTimerDetailMenu(pfodCmd);
  else if (pfodCmd.startsWith("q")) processGPSMenu(pfodCmd);
  else if (pfodCmd.startsWith("r")) processCommandMenu(pfodCmd);
  else if (pfodCmd.startsWith("s")) processSettingsMenu(pfodCmd);
  else if (pfodCmd.startsWith("t")) processDateTimeMenu(pfodCmd);
  else if (pfodCmd.startsWith("u")) processDropMenu(pfodCmd);
  else if (pfodCmd.startsWith("v")) processInfoMenu(pfodCmd);
  else if (pfodCmd.startsWith("w")) processByLaneMenu(pfodCmd);
  else if (pfodCmd.startsWith("x")) processFactorySettingsMenu(pfodCmd);
  else if (pfodCmd.startsWith("yt")) processTestOdoMenu(pfodCmd);
  else if (pfodCmd.startsWith("z")) processErrorMenu(pfodCmd);
}
