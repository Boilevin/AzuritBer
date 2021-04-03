/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri
  Copyright (c) 2014-2015 by Stefan Manteuffel
  Copyright (c) 2015 by Uwe Zimprich
  PriÂ²vate-use only! (you need to ask for a commercial-use)

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


//extern "C" char* sbrk(int incr);//use to know the freeram
//see also line 1968 and 264 to reactivate
//bb

#include "adcman.h"
#include "flashmem.h"
#include "robot.h"
#include "buzzer.h"
#include "pinman.h"
#include "perimeter.h"
#include "mower.h"
#include "timer.h"
#include "DHT.h"
#include "RpiRemote.h"




#define MAGIC 52
#define ADDR_USER_SETTINGS 2000 //New adress to avoid issue if Azurit1.09 is install
#define ADDR_ERR_COUNTERS 500
//carrefull that the  ADDR 600 is used by the IMU calibration
#define ADDR_ROBOT_STATS 800

//Setting for DHT22------------------------------------
#define DHTPIN 49                  // temperature sensor DHT22
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

//-----------------------------------------------------

//Setting for Raspberry -----------------------------------
RpiRemote MyRpi;

//bber



char* stateNames[] = {"OFF ", "RC  ", "FORW", "ROLL", "REV ", "CIRC", "ERR ", "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK", "STREV",
                      "STROL", "STFOR", "MANU", "ROLW", "POUTFOR", "POUTREV", "POUTROLL", "POBSREV", "POBSROLL", "POBSFRWD", "POBSCIRC", "NEXTLANE", "POUTSTOP", "LANEROL1", "LANEROL2",
                      "ROLLTOIN", "WAITREPEAT", "FRWODO", "TESTCOMPAS", "ROLLTOTRACK",
                      "STOPTOTRACK", "AUTOCALIB", "ROLLTOFINDYAW", "TESTMOTOR", "FINDYAWSTOP", "STOPONBUMPER",
                      "STOPCALIB", "SONARTRIG", "STOPSPIRAL", "MOWSPIRAL", "ROT360", "NEXTSPIRE", "ESCAPLANE",
                      "TRACKSTOP", "ROLLTOTAG", "STOPTONEWAREA", "ROLL1TONEWAREA", "DRIVE1TONEWAREA", "ROLL2TONEWAREA", "DRIVE2TONEWAREA", "WAITSIG2", "STOPTONEWAREA", "ROLLSTOPTOTRACK",
                      "STOPTOFASTSTART", "CALIBMOTORSPEED", "ACCELFRWRD"
                     };

char* statusNames[] = {"WAIT", "NORMALMOWING", "SPIRALEMOWING", "BACKTOSTATION", "TRACKTOSTART", "MANUAL", "REMOTE", "ERROR", "STATION", "TESTING", "SIGWAIT" , "WIREMOWING"
                      };


char* mowPatternNames[] = {"RAND", "LANE",  "WIRE" , "ZIGZAG"};
char* consoleModeNames[] = {"sen_counters", "sen_values", "perimeter", "off", "Tracking"};


unsigned long StartReadAt;
int distance_find;
unsigned long EndReadAt;
unsigned long ReadDuration;



Robot::Robot() {
  name = "Generic";
  developerActive = false;
  rc.setRobot(this);
  MyRpi.setRobot(this);

  stateLast = stateCurr = stateNext = STATE_OFF;
  statusCurr = WAIT; //initialise the status on power up

  stateTime = 0;
  idleTimeSec = 0;
  statsMowTimeTotalStart = false;
  //mowPatternCurr = MOW_RANDOM;

  odometryLeft = odometryRight = 0;
  odometryLeftLastState = odometryLeftLastState2 = odometryRightLastState = odometryRightLastState2 = LOW;
  odometryTheta = odometryX = odometryY = 0;
  prevYawCalcOdo = 0;
  motorRightRpmCurr = motorLeftRpmCurr = 0;
  lastMotorRpmTime = 0;
  lastSetMotorSpeedTime = 0;
  motorLeftSpeedRpmSet =  motorRightSpeedRpmSet = 0;
  motorLeftPWMCurr = motorRightPWMCurr = 0;
  motorRightSenseADC = motorLeftSenseADC = 0;
  motorLeftSenseCurrent = motorRightSenseCurrent = 0;
  motorLeftPower = motorRightPower = 0;
  motorLeftSenseCounter = motorRightSenseCounter = 0;
  motorZeroSettleTime = 0;
  motorLeftZeroTimeout = 0;
  motorRightZeroTimeout = 0;
  rotateLeft = true;

  remoteSteer = remoteSpeed = remoteMow = remoteSwitch = 0;
  remoteSteerLastTime = remoteSpeedLastTime = remoteMowLastTime = remoteSwitchLastTime = 0;
  remoteSteerLastState = remoteSpeedLastState = remoteMowLastState = remoteSwitchLastState = LOW;

  motorMowRpmCounter = 0;
  motorMowRpmLastState = LOW;
  motorMowEnable = false;
  motorMowForceOff = false;
  //ignoreRfidTag = false;
  motorMowSpeedPWMSet = motorMowSpeedMinPwm;  //use to set the speed of the mow motor
  motorMowPWMCurr = 0;
  motorMowSenseADC = 0;
  motorMowSenseCurrent  = 0;
  motorMowPower = 0;
  motorMowSenseCounter = 0;
  motorMowSenseErrorCounter = 0;
  motorMowPwmCoeff = 100;
  lastMowSpeedPWM = 0;
  timeToAddMowMedian = 0;
  lastSetMotorMowSpeedTime = 0;
  nextTimeCheckCurrent = 0;
  lastTimeMotorMowStuck = 0;
  totalDistDrive = 0;
  whereToResetSpeed = 50000;  // initial value to 500 meters

  bumperLeftCounter = bumperRightCounter = 0;
  bumperLeft = bumperRight = false;

  dropLeftCounter = dropRightCounter = 0;                                                                                              // Dropsensor - Absturzsensor
  dropLeft = dropRight = false;                                                                                                        // Dropsensor - Absturzsensor

  gpsLat = gpsLon = gpsX = gpsY = 0;
  robotIsStuckCounter = 0;

  imuDriveHeading = 0;
  periFindDriveHeading = 0;
  remoteDriveHeading = 0;
  imuRollHeading = 0;
  imuRollDir = LEFT;
  rollDir = LEFT;


  perimeterMag = 0;
  perimeterInside = true;
  perimeterCounter = 0;
  perimeterLastTransitionTime = 0;
  perimeterTriggerTime = 0;
  areaInMowing = 1;

  lawnSensorCounter = 0;
  lawnSensor = false;
  lawnSensorFront = lawnSensorFrontOld = lawnSensorBack = lawnSensorBackOld = 0;

  rain = false;
  rainCounter = 0;

  sonarLeftUse = sonarRightUse = sonarCenterUse = false;
  sonarDistCenter = sonarDistRight = sonarDistLeft = 0;
  sonarObstacleTimeout = 0;
  distToObstacle = 0;
  sonarSpeedCoeff = 1;

  batVoltage = 0;
  batRefFactor = 0;
  batCapacity = 0;
  lastTimeBatCapacity = 0;
  chgVoltage = 0;
  chgCurrent = 0;

  memset(errorCounterMax, 0, sizeof errorCounterMax);
  memset(errorCounter, 0, sizeof errorCounterMax);

  loopsPerSec = 0;
  loopsPerSecCounter = 0;
  buttonCounter = 0;
  ledState = 0;

  consoleMode = CONSOLE_OFF;
  nextTimeButtonCheck = 0;
  nextTimeInfo = 0;
  nextTimePrintConsole = 0;
  nextTimeMotorSense = 0;
  nextTimeIMU = 0;
  nextTimeCheckTilt = 0;
  nextTimeOdometry = 0;
  nextTimeOdometryInfo = 0;
  nextTimeBumper = 0;
  nextTimeDrop = 0;                                                                                                                    // Dropsensor - Absturzsensor
  //nextTimeSonar = 0;
  nextTimeBattery = 0;
  nextTimeCheckBattery = 0;
  nextTimePerimeter = 0;
  nextTimeLawnSensor = 0;
  nextTimeLawnSensorCheck = 0;
  nextTimeTimer = millis() + 60000;
  nextTimeRTC = 0;
  nextTimeGPS = 0;
  nextTimeCheckIfStuck = 0;
  nextTimePfodLoop = 0;
  nextTimeGpsRead = 0;
  nextTimeImuLoop = 0;
  nextTimeRain = 0;
  lastMotorMowRpmTime = millis();
  nextTimeButton = 0;
  nextTimeErrorCounterReset = 0;
  nextTimeErrorBeep = 0;
  nextTimeMotorControl = 0;
  nextTimeMotorImuControl = 0;
  nextTimeMotorOdoControl = 0;
  nextTimePidCompute = 0;
  nextTimeMotorPerimeterControl = 0;
  nextTimeMotorMowControl = 0;
  nextTimeRotationChange = 0;
  nextTimeAddYawMedian = 0;
  nextTimeRobotStats = 0;
  delayToReadVoltageStation = 0;
  //bb
  // nextTimeImuUse = 0;
  statsMowTimeMinutesTripCounter = 0;
  statsBatteryChargingCounter = 0;
  lastTimeForgetWire = 0; //use in peritrack
  nextTimeToDmpAutoCalibration = 0; //at this time the mower start calibration on first positive lane stop
  //bber17
  RollToInsideQty = 0;
  findedYaw = 999; //use the first time set the compass and the Gyro have the same direction with state roll to find yaw
  highGrassDetect = false;
  motorRightPID.Kp = motorLeftPID.Kp;
  motorRightPID.Ki = motorLeftPID.Ki;
  motorRightPID.Kd = motorLeftPID.Kd;
  gpsReady = false;
  MyrpiStatusSync = false;
}


char* Robot::stateName() {
  return stateNames[stateCurr];
}

char* Robot::statusName() {
  return statusNames[statusCurr];
}


char* Robot::mowPatternName() {
  return mowPatternNames[mowPatternCurr];
}

char* Robot::mowPatternNameList(byte mowPatternIndex) {
  return mowPatternNames[mowPatternIndex];
}
/*
  int freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
  }
*/

void watchdogSetup(void) {}


void Robot::loadSaveRobotStats(boolean readflag) {
  int addr = ADDR_ROBOT_STATS;

  if (readflag) {
    Console.println(F("Load Robot Stats"));
  }
  else {
    Console.println(F("Save Robot Stats"));
  }

  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    Console.println(F("PLEASE CHECK IF YOUR ROBOT STATS ARE CORRECT"));
  }
  eereadwrite(readflag, addr, statsMowTimeMinutesTrip);
  eereadwrite(readflag, addr, statsMowTimeMinutesTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCounterTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTrip);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityAverage);
  // <----------------------------new robot stats to save goes here!----------------
  Console.print(F("Robot Stats address Start = "));
  Console.println(ADDR_ROBOT_STATS);
  Console.print(F("Robot Stats address Stop = "));
  Console.println(addr);
}

void Robot::loadSaveErrorCounters(boolean readflag) {
  if (readflag) Console.println(F("Load ErrorCounters"));
  else Console.println(F("Save ErrorCounters"));
  int addr = ADDR_ERR_COUNTERS;
  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    Console.println(F("EEPROM ERR COUNTERS: NO EEPROM ERROR DATA"));
    Console.println(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, errorCounterMax);
  Console.print(F("ErrorCounters address Start="));
  Console.println(ADDR_ERR_COUNTERS);
  Console.print(F("ErrorCounters address Stop="));
  Console.println(addr);
}

void Robot::loadSaveUserSettings(boolean readflag) {


  int addr = ADDR_USER_SETTINGS;
  short magic = 0;
  if (!readflag) magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic

  if ((readflag) && (magic != MAGIC)) {

    Console.println(F("EEPROM USERDATA: NO EEPROM USER DATA"));
    Console.println(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }

  eereadwrite(readflag, addr, developerActive);
  eereadwrite(readflag, addr, motorAccel);
  eereadwrite(readflag, addr, motorSpeedMaxRpm);
  eereadwrite(readflag, addr, motorSpeedMaxPwm);
  eereadwrite(readflag, addr, motorPowerMax);
  eereadwrite(readflag, addr, motorSenseRightScale);
  eereadwrite(readflag, addr, motorSenseLeftScale);
  eereadwrite(readflag, addr, motorRollDegMax);
  eereadwrite(readflag, addr, motorRollDegMin);
  eereadwrite(readflag, addr, DistPeriOutRev);
  eereadwrite(readflag, addr, motorPowerIgnoreTime);
  eereadwrite(readflag, addr, motorForwTimeMax);
  eereadwrite(readflag, addr, motorMowSpeedMaxPwm);
  eereadwrite(readflag, addr, motorMowPowerMax);
  eereadwrite(readflag, addr, motorMowSpeedMinPwm);
  eereadwrite(readflag, addr, motorMowSenseScale);
  eereadwrite(readflag, addr, motorLeftPID.Kp);
  eereadwrite(readflag, addr, motorLeftPID.Ki);
  eereadwrite(readflag, addr, motorLeftPID.Kd);
  eereadwrite(readflag, addr, motorMowPID.Kp);
  eereadwrite(readflag, addr, motorMowPID.Ki);
  eereadwrite(readflag, addr, motorMowPID.Kd);
  eereadwrite(readflag, addr, motorBiDirSpeedRatio1);
  eereadwrite(readflag, addr, motorBiDirSpeedRatio2);
  eereadwrite(readflag, addr, motorLeftSwapDir);
  eereadwrite(readflag, addr, motorRightSwapDir);
  eereadwrite(readflag, addr, bumperUse);
  eereadwrite(readflag, addr, sonarUse);
  eereadwrite(readflag, addr, sonarCenterUse);
  eereadwrite(readflag, addr, sonarLeftUse);
  eereadwrite(readflag, addr, sonarRightUse);
  eereadwrite(readflag, addr, sonarTriggerBelow);
  eereadwrite(readflag, addr, perimeterUse);
  eereadwrite(readflag, addr, perimeter.timedOutIfBelowSmag);
  eereadwrite(readflag, addr, perimeterTriggerMinSmag);
  eereadwrite(readflag, addr, trackingErrorTimeOut);
  eereadwrite(readflag, addr, motorTickPerSecond);
  eereadwrite(readflag, addr, perimeterOutRevTime);
  eereadwrite(readflag, addr, perimeterTrackRollTime );
  eereadwrite(readflag, addr, perimeterTrackRevTime);
  eereadwrite(readflag, addr, perimeterPID.Kp);
  eereadwrite(readflag, addr, perimeterPID.Ki);
  eereadwrite(readflag, addr, perimeterPID.Kd);
  eereadwrite(readflag, addr, perimeter.signalCodeNo);
  eereadwrite(readflag, addr, perimeter.swapCoilPolarityLeft);
  eereadwrite(readflag, addr, perimeter.timeOutSecIfNotInside);
  eereadwrite(readflag, addr, trakBlockInnerWheel);
  eereadwrite(readflag, addr, lawnSensorUse);
  eereadwrite(readflag, addr, imuUse);
  eereadwrite(readflag, addr, stopMotorDuringCalib);
  eereadwrite(readflag, addr, imuDirPID.Kp);
  eereadwrite(readflag, addr, imuDirPID.Ki);
  eereadwrite(readflag, addr, imuDirPID.Kd);
  eereadwrite(readflag, addr, imuRollPID.Kp);
  eereadwrite(readflag, addr, imuRollPID.Ki);
  eereadwrite(readflag, addr, imuRollPID.Kd);
  eereadwrite(readflag, addr, remoteUse);
  eereadwrite(readflag, addr, batMonitor);
  eereadwrite(readflag, addr, batGoHomeIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfIdle);
  eereadwrite(readflag, addr, batFactor);
  eereadwrite(readflag, addr, batChgFactor);
  eereadwrite(readflag, addr, chgSenseZero);  //float adress free for something else
  eereadwrite(readflag, addr, batSenseFactor);
  eereadwrite(readflag, addr, batFullCurrent);
  eereadwrite(readflag, addr, startChargingIfBelow);
  eereadwrite(readflag, addr, stationRevDist);
  eereadwrite(readflag, addr, stationRollAngle);
  eereadwrite(readflag, addr, stationForwDist);
  eereadwrite(readflag, addr, stationCheckDist);
  eereadwrite(readflag, addr, UseBumperDock);
  eereadwrite(readflag, addr, odometryTicksPerRevolution);
  eereadwrite(readflag, addr, odometryTicksPerCm);
  eereadwrite(readflag, addr, odometryWheelBaseCm);
  eereadwrite(readflag, addr, autoResetActive);
  eereadwrite(readflag, addr, CompassUse);
  eereadwrite(readflag, addr, twoWayOdometrySensorUse);   // char YES NO adress free for something else
  eereadwrite(readflag, addr, buttonUse);
  eereadwrite(readflag, addr, userSwitch1);
  eereadwrite(readflag, addr, userSwitch2);
  eereadwrite(readflag, addr, userSwitch3);
  eereadwrite(readflag, addr, timerUse);
  eereadwrite(readflag, addr, timer);
  eereadwrite(readflag, addr, rainUse);
  eereadwrite(readflag, addr, gpsUse);
  eereadwrite(readflag, addr, stuckIfGpsSpeedBelow);
  eereadwrite(readflag, addr, gpsBaudrate);  //baudrate for the GPS
  eereadwrite(readflag, addr, dropUse);
  eereadwrite(readflag, addr, statsOverride);
  eereadwrite(readflag, addr, freeboolean); // old bluetoothUse only define into mower.cpp free for other boolean
  eereadwrite(readflag, addr, freeboolean);  // old esp8266Use only define into mower.cpp free for other boolean
  eereadwriteString(readflag, addr, esp8266ConfigString);
  eereadwrite(readflag, addr, tiltUse);
  eereadwrite(readflag, addr, trackingPerimeterTransitionTimeOut);
  eereadwrite(readflag, addr, motorMowForceOff);
  eereadwrite(readflag, addr, MaxSpeedperiPwm);
  ActualSpeedPeriPWM = MaxSpeedperiPwm; //initialise Actual tracking speed
  eereadwrite(readflag, addr, RollTimeFor45Deg);  //unsigned long adress free for something else
  eereadwrite(readflag, addr, DistPeriObstacleAvoid);
  eereadwrite(readflag, addr, circleTimeForObstacle);
  eereadwrite(readflag, addr, DistPeriOutRev);
  eereadwrite(readflag, addr, motorRightOffsetFwd);
  eereadwrite(readflag, addr, motorRightOffsetRev);
  eereadwrite(readflag, addr, perimeterMagMaxValue);
  eereadwrite(readflag, addr, SpeedOdoMin);
  eereadwrite(readflag, addr, SpeedOdoMax);
  eereadwrite(readflag, addr, yawSet1);
  eereadwrite(readflag, addr, yawSet2);
  eereadwrite(readflag, addr, yawSet3);
  eereadwrite(readflag, addr, yawOppositeLane1RollRight);
  eereadwrite(readflag, addr, yawOppositeLane2RollRight);
  eereadwrite(readflag, addr, yawOppositeLane3RollRight);
  eereadwrite(readflag, addr, yawOppositeLane1RollLeft);
  eereadwrite(readflag, addr, yawOppositeLane2RollLeft);
  eereadwrite(readflag, addr, yawOppositeLane3RollLeft);
  eereadwrite(readflag, addr, DistBetweenLane);
  eereadwrite(readflag, addr, maxLenghtByLane);
  actualLenghtByLane = maxLenghtByLane; //initialise lenght lane
  eereadwrite(readflag, addr, perimeter.swapCoilPolarityRight);
  eereadwrite(readflag, addr, perimeter.read2Coil);
  eereadwrite(readflag, addr, maxDriftPerSecond);
  eereadwrite(readflag, addr, delayBetweenTwoDmpAutocalib);
  eereadwrite(readflag, addr, maxDurationDmpAutocalib);
  eereadwrite(readflag, addr, mowPatternDurationMax);
  eereadwrite(readflag, addr, DistPeriOutStop);
  eereadwrite(readflag, addr, DHT22Use);
  eereadwrite(readflag, addr, RaspberryPIUse);
  eereadwrite(readflag, addr, sonarToFrontDist);
  eereadwrite(readflag, addr, maxTemperature);
  eereadwrite(readflag, addr, dockingSpeed);
  eereadwrite(readflag, addr, rfidUse);
  eereadwrite(readflag, addr, compassRollSpeedCoeff);
  if (readflag)
  {
    Console.print(F("UserSettings are read from EEprom Address : "));
    Console.print(ADDR_USER_SETTINGS);
    Console.print(F(" To "));
    Console.println(addr);
    motorInitialSpeedMaxPwm = motorSpeedMaxPwm; //the Pi can change the speed so store the initial value to restore after PFND for example
  }
  else
  {
    Console.print(F("UserSettings are saved from EEprom Address : "));
    Console.print(ADDR_USER_SETTINGS);
    Console.print(F(" To "));
    Console.println(addr);
  }

}

void Robot::loadUserSettings() {
  //return; // use in one shot to reset all the usersetting if acces on console is not possible
  loadSaveUserSettings(true);
}


void Robot::printSettingSerial() {

  // ------- wheel motors ---------------------------------------------------------
  Console.println("---------- wheel motors -----------");
  Console.print  ("motorAccel                 : ");
  Console.println(motorAccel);
  Console.print  ("motorSpeedMaxRpm           : ");
  Console.println(motorSpeedMaxRpm);
  Console.print  ("motorSpeedMaxPwm           : ");
  Console.println(motorSpeedMaxPwm);
  Console.print  ("motorPowerMax              : ");
  Console.println(motorPowerMax);
  Console.print  ("motorSenseRightScale       : ");
  Console.println(motorSenseRightScale);
  Console.print  ("motorSenseLeftScale        : ");
  Console.println(motorSenseLeftScale);
  watchdogReset();
  Console.print  ("motorPowerIgnoreTime       : ");
  Console.println(motorPowerIgnoreTime);
  Console.print  ("motorZeroSettleTime        : ");
  Console.println(motorZeroSettleTime);
  Console.print  ("motorRollDegMax            : ");
  Console.println(motorRollDegMax);
  Console.print  ("motorRollDegMin            : ");
  Console.println(motorRollDegMin);
  Console.print  ("DistPeriOutRev             : ");
  Console.println(DistPeriOutRev);
  watchdogReset();
  Console.print  ("DistPeriOutStop            : ");
  Console.println(DistPeriOutStop);
  Console.print  ("motorForwTimeMax           : ");
  Console.println(motorForwTimeMax);
  Console.print  ("DistPeriObstacleAvoid      : ");
  Console.println(DistPeriObstacleAvoid);
  Console.print  ("circleTimeForObstacle      : ");
  Console.println(circleTimeForObstacle);
  Console.print  ("motorRightOffsetFwd        : ");
  Console.println(motorRightOffsetFwd);
  watchdogReset();
  Console.print  ("motorRightOffsetRev        : ");
  Console.println(motorRightOffsetRev);
  Console.print  ("SpeedOdoMin                : ");
  Console.println(SpeedOdoMin);
  Console.print  ("SpeedOdoMax                : ");
  Console.println(SpeedOdoMax);
  Console.print  ("motorTickPerSecond         : ");
  Console.println(motorTickPerSecond);
  
 // Console.print  ("motorBiDirSpeedRatio1      : ");
 // Console.println(motorBiDirSpeedRatio1);
  watchdogReset();
 //Console.print  ("motorBiDirSpeedRatio2                      : ");
 //Console.println(motorBiDirSpeedRatio2);

  Console.print  ("motorLeftPID.Kp            : ");
  Console.println(motorLeftPID.Kp);
  Console.print  ("motorLeftPID.Ki            : ");
  Console.println(motorLeftPID.Ki);
  Console.print  ("motorLeftPID.Kd            : ");
  Console.println(motorLeftPID.Kd);

  Console.print  ("motorRightSwapDir          : ");
  Console.println(motorRightSwapDir);
  Console.print  ("motorLeftSwapDir           : ");
  Console.println(motorLeftSwapDir);
  Console.print  ("motorRightOffsetFwd        : ");
  Console.println(motorRightOffsetFwd);
  Console.print  ("motorRightOffsetRev        : ");
  Console.println(motorRightOffsetRev);
  watchdogReset();
  delayWithWatchdog (2000);
  // ------ mower motor -----------------------------------
  Console.println("---------- mower motor -----------------");
  Console.print  ("motorMowForceOff         : ");
  Console.println(motorMowForceOff);
  Console.print  ("motorMowAccel            : ");
  Console.println(motorMowAccel);
  Console.print  ("motorMowSpeedMaxPwm      : ");
  Console.println(motorMowSpeedMaxPwm);
  Console.print  ("(motorMowSpeedMinPwm     : ");
  Console.println(motorMowSpeedMinPwm);
  Console.print  ("motorMowPowerMax         : ");
  Console.println(motorMowPowerMax);
  Console.print  ("motorMowSenseScale       : ");
  Console.println(motorMowSenseScale);
 
  watchdogReset();
  // ------ bumper ------------------------------------
  Console.println("---------- bumper -----------------");
  Console.print  ("bumperUse           : ");
  Console.println(bumperUse, 1);

  // ------ drop -------------------------------------
  Console.println("---------- drop -----------------");
  Console.print  ("dropUse            : ");
  Console.println(dropUse, 1);
  Console.print  ("dropContact        : ");
  Console.println(dropcontact, 1);
  delayWithWatchdog (2000);
  // ------ rain -------------------------------------
  Console.println("---------- rain ----------------");
  Console.print  ("rainUse             : ");
  Console.println(rainUse, 1);

  // ------ DHT22 Temperature -----------------------
  Console.println("----------  DHT22 Temperature ---");
  Console.print  ("DHT22Use           : ");
  Console.println(DHT22Use, 1);
  Console.print  ("MaxTemperature     : ");
  Console.println(maxTemperature);

  watchdogReset();

  // ------ sonar -----------------------------------
  Console.println(F("---------- sonar ---------------"));
  Console.print  ("sonarUse              : ");
  Console.println(sonarUse, 1);
  Console.print  ("sonarLikeBumper       : ");
  Console.println(sonarLikeBumper, 1);
  Console.print  ("sonarLeftUse        : ");
  Console.println(sonarLeftUse, 1);
  Console.print  ("sonarRightUse       : ");
  Console.println(sonarRightUse, 1);
  Console.print  ("sonarCenterUse      : ");
  Console.println(sonarCenterUse, 1);
  Console.print  ("sonarTriggerBelow   : ");
  Console.println(sonarTriggerBelow);
  Console.print  ("sonarToFrontDist    : ");
  Console.println(sonarToFrontDist);

  watchdogReset();
  delayWithWatchdog (2000);
  // ------ perimeter --------------------------
  Console.println("---------- perimeter ------");
  Console.print  ("perimeterUse             : ");
  Console.println(perimeterUse, 1);
  Console.print  ("perimeterTriggerMinSmag  : ");
  Console.println(perimeterTriggerMinSmag);
  Console.print  ("MaxSpeedperiPwm          : ");
  Console.println(MaxSpeedperiPwm);
  Console.print  ("perimeterTrackRollTime   : ");
  Console.println(perimeterTrackRollTime);
  Console.print  ("perimeterTrackRevTime    : ");
  Console.println(perimeterTrackRevTime);
  Console.print  ("perimeterPID.Kp          : ");
  Console.println(perimeterPID.Kp);
  Console.print  ("perimeterPID.Ki          : ");
  Console.println( perimeterPID.Ki);
  watchdogReset();
  Console.print  ("perimeterPID.Kd          : ");
  Console.println(perimeterPID.Kd);
  Console.print  ("trackingPerimeterTransitionTimeOut: ");
  Console.println(trackingPerimeterTransitionTimeOut);
  Console.print  ("trackingErrorTimeOut     : ");
  Console.println(trackingErrorTimeOut);
  Console.print  ("perimeterMagMaxValue     : ");
  Console.println(perimeterMagMaxValue);
  Console.print  ("swapCoilPolarityRight    : ");
  watchdogReset();
  Console.println(perimeter.swapCoilPolarityRight);
  Console.print  ("swapCoilPolarityLeft     : ");
  Console.println(perimeter.swapCoilPolarityLeft);
  Console.print  ("read2Coil                : ");
  Console.println(perimeter.read2Coil);
  Console.print  ("trackingBlockInnerWheelWhilePerimeterStrug : ");
  Console.println(trakBlockInnerWheel, 1);
  Console.print  ("DistPeriOutRev           : ");
  Console.println(DistPeriOutRev);
  Console.print  ("DistPeriObstacleRev      : ");
  Console.println(DistPeriObstacleRev);
  Console.print  ("DistPeriOutForw          : ");
  Console.println(DistPeriOutForw);
  Console.print  ("DistPeriObstacleForw     : ");
  Console.println(DistPeriObstacleForw);
  watchdogReset();
  delayWithWatchdog (2000);
  // ------ By Lanes mowing --------------------- 
  Console.println(F("---------- By Lanes mowing ----------"));
  Console.print  (F("yawSet1                   : "));
  Console.println(yawSet1);
  Console.print  (F("yawSet2                   : "));
  Console.println(yawSet2);
  Console.print  (F("yawSet3                   : "));
  Console.println(yawSet3);
  Console.print  (F("yawOppositeLane1RollRight : "));
  Console.println(yawOppositeLane1RollRight);
  Console.print  (F("yawOppositeLane2RollRight : "));
  Console.println(yawOppositeLane2RollRight);
  Console.print  (F("yawOppositeLane3RollRight : "));
  Console.println(yawOppositeLane3RollRight);
  Console.print  (F("yawOppositeLane1RollLeft  : "));
  Console.println(yawOppositeLane1RollLeft);
  watchdogReset();
  Console.print  (F("yawOppositeLane2RollLeft  : "));
  Console.println(yawOppositeLane2RollLeft);
  Console.print  (F("yawOppositeLane3RollLeft  : "));
  Console.println(yawOppositeLane3RollLeft);
  Console.print  (F("DistBetweenLane           : "));
  Console.println(DistBetweenLane);
  Console.print  (F("maxLenghtByLane           : "));
  Console.println(maxLenghtByLane);
  watchdogReset();
  // ------ lawn sensor ----------------------------
  Console.println(F("---------- lawn sensor---------"));
  Console.print  (F("lawnSensorUse            : "));
  Console.println(lawnSensorUse, 1);

  // ------  IMU (compass/accel/gyro) ------ 
  Console.println(F("---------- IMU (compass/accel/gyro) ---- "));
  Console.print  (F("imuUse                : "));
  Console.println( imuUse, 1);
  Console.print  (F("CompassUse            : "));
  Console.println(CompassUse);
  Console.print  (F("stopMotorDuringCalib  : "));
  Console.println(stopMotorDuringCalib, 1);
  Console.print  (F("imuDirPID.Kp          : "));
  Console.println(imuDirPID.Kp);
  Console.print  (F("imuDirPID.Ki          : "));
  Console.println(imuDirPID.Ki);
  Console.print  (F("imuDirPID.Kd          : "));
  Console.println( imuDirPID.Kd);
  watchdogReset();
  Console.print  (F("maxDriftPerSecond     : "));
  Console.println(maxDriftPerSecond);
  Console.print  (F("delayBetweenTwoDmpAutocalib : "));
  Console.println(delayBetweenTwoDmpAutocalib);
  Console.print  (F("maxDurationDmpAutocalib     : "));
  Console.println(maxDurationDmpAutocalib);
  Console.print  (F("compassRollSpeedCoeff       : "));
  Console.println(compassRollSpeedCoeff);
  delayWithWatchdog (2000);
  watchdogReset();
  // ------ model R/C ------------------------------
  Console.println(F("---------- model R/C ---------"));
  Console.print  (F("remoteUse                   : "));
  Console.println(remoteUse, 1);

  // ------ battery ----------------------------
  Console.println(F("---------- battery --------  "));
  Console.print  (F("batMonitor           : "));
  Console.println( batMonitor, 1);
  Console.print  (F("batGoHomeIfBelow     : "));
  Console.println(batGoHomeIfBelow);
  Console.print  (F("batSwitchOffIfBelow  : "));
  Console.println(batSwitchOffIfBelow);
  Console.print  (F("batSwitchOffIfIdle   : "));
  Console.println(batSwitchOffIfIdle);
  Console.print  (F("batFactor            : "));
  Console.println( batFactor);
  Console.print  (F("batChgFactor         : "));
  Console.println( batChgFactor);
  Console.print  (F("batFull              : "));
  Console.println( batFull);
  watchdogReset();
  Console.print  (F("batChargingCurrentMax: "));
  Console.println(batChargingCurrentMax);
  Console.print  (F("batFullCurrent       : "));
  Console.println(batFullCurrent);
  Console.print  (F("startChargingIfBelow : "));
  Console.println(startChargingIfBelow);
  Console.print  (F("chargingTimeout      : "));
  Console.println(chargingTimeout);
  Console.print  (F("chgSenseZero         : "));
  Console.println(chgSenseZero);
  Console.print  (F("batSenseFactor       : "));
  Console.println( batSenseFactor);
  Console.print  (F("chgSense             : "));
  Console.println(chgSense);
  Console.print  (F("chgChange            : "));
  Console.println(chgChange, 1);
  Console.print  (F("chgNull              : "));
  Console.println(chgNull, 1);
  watchdogReset();
  // ------  charging station -----------------------------------------------------
  Console.println(F("---------- charging station ----------------------------------"));
  Console.print  (F("stationRevDist     : "));
  Console.println(stationRevDist);
  Console.print  (F("stationRollAngle   : "));
  Console.println(stationRollAngle);
  Console.print  (F("stationForwDist    : "));
  Console.println(stationForwDist);
  Console.print  (F("stationCheckDist   : "));
  Console.println(stationCheckDist);
  Console.print  (F("UseBumperDock      : "));
  Console.println(UseBumperDock);
  Console.print  (F("dockingSpeed       : "));
  Console.println(dockingSpeed);
  Console.print  (F("autoResetActive    : "));
  Console.println(autoResetActive);

  watchdogReset();


  // ------ odometry --------------------------------------------------------------
  Console.println(F("---------- odometry ------------------------------------------"));
  Console.print  (F("odometryTicksPerRevolution : "));
  Console.println( odometryTicksPerRevolution);
  Console.print  (F("odometryTicksPerCm         : "));
  Console.println( odometryTicksPerCm);
  Console.print  (F("odometryWheelBaseCm        : "));
  Console.println( odometryWheelBaseCm);



  watchdogReset();

  // ----- GPS ----------------------------------------------------------------------
  Console.println(F("---------- GPS -----------------------------------------------"));
  Console.print  (F("gpsUse                : "));
  Console.println(gpsUse, 1);
  Console.print  (F("stuckIfGpsSpeedBelow  : "));
  Console.println(stuckIfGpsSpeedBelow);
  Console.print  (F("gpsBaudrate           : "));
  Console.println(gpsBaudrate);
  //bber35
  // ----- RFID ----------------------------------------------------------------------
  Console.println(F("---------- RFID ----------- "));
  Console.print  (F("rfidUse         : "));
  Console.println(rfidUse, 1);
  watchdogReset();
  // ----- RASPBERRY PI -------------- 
  Console.println(F("---------- RASPBERRY PI------ "));
  Console.print  (F("RaspberryPIUse  : "));
  Console.println(RaspberryPIUse, 1);

  // ----- other ----------------------------------------------------
  Console.println(F("---------- other ------------"));
  Console.print  (F("buttonUse              : "));
  Console.println(buttonUse, 1);
  Console.print  (F("mowPatternDurationMax  : "));
  Console.println(mowPatternDurationMax);

  watchdogReset();

  // ----- user-defined switch ----------------------------------------
  Console.println(F("---------- user-defined switch -------"));
  Console.print  (F("userSwitch1       : "));
  Console.println(userSwitch1, 1);
  Console.print  (F("userSwitch2       : "));
  Console.println(userSwitch2, 1);
  Console.print  (F("userSwitch3       : "));
  Console.println(userSwitch3, 1);
  watchdogReset();
  // ----- timer --------------------------------------------------------------------
  Console.println(F("---------- timer ----------- "));
  Console.print  (F("timerUse       : "));
  Console.println(timerUse, 1);

  // ----- bluetooth ---------------------------------------------------------------
  Console.println(F("---------- bluetooth-----------------------------------------"));
  Console.print  (F("bluetoothuse   : "));
  Console.println(bluetoothUse, 1);

  // ----- esp8266 -----------------------------------------------------------------
  Console.println(F("---------- esp8266 ------------------------------------------"));
  Console.print  (F("esp8266Use          : "));
  Console.println(esp8266Use, 1);
  Console.print  (F("esp8266ConfigString : "));
  Console.println(esp8266ConfigString);
  watchdogReset();
  // -------robot stats--------------------------------------------------------------
  Console.println(F("---------- robot stats ---------------------------------------"));
  Console.print  (F("statsMowTimeMinutesTrip                    : "));
  Console.println(statsMowTimeMinutesTrip);
  Console.print  (F("statsMowTimeMinutesTotal                   : "));
  Console.println(statsMowTimeMinutesTotal);
  Console.print  (F("statsBatteryChargingCounterTotal           : "));
  Console.println(statsBatteryChargingCounterTotal);
  Console.print  (F("statsBatteryChargingCapacityTrip in mAh    : "));
  Console.println(statsBatteryChargingCapacityTrip);
  Console.print  (F("statsBatteryChargingCapacityTotal in Ah    : "));
  Console.println(statsBatteryChargingCapacityTotal / 1000);
  Console.print  (F("statsBatteryChargingCapacityAverage in mAh : "));
  Console.println(statsBatteryChargingCapacityAverage);
  watchdogReset();
  //return;


}


void Robot::saveUserSettings() {
  Console.println(F("START TO SAVE USER SETTINGS PLEASE WAIT"));
  loadSaveUserSettings(false);

}


void Robot::deleteUserSettings() {
  int addr = ADDR_USER_SETTINGS;
  Console.println(F("ALL USER SETTINGS ARE DELETED PLEASE RESTART THE DUE"));
  eewrite(addr, (short)0); // magic
}

void Robot::deleteRobotStats() {
  statsMowTimeMinutesTrip = statsMowTimeMinutesTotal = statsBatteryChargingCounterTotal =
                              statsBatteryChargingCapacityTotal = statsBatteryChargingCapacityTrip = 0;
  loadSaveRobotStats(false);
  Console.println(F("ALL ROBOT STATS ARE DELETED"));
}

void Robot::addErrorCounter(byte errType) {
  // increase error counters (both temporary and maximum error counters)
  if (errorCounter[errType] < 255) errorCounter[errType]++;
  if (errorCounterMax[errType] < 255) errorCounterMax[errType]++;
}

void Robot::resetErrorCounters() {
  Console.println(F("resetErrorCounters"));
  for (int i = 0; i < ERR_ENUM_COUNT; i++) errorCounter[i] = errorCounterMax[i] = 0;
  loadSaveErrorCounters(false);
  resetMotorFault();
}

void Robot::resetMotorFault() {
  if (digitalRead(pinMotorLeftFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    Console.println(F("Reset motor left fault"));
  }
  if  (digitalRead(pinMotorRightFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    Console.println(F("Reset motor right fault"));
  }
  if (digitalRead(pinMotorMowFault) == LOW) {
    digitalWrite(pinMotorMowEnable, LOW);
    digitalWrite(pinMotorMowEnable, HIGH);
    Console.println(F("Reset motor mow fault"));
  }
}


void Robot::checkErrorCounter() {
  if (millis() >= nextTimeErrorCounterReset) {
    // reset all temporary error counters after 30 seconds (maximum error counters still continue to count)
    for (int i = 0; i < ERR_ENUM_COUNT; i++) errorCounter[i] = 0;
    nextTimeErrorCounterReset = millis() + 30000; // 30 sec
  }
  if (stateCurr != STATE_OFF) {
    for (int i = 0; i < ERR_ENUM_COUNT; i++) {
      // set to fatal error if any temporary error counter reaches 10
      if (errorCounter[i] > 10) {
        Console.print("Error Counter > 10 for counter num ");
        Console.println(i);
        setNextState(STATE_ERROR, 0);
      }
    }
  }
}


void Robot::autoReboot() {
  //this feature use the watchdog to perform a restart of the due
  if (RaspberryPIUse) {
    Console.println(F("Due reset after 1 secondes, send a command to Pi for restart also"));
    MyRpi.sendCommandToPi("RestartPi");
  }
  else
  {
    Console.println(F("Due reset after 1 secondes"));
  }
  delay(1000);
  watchdogReset();
  delay(20000); // this reset the due.
}

// ---- motor RPM (interrupt) --------------------------------------------------------------
// mower motor RPM driver
void Robot::setMotorMowRPMState(boolean motorMowRpmState) {
  if (motorMowRpmState != motorMowRpmLastState) {
    motorMowRpmLastState = motorMowRpmState;
    if (motorMowRpmLastState) motorMowRpmCounter++;
  }
}


// ---- RC (interrupt) --------------------------------------------------------------
// RC remote control helper
// convert ppm time (us) to percent (-100..+100)
// ppmtime: zero stick pos: 1500 uS
//          right stick pos: 2000 uS
//          left stick pos: 1000 uS
int Robot::rcValue(int ppmTime) {
  int value = (int) (((double)((ppmTime) - 1500)) / 3.4);
  if ((value < 5) && (value > -5)) value = 0;  //  ensures exact zero position
  return value;
}

// RC remote control driver
// 1. save time (uS) and RC channel states (HI/LO)
// 2. if new state is LO, evaluate ppm time for channel
void Robot::setRemotePPMState(unsigned long timeMicros, boolean remoteSpeedState, boolean remoteSteerState, boolean remoteMowState, boolean remoteSwitchState) {
  if (remoteSpeedState != remoteSpeedLastState) {
    remoteSpeedLastState = remoteSpeedState;
    if (remoteSpeedState) remoteSpeedLastTime = timeMicros; else remoteSpeed = rcValue(timeMicros - remoteSpeedLastTime);
  }
  if (remoteSteerState != remoteSteerLastState) {
    remoteSteerLastState = remoteSteerState;
    if (remoteSteerState) remoteSteerLastTime = timeMicros; else remoteSteer = rcValue(timeMicros - remoteSteerLastTime);
  }
  if (remoteMowState != remoteMowLastState) {
    remoteMowLastState = remoteMowState;
    if (remoteMowState) remoteMowLastTime = timeMicros; else remoteMow = max(0, (rcValue(timeMicros - remoteMowLastTime) + 100) / 2);
  }
  if (remoteSwitchState != remoteSwitchLastState) {
    remoteSwitchLastState = remoteSwitchState;
    if (remoteSwitchState) remoteSwitchLastTime = timeMicros; else remoteSwitch = rcValue(timeMicros - remoteSwitchLastTime);
  }
}

// sets mower motor actuator
// - ensures that the motor is not switched to 100% too fast (motorMowAccel)
// - ensures that the motor voltage is not higher than motorMowSpeedMaxPwm
void Robot::setMotorMowPWM(int pwm, boolean useAccel) {
  unsigned long TaC = millis() - lastSetMotorMowSpeedTime;    // sampling time in millis
  lastSetMotorMowSpeedTime = millis();
  if (TaC > 1000) TaC = 1;
  //bber13
  if ( (!useAccel)) {  //accel is not use when stop the blade on tilt
    motorMowPWMCurr = pwm;
  }
  else {
    motorMowPWMCurr += int(TaC) * (pwm - motorMowPWMCurr) / motorMowAccel;
  }
  setActuator(ACT_MOTOR_MOW, min(motorMowSpeedMaxPwm, max(0, motorMowPWMCurr)));
}


// sets wheel motor actuators
// - driver protection: delays polarity change until motor speed (EMV) is zero
//   http://wiki.ardumower.de/images/a/a5/Motor_polarity_switch_protection.png
// - optional: ensures that the motors (and gears) are not switched to 0% (or 100%) too fast (motorAccel)
void Robot::setMotorPWM(int pwmLeft, int pwmRight, boolean useAccel) {
  int TaC = int(millis() - lastSetMotorSpeedTime);    // sampling time in millis
  lastSetMotorSpeedTime = millis();
  if (TaC > 1000) TaC = 1;
  /*
    if (stateCurr != STATE_OFF) {
    Console.print(stateNames[stateCurr]);
    Console.print(" Les valeurs demandÃ©es a ");
    Console.print (millis());
    Console.print(" TaC=");
    Console.print (TaC);
    Console.print(" Useaccel=");
    Console.print (useAccel);
    Console.print(" pwmLeft=");
    Console.print (pwmLeft);
    Console.print ("  motorLeftZeroTimeout : ");
    Console.print (motorLeftZeroTimeout);
    Console.print(" motorLeftPWMCurr=");
    Console.println (motorLeftPWMCurr);
    }
  */
  // ----- driver protection (avoids driver explosion) ----------
  if ( ((pwmLeft < 0) && (motorLeftPWMCurr > 0)) || ((pwmLeft > 0) && (motorLeftPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      Console.print("WARNING PROTECTION ON LEFT MOTOR ");
      Console.print("  motorLeftPWMCurr=");
      Console.print (motorLeftPWMCurr);
      Console.print("  pwmLeft=");
      Console.print (pwmLeft);
      Console.print(" state ");
      Console.println(stateNames[stateCurr]);
      if (motorLeftZeroTimeout != 0) pwmLeft = motorLeftPWMCurr - motorLeftPWMCurr * ((float)TaC) / 200.0; // reduce speed
    }
  }
  if ( ((pwmRight < 0) && (motorRightPWMCurr > 0)) || ((pwmRight > 0) && (motorRightPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      Console.print("WARNING PROTECTION ON RIGHT MOTOR ");
      Console.print("  motorRightPWMCurr=");
      Console.print (motorRightPWMCurr);
      Console.print("  pwmRight=");
      Console.print (pwmRight);
      Console.print("  On state ");
      Console.println(stateNames[stateCurr]);
      if (motorRightZeroTimeout != 0) pwmRight = motorRightPWMCurr - motorRightPWMCurr * ((float)TaC) / 200.0; // reduce speed
    }
  }


  if (useAccel) {
    // http://phrogz.net/js/framerate-independent-low-pass-filter.html
    // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
    if ((abs(pwmLeft) - abs(motorLeftPWMCurr)) > 0) motorLeftChange = motorAccel; //we are in accel mode
    else motorLeftChange = 500; // we are in breaking mode
    if ((abs(pwmRight) - abs(motorRightPWMCurr)) > 0) motorRightChange = motorAccel;
    else motorRightChange = 500;


    motorLeftPWMCurr += int(TaC) * (pwmLeft - motorLeftPWMCurr) / motorLeftChange;
    motorRightPWMCurr +=  int(TaC) * (pwmRight - motorRightPWMCurr) / motorRightChange;
    /*
        Console.print(" motorLeftZeroTimeout=");
        Console.print (motorLeftZeroTimeout);
        Console.print(" motorLeftChange=");
        Console.print (motorLeftChange);
        Console.print(" pwmRight=");
        Console.print (pwmRight);
        Console.print(" motorRightPWMCurr=");
        Console.print (motorRightPWMCurr);
        Console.print(" pwmLeft=");
        Console.print (pwmLeft);
        Console.print(" motorLeftPWMCurr=");
        Console.println (motorLeftPWMCurr);
         if (motorLeftPWMCurr >255) {
          motorLeftPWMCurr=255;
          Console.println ("motorLeftPWMCurr 2555555555555555555555555555555555555555");
          }
          if (motorRightPWMCurr >255) motorRightPWMCurr=255;
    */

  }
  /*
    else
    {
    motorLeftPWMCurr = pwmLeft;
    motorRightPWMCurr = pwmRight;
    }
  */
  motorLeftPWMCurr = pwmLeft;
  motorRightPWMCurr = pwmRight;

  if (abs(motorLeftRpmCurr) < 1) motorLeftZeroTimeout = max(0, ((int)(motorLeftZeroTimeout - TaC)) );
  else motorLeftZeroTimeout = 500;
  if (abs(motorRightRpmCurr) < 1) motorRightZeroTimeout = max(0, ((int)(motorRightZeroTimeout - TaC)) );
  else motorRightZeroTimeout = 500;

  if (stateCurr != STATE_OFF) {
    /*
      Console.print (millis());
      Console.print("; Right/Left PWM ;");
      Console.print (motorRightPWMCurr);
      Console.print(";");
      Console.print (motorLeftPWMCurr);
      Console.print("; RightLEFT Current= ;");
      Console.print (motorRightSenseCurrent);
      Console.print(";");
      Console.println (motorLeftSenseCurrent);
    */

  }
  // ---------------------------------
  if (motorLeftSwapDir)  // swap pin polarity?
    setActuator(ACT_MOTOR_LEFT, -motorLeftPWMCurr);
  else
    setActuator(ACT_MOTOR_LEFT, motorLeftPWMCurr);
  if (motorRightSwapDir)   // swap pin polarity?
    setActuator(ACT_MOTOR_RIGHT, -motorRightPWMCurr);
  else
    setActuator(ACT_MOTOR_RIGHT, motorRightPWMCurr);
}




void Robot::OdoRampCompute() { //execute only one time when a new state execution
  //Compute the accel duration (very important for small distance)
  //Compute when you need to brake the 2 wheels to stop at the ODO
  //Compute the estimate duration of the state so can force next state if the mower is stuck
  stateStartOdometryLeft = odometryLeft;
  stateStartOdometryRight = odometryRight;
  lastStartOdometryRight = odometryRight;
  lastStartOdometryLeft = odometryLeft;
  straightLineTheta = 0;
  motorRightPID.reset();
  PwmRightSpeed = min(motorSpeedMaxPwm, max(-motorSpeedMaxPwm, map(motorRightSpeedRpmSet, -motorSpeedMaxRpm, motorSpeedMaxRpm, -motorSpeedMaxPwm, motorSpeedMaxPwm)));
  PwmLeftSpeed = min(motorSpeedMaxPwm, max(-motorSpeedMaxPwm, map(motorLeftSpeedRpmSet, -motorSpeedMaxRpm, motorSpeedMaxRpm, -motorSpeedMaxPwm, motorSpeedMaxPwm)));
  //try to find when we need to brake the wheel (depend of the distance)

  int  distToMoveLeft;
  int  distToMoveRight;
  distToMoveLeft = abs(stateStartOdometryLeft - stateEndOdometryLeft);
  distToMoveRight = abs(stateStartOdometryRight - stateEndOdometryRight);
  //left wheel
  if (distToMoveLeft >= odometryTicksPerRevolution)  {
    OdoStartBrakeLeft =  odometryTicksPerRevolution / 2; //si plus d'1 tour on freine dans la moitie du dernier tour
    SpeedOdoMaxLeft = PwmLeftSpeed; //valeur de vitesse max en fonction de la distance a parcourir
  }
  else {  // si moins d 1 tour
    if (UseAccelLeft && UseBrakeLeft) { //need 2 ramp
      OdoStartBrakeLeft = distToMoveLeft / 2; //on freine a la moitie de la distance a parcourir
      if (PwmLeftSpeed <= 0) SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
    }
    else
    { //need 1 ramp
      OdoStartBrakeLeft = distToMoveLeft ; //on freine sur toute la distance a parcourir
      if (PwmLeftSpeed <= 0) SpeedOdoMaxLeft = map(distToMoveLeft , odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxLeft = map(distToMoveLeft , odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
    }
  }

  //right wheel
  if (distToMoveRight >= odometryTicksPerRevolution) { //more than 1 rev
    OdoStartBrakeRight =  odometryTicksPerRevolution / 2;
    SpeedOdoMaxRight = PwmRightSpeed;
  }
  else {  //if less than 1 rev right wheel
    if (UseAccelRight && UseBrakeRight) {
      OdoStartBrakeRight = distToMoveRight / 2; //on freine a la moitie de la distance a parcourir
      if (PwmRightSpeed <= 0) SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, SpeedOdoMax);
    }
    else
    {
      OdoStartBrakeRight = distToMoveRight ; //on freine sur toute la distance a parcourir
      if (PwmRightSpeed <= 0) SpeedOdoMaxRight = map(distToMoveRight , odometryTicksPerRevolution / 2, 0, PwmRightSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      else SpeedOdoMaxRight = map(distToMoveRight , odometryTicksPerRevolution / 2, 0, PwmRightSpeed, SpeedOdoMax);
    }
  }


  //compute the approximative moving time in millis()
  //Need to compute in 2 times to avoid overflow  !!!!!

  movingTimeLeft = 1000 * distToMoveLeft / motorTickPerSecond ;
  movingTimeLeft = movingTimeLeft * motorSpeedMaxPwm / abs(SpeedOdoMaxLeft);
  movingTimeRight = 1000 * distToMoveRight / motorTickPerSecond ;
  movingTimeRight = movingTimeRight * motorSpeedMaxPwm / abs(SpeedOdoMaxRight);

  //for small mouvement need to reduce the accel duration
  if (movingTimeLeft >= motorOdoAccel) accelDurationLeft = motorOdoAccel;
  else   accelDurationLeft =  movingTimeLeft / 2;
  if (movingTimeRight >= motorOdoAccel) accelDurationRight = motorOdoAccel;
  else   accelDurationRight =  movingTimeRight / 2;
  if (statusCurr == TESTING) {  //avoid maxduration stop when use test Odo with Pfod
    MaxOdoStateDuration = 30000 + max(movingTimeRight, movingTimeLeft); //add 3 secondes to the max moving duration of the 2 wheels
  }
  else
  {
    MaxOdoStateDuration = 3000 + max(movingTimeRight, movingTimeLeft); //add 3 secondes to the max moving duration of the 2 wheels
  }
  //check to set the correct heading
  imuDriveHeading = imu.ypr.yaw / PI * 180; //normal mowing heading
  if (statusCurr == BACK_TO_STATION) {  //possible heading change
    imuDriveHeading = periFindDriveHeading / PI * 180;
  }
  if (statusCurr == REMOTE) {   //possible heading change
    imuDriveHeading = remoteDriveHeading / PI * 180;
  }

  /*
    Console.print(" **************** compute  at  ");
    Console.println(millis());
    Console.print(" UseAccelRight ");
    Console.print(UseAccelRight);
    Console.print(" UseBrakeRight ");
    Console.print(UseBrakeRight);
    Console.print(" UseAccelLeft ");
    Console.print(UseAccelLeft);
    Console.print(" UseBrakeLeft ");
    Console.print(UseBrakeLeft);
    Console.print(" distToMoveLeft ");
    Console.print(distToMoveLeft);
    Console.print(" movingTimeLeft ");
    Console.print(movingTimeLeft);
    Console.print("ms movingTimeRight ");
    Console.println(movingTimeRight);
    Console.print("accelDurationLeft ");
    Console.print(accelDurationLeft);
    Console.print("ms accelDurationRight ");
    Console.println(accelDurationRight);
    Console.print("SpeedOdoMaxLeft ");
    Console.print(SpeedOdoMaxLeft);
    Console.print("pwm SpeedOdoMaxRight ");
    Console.println(SpeedOdoMaxRight);
    Console.print("OdoStartBrakeLeft ");
    Console.print(OdoStartBrakeLeft);
    Console.print("Ticks OdoStartBrakeRight ");
    Console.println(OdoStartBrakeRight);
    Console.print("MaxOdoStateDuration ");
    Console.print(MaxOdoStateDuration);
    Console.println(" ms");
  */

}



void Robot::motorControlOdo() {

  // call to reach a ODO cible on left AND right wheel so they don't stop at the same time accel and slow are used to smooth the movement of the mower
  //Stop motor independently when the cible is reach
  //
  if (UseBrakeLeft && (motorLeftSpeedRpmSet >= 0) && (stateEndOdometryLeft - odometryLeft <= -10)) {//Forward left need -10 because when stop the ticks can move in+ or- so do not stop before
    PwmLeftSpeed = 0;
    motorLeftSpeedRpmSet = 0;
    motorLeftRpmCurr = 0;
  }
  if (UseBrakeRight && (motorRightSpeedRpmSet >= 0) && (stateEndOdometryRight - odometryRight <= -10)) {//right
    PwmRightSpeed = 0;
    motorRightSpeedRpmSet = 0;
    motorRightRpmCurr = 0;
  }
  //Reverse
  if (UseBrakeRight && (motorRightSpeedRpmSet <= 0) && (stateEndOdometryRight - odometryRight >= 10)) {//right
    PwmRightSpeed = 0;
    motorRightSpeedRpmSet = 0;
    motorRightRpmCurr = 0;
  }
  if (UseBrakeLeft && (motorLeftSpeedRpmSet <= 0) && (stateEndOdometryLeft - odometryLeft >= 10)) {//left
    PwmLeftSpeed = 0;
    motorLeftSpeedRpmSet = 0;
    motorLeftRpmCurr = 0;
  }
  if (millis() < nextTimeMotorOdoControl) return;
  nextTimeMotorOdoControl = millis() + 15;

  //LEFT WHEEL

  leftSpeed = PwmLeftSpeed ; //Set first to Normal speed and stay like this if not change  by accel or brake so limit the compute time
  if (motorLeftSpeedRpmSet > 0) { //forward left wheel --------------------------------------------------------------------------
    if ((UseAccelLeft) && (millis() - stateStartTime < accelDurationLeft)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationLeft, 0, 89);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp)); //convert degree to radians
    }
    if (UseBrakeLeft && (odometryLeft > stateEndOdometryLeft - (OdoStartBrakeLeft))) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryLeft - odometryLeft), OdoStartBrakeLeft, 0, 89, 10);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp));
    }
    if (leftSpeed > SpeedOdoMaxLeft) leftSpeed = SpeedOdoMaxLeft;
    if (leftSpeed < SpeedOdoMin) leftSpeed = SpeedOdoMin; //Minimum speed to be sure the mower is always moving before stop
  }

  if (motorLeftSpeedRpmSet < 0) { //reverse left wheel ----------------------------------------------------------------------------
    if ((UseAccelLeft) && (millis() - stateStartTime < accelDurationLeft)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationLeft, 0, 89);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp)); //convert degree to radians
    }
    if (UseBrakeLeft && (odometryLeft < stateEndOdometryLeft + OdoStartBrakeLeft)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryLeft - odometryLeft), OdoStartBrakeLeft, 0, 89, 10);
      leftSpeed = PwmLeftSpeed * sin(radians(angleCorresp));
    }
    if (leftSpeed < SpeedOdoMaxLeft) leftSpeed = SpeedOdoMaxLeft;
    if (abs(leftSpeed) < SpeedOdoMin) leftSpeed = -SpeedOdoMin;
  }

  //  RIGHT WHEEL
  rightSpeed = PwmRightSpeed ; //Normal speed

  if (motorRightSpeedRpmSet > 0) { //forward Right wheel -----------------------------------------------------------------------------
    // Console.print(" FR rotate ");
    if (UseAccelRight && (millis() - stateStartTime < accelDurationRight)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationRight, 0, 89);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (UseBrakeRight && (odometryRight > stateEndOdometryRight - OdoStartBrakeRight)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryRight - odometryRight), OdoStartBrakeRight, 0, 89, 10);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (rightSpeed > SpeedOdoMaxRight) rightSpeed = SpeedOdoMaxRight;
    if (rightSpeed < SpeedOdoMin) rightSpeed = SpeedOdoMin;
  }
  if (motorRightSpeedRpmSet < 0) { //reverse Right wheel ------------------------------------------------------------------------------
    if (UseAccelRight && (millis() - stateStartTime < accelDurationRight)) { //Accel mode for duration
      //Sinus accel
      angleCorresp = map(millis() - stateStartTime, 0, accelDurationRight, 0, 89);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }
    if (UseBrakeRight && (odometryRight < stateEndOdometryRight +  OdoStartBrakeRight)) { //Braking mode by odometry
      //Sinus brake
      angleCorresp = map(abs(stateEndOdometryRight - odometryRight), OdoStartBrakeRight, 0, 89, 10);
      rightSpeed = PwmRightSpeed * sin(radians(angleCorresp));
    }

    if (rightSpeed < SpeedOdoMaxRight) rightSpeed = SpeedOdoMaxRight;
    if (abs(rightSpeed) < SpeedOdoMin) rightSpeed = -SpeedOdoMin;

  }

  //DRIVE IN STRAIGHT LINE
  if (stateCurr == STATE_FORWARD_ODO || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_DRIVE1_TO_NEWAREA) || (stateCurr == STATE_DRIVE2_TO_NEWAREA))  { //PID compute to accel or brake the wheel to drive straight
    motorRightPID.Kp = motorLeftPID.Kp;
    motorRightPID.Ki = motorLeftPID.Ki;
    motorRightPID.Kd = motorLeftPID.Kd;
    // USE THE IMU
    if ((imuUse) && (mowPatternCurr == MOW_LANES) && (stateCurr == STATE_FORWARD_ODO)) { //if mow by lane need different cible
      YawActualDeg = imu.ypr.yaw / PI * 180;
      if (laneUseNr == 1) {   //from -45 to 45 deg
        yawCiblePos = yawSet1 ;
        // ImuPidCiblePos= yawSet1+360;
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane1RollRight;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane1RollRight);
        }
        else {
          yawCibleNeg = yawOppositeLane1RollLeft;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane1RollLeft);
        }

      }
      if (laneUseNr == 2) {   //from 45 to 135 deg
        yawCiblePos = yawSet2;
        //ImuPidCiblePos= yawSet2;
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane2RollRight;
          // ImuPidCibleNeg = abs(yawOppositeLane2RollRight);
        }
        else {
          yawCibleNeg = yawOppositeLane2RollLeft;
          // ImuPidCibleNeg = abs(yawOppositeLane2RollLeft);
        }
      }
      if (laneUseNr == 3) {    //from 135 to -135 or 225 deg
        yawCiblePos = yawSet3;
        // ImuPidCiblePos= imu.rotate360(yawSet3);
        if (rollDir == RIGHT) {
          yawCibleNeg = yawOppositeLane3RollRight;
          // ImuPidCibleNeg = imu.rotate360(yawOppositeLane3RollRight);
        }
        else {
          yawCibleNeg = yawOppositeLane3RollLeft;
          //  ImuPidCibleNeg = imu.rotate360(yawOppositeLane3RollLeft);
        }
      }

      if ((imu.ypr.yaw / PI * 180) > 0 ) imuDriveHeading = yawCiblePos;
      else imuDriveHeading = yawCibleNeg;
      imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
      imuDirPID.w = 0;
      imuDirPID.y_min = -motorSpeedMaxPwm / 2;
      imuDirPID.y_max = motorSpeedMaxPwm / 2;
      imuDirPID.max_output = motorSpeedMaxPwm / 2;
      imuDirPID.compute();

      if ((millis() - stateStartTime) < 1000) { // acceleration and more influence of PID vs speed
        rightSpeed =  rightSpeed - (66 - (millis() - stateStartTime) / 30);
        leftSpeed =  leftSpeed - (66 - (millis() - stateStartTime) / 30);
      }

      rightSpeed =  rightSpeed + imuDirPID.y / 2;
      leftSpeed =  leftSpeed - imuDirPID.y / 2;




      //--------------------------------------------------------------------------------------try to find the yaw with the odometry-------------------------------------
      if (((millis() - stateStartTime) > 2000) && (millis() >= nextTimePidCompute)) { //compute  the yaw with the odometry only after 2 sec
        float odoTheta;
        int odoDiffRightLeft;
        nextTimePidCompute = millis() + 800; //not to short to have enought ticks
        //stateStartOdometryLeft = stateStartOdometryLeft + ((odometryRight - stateStartOdometryRight) - (odometryLeft - stateStartOdometryLeft)); // very important change the odo to retrieve the line to avoid drift
        odoDiffRightLeft = ((odometryRight - lastStartOdometryRight) - (odometryLeft - lastStartOdometryLeft));
        lastStartOdometryRight = odometryRight;
        lastStartOdometryLeft = odometryLeft;
        odoTheta = asin( 2 * odoDiffRightLeft / odometryTicksPerCm  / odometryWheelBaseCm);
        straightLineTheta += odoTheta;
        /*
          Console.print(" odoDiffRightLeft  ");
          Console.print(odoDiffRightLeft);
          Console.print(" 2* odoDiffRightLeft /odometryTicksPerCm  / odometryWheelBaseCm ");
          Console.print(2* odoDiffRightLeft /odometryTicksPerCm  / odometryWheelBaseCm);
          Console.print(" odoTheta  ");
          Console.print(odoTheta,4);
          Console.print(" straightLineTheta  ");
          Console.println(straightLineTheta,4);
        */
      }
      //----------------------------------------------------------------------------------------------------------------------------------------------------------------

    }
    else
      //// NORMAL MOWING OR PERIFIND
    {
      if (imuUse) /// use the IMU for straight line
      {
        YawActualDeg = imu.ypr.yaw / PI * 180;
        // if(abs(YawActualDeg) >90) YawMedianDeg = imu.rotate360(YawActualDeg);
        // else YawMedianDeg= YawActualDeg+90;

        imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
        imuDirPID.w = 0;
        imuDirPID.y_min = -motorSpeedMaxPwm / 2;
        imuDirPID.y_max = motorSpeedMaxPwm / 2;
        imuDirPID.max_output = motorSpeedMaxPwm / 2;
        imuDirPID.compute();
        rightSpeed =  rightSpeed + imuDirPID.y / 2;
        leftSpeed =  leftSpeed - imuDirPID.y / 2;


      }
      else   /// use only the odometry  for straight line
      {
        if (millis() >= nextTimePidCompute) { //to go in straight line need to compute only each 200 milliseconde and add the dif to one wheel
          nextTimePidCompute = millis() + 200;
          //stateStartOdometryLeft = stateStartOdometryLeft + ((odometryRight - stateStartOdometryRight) - (odometryLeft - stateStartOdometryLeft)); // very important change the odo to retrieve the line to avoid drift
          motorRightPID.x = ((odometryRight - stateStartOdometryRight) - (odometryLeft - stateStartOdometryLeft));
          motorRightPID.w = 0;
          motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
          motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
          motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
          motorRightPID.compute();
          rightSpeed =  rightSpeed + motorRightPID.y ;
          leftSpeed =  leftSpeed - motorRightPID.y ;
        }
      }
    }

    //bber200
    rightSpeed = rightSpeed * sonarSpeedCoeff;
    leftSpeed = leftSpeed * sonarSpeedCoeff;


    if (rightSpeed > 255) rightSpeed = 255;
    if (leftSpeed > 255) leftSpeed = 255;
    if (rightSpeed < 0) rightSpeed = 0;
    if (leftSpeed < 0) leftSpeed = 0;

  }

  if (stateCurr != STATE_OFF) {
    /*
        Console.print(millis());
        Console.print(" Moving Average Dist= ");
        Console.print(currDistToDrive);
        Console.print(" ODO **** Lspeed= ");
        Console.print(leftSpeed);
        Console.print(" ODO Start/Actual/End ");
        Console.print(stateStartOdometryLeft);
        Console.print("/");
        Console.print(odometryLeft);
        Console.print("/");
        Console.print(stateEndOdometryLeft);
        Console.print(" ************************* Rspeed= ");
        Console.print(rightSpeed);
        Console.print(" ODO Start/Actual/End ");
        Console.print(stateStartOdometryRight);
        Console.print("/");
        Console.print(odometryRight);
        Console.print("/");
        Console.print(stateEndOdometryRight);
        Console.print(" PID reel ");
        Console.print(motorRightPID.x);
        Console.print(" PID resultat du calcul ");
        Console.println(motorRightPID.y);
        Console.print("IMU ***** Line use ");
        Console.print(laneUseNr);
        Console.print(" imuDriveHeading ");
        Console.print(imuDriveHeading);
        Console.print(" YawMedianDeg ");
        Console.print(YawMedianDeg);
        Console.print(" YawActualDeg ");
        Console.print(YawActualDeg);
        Console.print(" correctRight ");
        Console.print(correctRight);
        Console.print(" correctLeft ");
        Console.print(correctLeft);
        Console.print(" PID reel ");
        Console.print(imuDirPID.x);
        Console.print(" PID resultat du calcul ");
        Console.println(imuDirPID.y);
        Console.print(" imu.ypr.yaw ");
        Console.println(imu.ypr.yaw);
    */
  }


  setMotorPWM(leftSpeed, rightSpeed, false );
}




// PID controller: track perimeter
void Robot::motorControlPerimeter() {

  if (millis() < nextTimeMotorPerimeterControl) return;
  nextTimeMotorPerimeterControl = millis() + 15; //bb read the perimeter each 15 ms
  //never stop the PID compute while turning for the new transition
  //use the PerimeterMag as cible to smooth the tracking
  //Value reference perimeterMagMaxValue , maybe need to be calculate in mower setting up procedure

  perimeterPID.x = 5 * (double(perimeterMag) / perimeterMagMaxValue);

  if (perimeterInside)  perimeterPID.w = -0.5;
  else     perimeterPID.w = 0.5;

  perimeterPID.y_min = -ActualSpeedPeriPWM ;
  perimeterPID.y_max = ActualSpeedPeriPWM ;
  perimeterPID.max_output = ActualSpeedPeriPWM ;
  perimeterPID.compute();

  if ((millis() > stateStartTime + 10000) && (millis() > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut)) {
    // robot is wheel-spinning while tracking => roll to get ground again

    if (trakBlockInnerWheel == 0) {
      if (perimeterInside) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = -ActualSpeedPeriPWM / 2;
      }
      else {
        rightSpeedperi = -ActualSpeedPeriPWM / 2;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (trakBlockInnerWheel == 1) {
      if (perimeterInside) {
        rightSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  + perimeterPID.y));
        leftSpeedperi = 0;
      }
      else {
        rightSpeedperi = 0;
        leftSpeedperi = max(-ActualSpeedPeriPWM, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 - perimeterPID.y));
      }
    }
    if (consoleMode == CONSOLE_TRACKING) {
      Console.print("SEARCH;");
      Console.print(millis());
      Console.print(";");
      Console.print (perimeterMag);
      Console.print(";");
      Console.print(perimeterInside);
      Console.print(";");
      Console.print (perimeterPID.x);
      Console.print(";");
      Console.print(perimeterPID.y);
      Console.print(";");
      Console.print (leftSpeedperi);
      Console.print(";");
      Console.print (rightSpeedperi);
      Console.print(";");
      Console.println(perimeterLastTransitionTime);
    }
    setMotorPWM( leftSpeedperi, rightSpeedperi, false);

    lastTimeForgetWire = millis();

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut) {
      if (perimeterInside) {
        Console.println("Tracking Fail and we are inside, So start to find again the perimeter");
        setNextState(STATE_PERI_FIND, 0);
      }
      else
      {
        Console.println("Tracking Fail and we are outside, So start to roll to find again the perimeter");
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);
      }

    }
    return;
  }


  if ((millis() - lastTimeForgetWire ) < trackingPerimeterTransitionTimeOut) {
    //PeriCoeffAccel move gently from 3 to 1 and so perimeterPID.y/PeriCoeffAccel increase during 3 secondes
    PeriCoeffAccel = (3000.00 - (millis() - lastTimeForgetWire)) / 1000.00 ;
    if (PeriCoeffAccel < 1.00) PeriCoeffAccel = 1.00;
    rightSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 +  perimeterPID.y / PeriCoeffAccel));
    leftSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5 -  perimeterPID.y / PeriCoeffAccel));
    //bber30 we are in sonartrigger ,so maybe near station , so avoid 1 wheel reverse because station check is forward
    if (ActualSpeedPeriPWM != MaxSpeedperiPwm)
    {
      if (rightSpeedperi < 0) rightSpeedperi = 0;
      if (leftSpeedperi < 0) leftSpeedperi = 0;
    }

    if (consoleMode == CONSOLE_TRACKING) {
      Console.print("SLOW;");
      Console.print(millis());
      Console.print(";");
      Console.print (perimeterMag);
      Console.print(";");
      Console.print(perimeterInside);
      Console.print(";");
      Console.print (perimeterPID.x);
      Console.print(";");
      Console.print(perimeterPID.y);
      Console.print(";");
      Console.print (leftSpeedperi);
      Console.print(";");
      Console.print (rightSpeedperi);
      Console.print(";");
      Console.println(perimeterLastTransitionTime);
    }
  }
  else
  {
    rightSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5   + perimeterPID.y));
    leftSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  - perimeterPID.y));

    if (consoleMode == CONSOLE_TRACKING) {
      Console.print("FAST;");
      Console.print(millis());
      Console.print(";");
      Console.print (perimeterMag);
      Console.print(";");
      Console.print(perimeterInside);
      Console.print(";");
      Console.print (perimeterPID.x);
      Console.print(";");
      Console.print(perimeterPID.y);
      Console.print(";");
      Console.print (leftSpeedperi);
      Console.print(";");
      Console.print (rightSpeedperi);
      Console.print(";");
      Console.println(perimeterLastTransitionTime);
    }
  }

  //bb2
  if ((millis() - stateStartTime ) < 2000) { //at the start of the tracking accelerate slowly during 2 secondes
    leftSpeedperi = leftSpeedperi - (66 - (millis() - stateStartTime) / 30);
    rightSpeedperi = rightSpeedperi - (66 - (millis() - stateStartTime) / 30);
  }

  setMotorPWM( leftSpeedperi, rightSpeedperi, false);

  if (abs(perimeterMag) < perimeterMagMaxValue / 4) { //250 can be replace by timedOutIfBelowSmag to be tested
    perimeterLastTransitionTime = millis(); //initialise perimeterLastTransitionTime if perfect sthraith line

  }
}



// check for odometry sensor faults
void Robot::checkOdometryFaults() {

  boolean leftErr = false;
  boolean rightErr = false;
  if ((stateCurr == STATE_FORWARD) &&  (millis() - stateStartTime > 8000) ) {
    // just check if odometry sensors may not be working at all
    if ( (motorLeftPWMCurr > 100) && (abs(motorLeftRpmCurr) < 1)  )  leftErr = true;
    if ( (motorRightPWMCurr > 100) && (abs(motorRightRpmCurr) < 1)  ) rightErr = true;
  }
  if ((stateCurr == STATE_ROLL) &&  (millis() - stateStartTime > 1000) ) {
    // just check if odometry sensors may be turning in the wrong direction
    if ( ((motorLeftPWMCurr > 100) && (motorLeftRpmCurr < -3)) || ((motorLeftPWMCurr < -100) && (motorLeftRpmCurr > 3)) ) leftErr = true;
    if ( ((motorRightPWMCurr > 100) && (motorRightRpmCurr < -3)) || ((motorRightPWMCurr < -100) && (motorRightRpmCurr > 3)) ) rightErr = true;
  }
  if (leftErr) {
    Console.print("Left odometry error: PWM=");
    Console.print(motorLeftPWMCurr);
    Console.print("\tRPM=");
    Console.println(motorLeftRpmCurr);
    addErrorCounter(ERR_ODOMETRY_LEFT);
    setNextState(STATE_ERROR, 0);
  }
  if (rightErr) {
    Console.print("Right odometry error: PWM=");
    Console.print(motorRightPWMCurr);
    Console.print("\tRPM=");
    Console.println(motorRightRpmCurr);
    addErrorCounter(ERR_ODOMETRY_RIGHT);
    setNextState(STATE_ERROR, 0);
  }
}

void Robot::motorControl() {
  if (millis() < nextTimeMotorControl) return;
  nextTimeMotorControl = millis() + 200;  // 10 at the original
  static unsigned long nextMotorControlOutputTime = 0;

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen hÃ¶chstes Drehmoment fÃ¼r die Solldrehzahl zu gewÃ¤hrleisten
  motorLeftPID.w = motorLeftSpeedRpmSet;               // SOLL
  motorRightPID.w = motorRightSpeedRpmSet;             // SOLL

  float RLdiff = motorLeftRpmCurr - motorRightRpmCurr;

  if (motorLeftSpeedRpmSet == motorRightSpeedRpmSet) {
    // line motion
    if (odoLeftRightCorrection) {
      motorLeftPID.w = motorLeftSpeedRpmSet - RLdiff / 2;
      motorRightPID.w = motorRightSpeedRpmSet + RLdiff / 2;
    }
  }
  motorLeftPID.x = motorLeftRpmCurr;                 // IST
  // if (millis() < stateStartTime + motorZeroSettleTime) motorLeftPID.w = 0; // get zero speed first after state change
  if ((stateCurr == STATE_OFF)) motorLeftPID.w = 0; // to be sure the motor stop when OFF
  motorLeftPID.y_min = -motorSpeedMaxPwm;        // Regel-MIN
  motorLeftPID.y_max = motorSpeedMaxPwm;     // Regel-MAX
  motorLeftPID.max_output = motorSpeedMaxPwm;    // Begrenzung
  motorLeftPID.compute();
  int leftSpeed = motorLeftPWMCurr + motorLeftPID.y;
  if (motorLeftSpeedRpmSet >= 0) leftSpeed = min( max(0, leftSpeed), motorSpeedMaxPwm);
  if (motorLeftSpeedRpmSet < 0) leftSpeed = max(-motorSpeedMaxPwm, min(0, leftSpeed));

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen hÃ¶chstes Drehmoment fÃ¼r die Solldrehzahl zu gewÃ¤hrleisten
  motorRightPID.Kp = motorLeftPID.Kp;
  motorRightPID.Ki = motorLeftPID.Ki;
  motorRightPID.Kd = motorLeftPID.Kd;
  motorRightPID.x = motorRightRpmCurr;               // IST
  // if (millis() < stateStartTime + motorZeroSettleTime) motorRightPID.w = 0; // get zero speed first after state change
  if ((stateCurr == STATE_OFF)) motorRightPID.w = 0; // to be sure the motor stop when OFF
  motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
  motorRightPID.y_max = motorSpeedMaxPwm;        // Regel-MAX
  motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
  motorRightPID.compute();
  int rightSpeed = motorRightPWMCurr + motorRightPID.y;
  if (motorRightSpeedRpmSet >= 0) rightSpeed = min( max(0, rightSpeed), motorSpeedMaxPwm);
  if (motorRightSpeedRpmSet < 0) rightSpeed = max(-motorSpeedMaxPwm, min(0, rightSpeed));

  if ( (abs(motorLeftPID.x) < 2) && (abs(motorLeftPID.w) < 0.1) ) leftSpeed = 0; // ensures PWM is really zero
  if ( (abs(motorRightPID.x)  < 2) && (abs(motorRightPID.w) < 0.1) ) rightSpeed = 0; // ensures PWM is really zero

  /*  if (millis() >= nextMotorControlOutputTime){
      nextMotorControlOutputTime = millis() + 1000;
      Console.print("PID x=");
      Console.print(motorLeftPID.x);
      Console.print("\tPID w=");
      Console.print(motorLeftPID.w);
      Console.print("\tPID y=");
      Console.print(motorLeftPID.y);
      Console.print("\tPWM=");
      Console.println(leftSpeed);
    }
  */

  setMotorPWM( leftSpeed, rightSpeed, false );


}


void Robot::motorMowControl() {
  if (millis() < nextTimeMotorMowControl) return;
  nextTimeMotorMowControl = millis() + 100;
  if (motorMowForceOff) motorMowEnable = false;
  //Auto adjust the motor speed according to cutting power (The goal is On high grass the motor rotate faster)
  //A runningmedian process is used to check each seconde the power value of mow motor
  //if power is low the speed is reduce to have a longer mowing duration and less noise.
  if (motorMowEnable) {
    motorMowPowerMedian.add(motorMowPower);
    if (motorMowPowerMedian.getCount() > 10) { //check each 1 secondes
      int prevcoeff =  motorMowPwmCoeff;
      motorMowPwmCoeff = int((100 * motorMowPowerMedian.getAverage(4)) / (0.5 * motorMowPowerMax));
      if (motorMowPwmCoeff < prevcoeff) {
        //filter on speed reduce to keep the mow speed high for longuer duration
        motorMowPwmCoeff = int((0.1) * motorMowPwmCoeff + (0.9) * prevcoeff);// use only 10% of the new value
      }
      if ((statusCurr == WIRE_MOWING) || (statusCurr == SPIRALE_MOWING)) motorMowPwmCoeff = 100;
      if (motorMowPwmCoeff > 100) motorMowPwmCoeff = 100;
      if (motorMowEnable) {
        motorMowSpeedPWMSet = motorMowSpeedMinPwm + ((double)(motorMowSpeedMaxPwm - motorMowSpeedMinPwm)) * (((double)motorMowPwmCoeff) / 100.0);
      }
      if (motorMowSpeedPWMSet < motorMowSpeedMinPwm) motorMowSpeedPWMSet = motorMowSpeedMinPwm;
      if (motorMowSpeedPWMSet > motorMowSpeedMaxPwm) motorMowSpeedPWMSet = motorMowSpeedMaxPwm;
      //max speed on wire and spirale
      motorMowPowerMedian.clear();
    }
  }
  else
  {
    motorMowSpeedPWMSet = 0;
  }
  if (stateCurr == STATE_ERROR) {
    setMotorMowPWM(0, false); //stop immediatly on error (tilt etc....)
  }
  else
  {
    setMotorMowPWM(motorMowSpeedPWMSet, true);
  }
}

void Robot::resetIdleTime() {
  if (idleTimeSec == BATTERY_SW_OFF) { // battery switched off?
    Console.println(F("BATTERY switching ON again"));
    setActuator(ACT_BATTERY_SW, 1);  // switch on battery again (if connected via USB)
  }
  idleTimeSec = 0;
}

void Robot::setBeeper(int totalDuration, byte OnDuration, byte OffDuration, byte frequenceOn, byte frequenceOff ) { // Set the variable for the beeper
  endBeepTime = millis() + totalDuration * 10;
  beepOnDuration = OnDuration * 10;
  beepOffDuration = OffDuration * 10;
  beepfrequenceOn = frequenceOn * 10;
  beepfrequenceOff = frequenceOff * 10;
}


void Robot::beeper() {  //beeper avoid to use the delay() fonction to not freeze the DUE
  if (millis() < nextTimeBeeper) return;
  nextTimeBeeper = millis() + 50; //maybe test with 100 if loops is too low
  if (((beepOnDuration == 0) && (beepOffDuration == 0)) || (millis() > endBeepTime)) {
    Buzzer.noTone();
    beepOnOFFDuration = 0;
  }
  else {
    if (beepOnOFFDuration == 0) beepOnOFFDuration = millis();
    if (millis() >= beepOnOFFDuration )
    {
      if (beepState) beepOnOFFDuration = beepOnOFFDuration + beepOnDuration;
      else beepOnOFFDuration = beepOnOFFDuration + beepOffDuration;
      beepState = !beepState;
      if (beepState) {
        Buzzer.tone(beepfrequenceOn);
      }
      else {
        Buzzer.tone(beepfrequenceOff);
      }
    }
  }
}


// set user-defined switches
void Robot::setUserSwitches() {
  setActuator(ACT_USER_SW1, userSwitch1);
  setActuator(ACT_USER_SW2, userSwitch2);
  setActuator(ACT_USER_SW3, userSwitch3);
}



void Robot::setup()  {

  // i don't understand why the mower start before the robot setup ????????????????????????????????????????????

  Console.print(" --> ++++++++++++++++++++++++++++++++++* Start Robot Setup at ");
  Console.print(millis());
  Console.println(" --> +++++++++++++++++++++++++++");


  ADCMan.begin();
  PinMan.begin();
  if (RaspberryPIUse) MyRpi.init();





  //setDefaultTime();
  //init of timer for factory setting
  for (int i = 0; i < MAX_TIMERS; i++) {
    timer[i].active = false;
    timer[i].startTime.hour = 0;
    timer[i].startTime.minute = 0;
    timer[i].stopTime.hour = 0;
    timer[i].stopTime.minute = 0;
    timer[i].daysOfWeek = 0;
    timer[i].startDistance = 0;
    timer[i].startArea = 1;
    timer[i].startMowPattern = 0;
    timer[i].startNrLane = 0;
    timer[i].startRollDir = 0;
    timer[i].startLaneMaxlengh = 0;
    timer[i].rfidBeacon = 0;
  }
  ActualRunningTimer = 99;
  setMotorPWM(0, 0, false);
  loadSaveErrorCounters(true);
  loadUserSettings();
  if (!statsOverride) loadSaveRobotStats(true);
  else loadSaveRobotStats(false);
  setUserSwitches();


  if (imuUse) imu.begin();

  if (perimeterUse) {
    perimeter.changeArea(1);
    perimeter.begin(pinPerimeterLeft, pinPerimeterRight);
  }
  //if (perimeterUse) perimeter.begin(pinPerimeterCenter, pinPerimeterRight);

  if (!buttonUse) {
    // robot has no ON/OFF button => start immediately
    setNextState(STATE_FORWARD_ODO, 0);
  }

  dht.begin();
  nextTimeReadDHT22 = millis() + 15000; //read only after all the setting of the mower are OK

  stateStartTime = millis();
  setBeeper(100, 50, 50, 200, 200 );//beep for 3 sec
  gps.init();
  Console.println(F("START"));
  Console.print(F("Ardumower "));
  Console.println(VER);
#ifdef USE_DEVELOPER_TEST
  Console.println("Warning: USE_DEVELOPER_TEST activated");
#endif
  Console.print(F("Config: "));
  Console.println(name);
  Console.println(F("press..."));
  Console.println(F("  d for menu"));
  Console.println(F("  v to change console output (sensor counters, values, perimeter etc.)"));
  Console.println(consoleModeNames[consoleMode]);
  Console.println ();
  // Console.print ("        Free memory is :   ");
  // Console.println (freeMemory ());

  // watchdog enable at the end of the setup
  if (Enable_DueWatchdog) {
    Console.println ("Watchdog is enable and set to 3 secondes");
    watchdogEnable(3000);// Watchdog trigger after  3 sec if not reseted.

  }
  else
  {
    Console.println ("Watchdog is disable");
  }

  nextTimeInfo = millis();

}


void Robot::printRemote() {
  Console.print(F("RC "));
  Console.print(remoteSwitch);
  Console.print(",");
  Console.print(remoteSteer);
  Console.print(",");
  Console.print(remoteSpeed);
  Console.print(",");
  Console.println(remoteMow);
}

void Robot::printOdometry() {
  Console.print(F("ODO,"));
  Console.print(odometryX);
  Console.print(",");
  Console.println(odometryY);
  Console.print(F("ODO,"));
  Console.print(odometryX);
  Console.print(",");
  Console.println(odometryY);
}


void Robot::receivePiPfodCommand (String RpiCmd, float v1, float v2, float v3) {
  rc.processPI(RpiCmd, v1, v2, v3);
}






void Robot::printInfo(Stream & s) {



  if ((consoleMode == CONSOLE_OFF) || (consoleMode == CONSOLE_TRACKING)) {




  } else {
    Streamprint(s, "t%6u ", (millis() - stateStartTime) / 1000);
    Streamprint(s, "Loops%7u ", loopsPerSec);
    //Streamprint(s, "r%4u ", freeRam());
    Streamprint(s, "v%1d ", consoleMode);

    Streamprint(s, "%4s ", stateNames[stateCurr]);

    if (consoleMode == CONSOLE_PERIMETER) {
      Streamprint(s, "sig min %4d max %4d avg %4d mag %5d qty %3d",
                  (int)perimeter.getSignalMin(0), (int)perimeter.getSignalMax(0), (int)perimeter.getSignalAvg(0),
                  perimeterMag, (int)(perimeter.getFilterQuality(0) * 100.0));
      Streamprint(s, "  in %2d  cnt %4d  on %1d\r\n",
                  (int)perimeterInside, perimeterCounter, (int)(!perimeter.signalTimedOut(0)) );
    } else {
      Streamprint(s, "odo %4d %4d ", (int)odometryLeft, (int)odometryRight);
      Streamprint(s, "spd %4d %4d %4d ", (int)motorLeftSpeedRpmSet, (int)motorRightSpeedRpmSet, (int)motorMowPwmCoeff);
      if (consoleMode == CONSOLE_SENSOR_VALUES) {
        // sensor values
        Streamprint(s, "sen %4d %4d %4d ", (int)motorLeftPower, (int)motorRightPower, (int)motorMowPower);
        Streamprint(s, "bum %4d %4d ", bumperLeft, bumperRight);
        Streamprint(s, "dro %4d %4d ", dropLeft, dropRight);                                                                                      // Dropsensor - Absturzsensor
        Streamprint(s, "son %4d %4d %4d ", sonarDistLeft, sonarDistCenter, sonarDistRight);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));
        if (perimeterUse) Streamprint(s, "per %3d ", (int)perimeterInside);
        if (lawnSensorUse) Streamprint(s, "lawn %3d %3d ", (int)lawnSensorFront, (int)lawnSensorBack);
      } else {
        // sensor counters
        Streamprint(s, "sen %4d %4d %4d ", motorLeftSenseCounter, motorRightSenseCounter, motorMowSenseCounter);
        Streamprint(s, "bum %4d %4d ", bumperLeftCounter, bumperRightCounter);
        Streamprint(s, "dro %4d %4d ", dropLeftCounter, dropRightCounter);                                                                      // Dropsensor - Absturzsensor
        //Streamprint(s, "son %3d ", sonarDistCounter);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));
        //Streamprint(s, "per %3d ", perimeterLeft);
        if (perimeterUse) Streamprint(s, "per %3d ", perimeterCounter);
        if (lawnSensorUse) Streamprint(s, "lawn %3d ", lawnSensorCounter);
        //if (gpsUse) Streamprint(s, "gps %2d ", (int)gps.satellites());
      }
      Streamprint(s, "bat %2d.%01d ", (int)batVoltage, (int)((batVoltage * 10) - ((int)batVoltage * 10)) );
      Streamprint(s, "chg %2d.%01d %2d.%01d ",
                  (int)chgVoltage, (int)((chgVoltage * 10) - ((int)chgVoltage * 10)),
                  (int)chgCurrent, (int)((abs(chgCurrent) * 10) - ((int)abs(chgCurrent) * 10))
                 );
      //Streamprint(s, "imu%3d ", imu.getCallCounter());
      //   Streamprint(s, "adc%3d ", ADCMan.getCapturedChannels());
      Streamprint(s, "%s\r\n", name.c_str());
    }
  }
}

void Robot::printMenu() {
  Console.println();
  Console.println(F(" MAIN MENU:"));
  Console.println(F("1=test motors"));
  Console.println(F("To test odometry --> use Arduremote"));
  Console.println(F("3=communications menu"));
  Console.println(F("5=Deactivate and Delete GYRO calibration : To calibrate GYRO --> use Arduremote Do not move IMU during the Calib"));
  Console.println(F("6=Deactivate and Delete Compass calibration : To calibrate Compass --> use Arduremote start/stop"));
  Console.println(F("9=save user settings"));
  Console.println(F("l=load factory settings: Do not save setting before restart the mower"));
  Console.println(F("r=delete robot stats"));
  Console.println(F("x=read settings"));
  Console.println(F("e=delete all errors"));
  Console.println(F("0=exit"));
  Console.println();
}

void Robot::delayWithWatchdog(int ms) {
  unsigned long endtime = millis() + ms;
  while (millis() < endtime) {
    delay(500);
    watchdogReset();
  }
}

void Robot::delayInfo(int ms) {
  unsigned long endtime = millis() + ms;
  while (millis() < endtime) {
    readSensors();
    printInfo(Console);
    watchdogReset();
    delay(1000);
    watchdogReset();
  }
}
/*odometryTicksPerRevolution = 720;   // encoder ticks per one full resolution
    odometryTicksPerCm = 9.96;  // encoder ticks per cm
    odometryWheelBaseCm = 42;
*/

void Robot::testMotors() {
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  Console.println(F("testing left motor (forward) half speed..."));
  delay(100);
  motorLeftPWMCurr = motorSpeedMaxPwm / 2; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  Console.println(F("testing left motor (reverse) full speed..."));
  delay(100);
  motorLeftPWMCurr = -motorSpeedMaxPwm; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  Console.println(F("testing right motor (forward) half speed..."));
  delay(100);
  motorLeftPWMCurr = 0; motorRightPWMCurr = motorSpeedMaxPwm / 2;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  Console.println(F("testing right motor (reverse) full speed..."));
  delay(100);
  motorLeftPWMCurr = 0; motorRightPWMCurr = -motorSpeedMaxPwm;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
}

void Robot::menu() {
  char ch;
  printMenu();

  while (true) {
    watchdogReset();
    resetIdleTime();
    // imu.update();
    if ((!RaspberryPIUse) && (Console.available() > 0)) {  //do not read console is raspberry connected
      ch = (char)Console.read();
      switch (ch) {
        case '0':
          nextTimeInfo = millis();
          return;
        case '1':
          testMotors();
          printMenu();
          break;
        
        case '5':
          imu.deleteAccelGyroCalib();
          imuUse=false;
          printMenu();
          break;
        case '6':
          imu.deleteCompassCalib();
          CompassUse=false;
          printMenu();
          break;
       
        case '9':
          saveUserSettings();
          printMenu();
          break;
        case 'l':
          printSettingSerial();
          deleteUserSettings();
          printMenu();
          break;
        case 'r':
          printSettingSerial();
          deleteRobotStats();
          printMenu();
          break;
        case 'x':
          printSettingSerial();
          Console.println(F("DONE"));
          printMenu();
          break;
        case 'e':
          resetErrorCounters();
          setNextState(STATE_OFF, 0);
          Console.println(F("ALL ERRORS ARE DELETED"));
          printMenu();
          break;
      }
    }
    delay(10);
  }
}


/*
void Robot::commsMenuBT() {
  while (true) {
    Console.println();
    Console.println(F("COMMUNICATIONS MENU  == Bluetooth =="));
    Console.println(F(" 1=Select other communication method"));
    Console.println(F(" 2=setup BT module config (quick baudscan (recommended))"));
    Console.println(F(" 3=setup BT module config (extensive baudscan)"));
    Console.println(F(" 0=Main Menu"));
    Console.println();

    delay(100);
    purgeConsole();

    switch (waitCharConsole()) {
      case '0':
        return;
      case '1':
        commsMenuSelect();
        return;
      case '2':
        configureBluetooth(true);
        break;
      case '3':
        configureBluetooth(false);
        break;
    }
  }
}



void Robot::commsMenuSelect(void) {
  bluetoothUse = 0;
  esp8266Use = 0;

  while (true) {
    Console.println(F("Select communication method"));
    Console.println(F(" 1=Bluetooth"));
    Console.println(F(" 2=Wifi"));

    delay(100);
    purgeConsole();

    switch (waitCharConsole()) {
      case '1': bluetoothUse = 1; return;
      case '2': esp8266Use = 1; return;
    }
  }
}
*/
void Robot::readSerial() {


  if ((!RaspberryPIUse) && (Console.available() > 0)) {  //do not read console if raspberry connected
    char ch = (char)Console.read();
    //resetIdleTime();
    switch (ch) {
      case '0':
        // press '0' for OFF
        setNextState(STATE_OFF, 0);
        break;
      case '1':
        // press '1' for Automode
        //motorMowEnable = true;

        setNextState(STATE_ACCEL_FRWRD, 0);
        break;
      case 'd':
        menu(); // menu
        break;

      case 'h':

        setNextState(STATE_PERI_FIND, 0); // press 'h' to drive home
        break;
      case 'l':
        bumperLeft = true; // press 'l' to simulate left bumper
        bumperLeftCounter++;
        break;
      case 'q':
        yawCiblePos = 90;
        setNextState(STATE_ROLL_TO_FIND_YAW, 0); // press 'h' to drive home
        break;
      case 'r':
        setBeeper(400, 50, 50, 200, 0 );//error
        break;
      case 's':
        //imu.calibComStartStop();
        break;
      case 't':
        setNextState(STATE_PERI_TRACK, 0); // press 't' to track perimeter
        break;
      case 'u':
        setNextState(STATE_ACCEL_FRWRD, RIGHT);
        break;
      case 'v':
        //bb
        consoleMode = (consoleMode + 1) % 5;
        Console.println(consoleModeNames[consoleMode]);
        break;


    }
  }

}

void Robot::checkButton() {
  if ( (!buttonUse) || (millis() < nextTimeButtonCheck) ) return;
  nextTimeButtonCheck = millis() + 100;
  boolean buttonPressed = (readSensor(SEN_BUTTON) == LOW);
  if ( ((!buttonPressed) && (buttonCounter > 0)) || ((buttonPressed) && (millis() >= nextTimeButton)) )
  {
    nextTimeButton = millis() + 1000;
    if (buttonPressed) {
      //Console.print(F("Button Pressed counter : "));
      //Console.println(buttonCounter);
      // ON/OFF button pressed
      setBeeper(50, 50, 0, 200, 0 );//
      buttonCounter++;
      if (buttonCounter >= 3) buttonCounter = 3;
      //resetIdleTime();
    }
    else
    {
      // ON/OFF button released
      //Console.print(F("Button Release counter : "));
      //Console.println(buttonCounter);
      if ((statusCurr == NORMAL_MOWING) || (statusCurr == SPIRALE_MOWING) || (stateCurr == STATE_ERROR) || (statusCurr == WIRE_MOWING) || (statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START)) {
        Console.println(F("ButtonPressed Stop Mowing and Reset Error"));
        motorMowEnable = false;
        buttonCounter = 0;
        setNextState(STATE_OFF, 0);
        return;
      }
      if  ((stateCurr == STATE_OFF) || (stateCurr == STATE_STATION)) {
        if (buttonCounter == 1) {
          // start normal with mowing in lanes
          motorMowEnable = true;
          statusCurr = NORMAL_MOWING;
          mowPatternCurr = MOW_LANES;
          buttonCounter = 0;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
          setNextState(STATE_ACCEL_FRWRD, 0);
          return;
        }
        else if (buttonCounter == 2) {
          // start normal with random mowing
          motorMowEnable = true;
          statusCurr = NORMAL_MOWING;
          mowPatternCurr = MOW_RANDOM;
          buttonCounter = 0;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
          setNextState(STATE_ACCEL_FRWRD, 0);
          return;
        }
        else if (buttonCounter == 3) {
          if (stateCurr == STATE_STATION) return;
          //go to station
          periFindDriveHeading = scalePI(imu.ypr.yaw);
          areaToGo = 1;
          whereToStart = 99999;
          nextTimeTimer = millis() + 3600000; //avoid the mower start again if timer activate.
          statusCurr = BACK_TO_STATION;
          buttonCounter = 0;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
          setNextState(STATE_PERI_FIND, 0);
          return;
        }
      }
      buttonCounter = 0;
    }

  }
}
void Robot::newTagFind() {
  if (millis() >= nextTimeSendTagToPi) {
    nextTimeSendTagToPi = millis() + 10000;
    Console.print("Find a tag : ");
    Console.println(rfidTagFind);
    if (rfidUse) {
      if (RaspberryPIUse) MyRpi.SendRfidToPi();
    }
  }



}

void Robot::readSensors() {
  //NOTE: this function should only put sensors value into variables - it should NOT change any state!
  //The ADC return is now 12 bits so 0 to 4096
  if (millis() >= nextTimeMotorSense) {
    nextTimeMotorSense = millis() +  50;
    double accel = 0.05;
    motorRightSenseADC = readSensor(SEN_MOTOR_RIGHT) ; //return the ADC value,for MC33926 0.525V/1A so ADC=651/1Amp
    motorLeftSenseADC = readSensor(SEN_MOTOR_LEFT) ;
    motorMowSenseADC = readSensor(SEN_MOTOR_MOW) ;
    //  double batvolt = batFactor*readSensor(SEN_BAT_VOLTAGE)*3.3/4096 ;
    // motorRightSenseADC =651 for 1000ma so motorSenseRightScale=1.536
    motorRightSenseCurrent = motorRightSenseCurrent * (1.0 - accel) + ((double)motorRightSenseADC) * motorSenseRightScale * accel;
    motorLeftSenseCurrent = motorLeftSenseCurrent * (1.0 - accel) + ((double)motorLeftSenseADC) * motorSenseLeftScale * accel;
    motorMowSenseCurrent = motorMowSenseCurrent * (1.0 - accel) + ((double)motorMowSenseADC) * motorMowSenseScale * accel;

    if (batVoltage > 8) {
      motorRightPower = motorRightSenseCurrent * batVoltage / 1000;  // conversion to power in Watt
      motorLeftPower  = motorLeftSenseCurrent  * batVoltage / 1000;
      motorMowPower   = motorMowSenseCurrent   * batVoltage / 1000;
    }
    else {
      motorRightPower = motorRightSenseCurrent * batFull / 1000;  // conversion to power in Watt in absence of battery voltage measurement
      motorLeftPower  = motorLeftSenseCurrent  * batFull / 1000;
      motorMowPower   = motorMowSenseCurrent   * batFull / 1000;
    }
    /*
        if ((millis() - lastMotorMowRpmTime) >= 500) {
          motorMowRpmCurr = readSensor(SEN_MOTOR_MOW_RPM);
          if ((motorMowRpmCurr == 0) && (motorMowRpmCounter != 0)) {
            // rpm may be updated via interrupt
            motorMowRpmCurr = (int) ((((double)motorMowRpmCounter) / ((double)(millis() - lastMotorMowRpmTime))) * 60000.0);
            motorMowRpmCounter = 0;
          }
          lastMotorMowRpmTime = millis();

        }
    */
  }


  if ((stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING) && (perimeterUse) && (millis() >= nextTimePerimeter)) {
    //bber2



    nextTimePerimeter = millis() +  15;
    if (perimeter.read2Coil) {
      perimeterMagRight = readSensor(SEN_PERIM_RIGHT);
    }
    perimeterMag = readSensor(SEN_PERIM_LEFT);
    if ((perimeter.isInside(0) != perimeterInside)) {
      perimeterCounter++;
      perimeterLastTransitionTime = millis();
      perimeterInside = perimeter.isInside(0);
    }

    if ((!perimeterInside) && (perimeterTriggerTime == 0)) {
      // set perimeter trigger time

      //bber2
      //use smooth to avoid big area transition, in the middle of the area with noise the mag can change from + to -
      smoothPeriMag = perimeter.getSmoothMagnitude(0);
      if (smoothPeriMag > perimeterTriggerMinSmag) {
        perimeterTriggerTime = millis();
      }
      else
      {
        if (millis() >= nextTimePrintConsole) {
          nextTimePrintConsole = millis() + 1000;
          if ((developerActive) && (stateCurr == STATE_FORWARD_ODO)) {
            Console.println("Bad reading perimeter In/Out, certainly we are very far the wire");
          }
        }
      }

    }


    if (perimeter.signalTimedOut(0) || ((perimeter.read2Coil) && perimeter.signalTimedOut(1) ))  {
      //bber2
      if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_MOW_SPIRALE))   { // all the other state are distance limited
        //need to find a way in tracking mode maybe timeout error if the tracking is perfect, the mower is so near the wire than the mag is near 0 (adjust the timedOutIfBelowSmag)
        //if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_MOW_SPIRALE))   { // all the other state are distance limited
        Console.println("Error: perimeter too far away");
        addErrorCounter(ERR_PERIMETER_TIMEOUT);
        setNextState(STATE_ERROR, 0);
        return;
      }
    }
  }


  if ((lawnSensorUse) && (millis() >= nextTimeLawnSensor)) {
    nextTimeLawnSensor = millis() + 100;
    double accel = 0.03;
    lawnSensorFront = (1.0 - accel) * lawnSensorFront + accel * ((double)readSensor(SEN_LAWN_FRONT));
    lawnSensorBack  = (1.0 - accel) * lawnSensorBack  + accel * ((double)readSensor(SEN_LAWN_BACK));
  }
  if ((lawnSensorUse) && (millis() >= nextTimeLawnSensorCheck)) {
    nextTimeLawnSensorCheck = millis() + 2000;
    double deltaFront = lawnSensorFront / lawnSensorFrontOld * 100.0;
    double deltaBack = lawnSensorBack / lawnSensorBackOld * 100.0;
    if ((deltaFront <= 95) || (deltaBack <= 95)) {
      Console.print(F("LAWN "));
      Console.print(deltaFront);
      Console.print(",");
      Console.println(deltaBack);
      lawnSensorCounter++;
      lawnSensor = true;
    }
    lawnSensorFrontOld = lawnSensorFront;
    lawnSensorBackOld  = lawnSensorBack;
  }

  if ((bumperUse) && (millis() >= nextTimeBumper)) {
    nextTimeBumper = millis() + 100;
    if (readSensor(SEN_BUMPER_LEFT) == 0) {
      //Console.println("Bumper left trigger");
      bumperLeftCounter++;
      bumperLeft = true;
    }

    if (readSensor(SEN_BUMPER_RIGHT) == 0) {
      //Console.println("Bumper right trigger");
      bumperRightCounter++;
      bumperRight = true;
    }
  }


  if ((dropUse) && (millis() >= nextTimeDrop)) {                                                                         // Dropsensor - Absturzsensor
    nextTimeDrop = millis() + 100;                                                                                          // Dropsensor - Absturzsensor
    if (readSensor(SEN_DROP_LEFT) == dropcontact) {                                                                         // Dropsensor - Absturzsensor
      dropLeftCounter++;                                                                                                    // Dropsensor - Absturzsensor
      dropLeft = true;                                                                                                      // Dropsensor - Absturzsensor
    }                                                                                                                       // Dropsensor - Absturzsensor

    if (readSensor(SEN_DROP_RIGHT) == dropcontact) {                                                                          // Dropsensor - Absturzsensor
      dropRightCounter++;                                                                                                   // Dropsensor - Absturzsensor
      dropRight = true;                                                                                                     // Dropsensor - Absturzsensor
    }
  }
  if (millis() >= nextTimeRTC) {
    // if ((timerUse) && (millis() >= nextTimeRTC)) {
    nextTimeRTC = millis() + 20000;
    readSensor(SEN_RTC);       // read RTC
    //Console.print(F("RTC date received: "));
    //Console.println(date2str(datetime.date));
    //Console.print(F("RTC time received: "));
    //Console.print(datetime.time.hour);
    //Console.print(F(":"));
    //Console.println(datetime.time.minute);


  }



  if (millis() >= nextTimeBattery) {
    // read battery
    nextTimeBattery = millis() + 500;
    if ((abs(chgCurrent) > 0.04) && (chgVoltage > 5)) {
      // charging
      batCapacity += (chgCurrent / 36.0);
    }
    //   batADC = readSensor(SEN_BAT_VOLTAGE);



    double batvolt = batFactor * readSensor(SEN_BAT_VOLTAGE) * 3.3 / 4096 ; //readsensor return the ADC value 0 to 4096 so *3.3/4096=voltage on the arduino pin batfactor depend on the resitor on board
    double chgvolt = batChgFactor * readSensor(SEN_CHG_VOLTAGE) * 3.3 / 4096 ;
    double curramp = batSenseFactor * readSensor(SEN_CHG_CURRENT) * 3.3 / 4096 ;
    /*
      Console.print(millis());
      Console.print("/batvolt ");
      Console.print(batvolt);
      Console.print("/chgvolt ");
      Console.print(chgvolt);
      Console.print("/curramp ");
      Console.println(curramp);
    */
    // low-pass filter
    //double accel = 0.01;
    double accel = 0.05;

    if (abs(batVoltage - batvolt) > 8)   batVoltage = batvolt; else batVoltage = (1.0 - accel) * batVoltage + accel * batvolt;
    if (abs(chgVoltage - chgvolt) > 8)   chgVoltage = chgvolt; else chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
    if (abs(chgCurrent - curramp) > 0.4) chgCurrent = curramp; else chgCurrent = (1.0 - accel) * chgCurrent + accel * curramp; //Deaktiviert fÃ¼r Ladestromsensor berechnung
    //bber30 tracking not ok with this but can check the chgvoltage
    /*
        Console.print(millis());
        Console.print("/batVoltage ");
        Console.print(batVoltage);
        Console.print("/chgVoltage ");
        Console.print(chgVoltage);
        Console.print("/chgCurrent ");
        Console.println(chgCurrent);
    */
  }

  if ((rainUse) && (millis() >= nextTimeRain)) {
    // read rain sensor
    nextTimeRain = millis() + 5000;
    rain = (readSensor(SEN_RAIN) != 0);
    if (rain) rainCounter++;
  }
}


void Robot::setDefaults() {
  motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
  motorMowEnable = false;
}

// set state machine new state
// called *ONCE* to set to a *NEW* state
void Robot::setNextState(byte stateNew, byte dir) {
  stateTime = millis() - stateStartTime; //last state duration
  if (stateNew == stateCurr) return;

  // evaluate new state
  stateNext = stateNew;
  rollDir = dir;

  switch (stateNew) {

    case STATE_FORWARD:
      if ((stateCurr == STATE_STATION_REV) || (stateCurr == STATE_STATION_ROLL) || (stateCurr == STATE_STATION_CHECK) ) return;
      if ((stateCurr == STATE_STATION) || (stateCurr == STATE_STATION_CHARGING)) {
        //stateNew = STATE_STATION_CHECK;
        setActuator(ACT_CHGRELAY, 0);
        motorMowEnable = false;
      }
      motorLeftSpeedRpmSet = motorSpeedMaxRpm; //use RPM instead of PWM to straight line
      motorRightSpeedRpmSet = motorSpeedMaxRpm;


      // motorLeftPID.reset();
      // motorRightPID.reset();
      //motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      statsMowTimeTotalStart = true;

      break;


    case STATE_FORWARD_ODO:
      if (statusCurr != NORMAL_MOWING) {
        statusCurr = NORMAL_MOWING;
        if (RaspberryPIUse) MyRpi.SendStatusToPi();
      }

      UseAccelRight = 0;
      UseAccelLeft = 0;
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
      //bber60

      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 30000);// set a very large distance 300 ml for random mowing
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 30000);
      if ((mowPatternCurr == MOW_LANES) && (!justChangeLaneDir)) { //it s a not new lane so limit the forward distance
        stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * actualLenghtByLane * 100); //limit the lenght
        stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * actualLenghtByLane * 100);
      }

      OdoRampCompute();

      statsMowTimeTotalStart = true;

      break;

    case STATE_ESCAPE_LANE:
      Console.println("Mowing in Half lane width");
      //it's approximation try to go into a parcel already mowed little on left or right
      halfLaneNb = halfLaneNb + 1; //to avoid repetition the state is lauch only if halfLaneNb=0
      UseAccelLeft = 0;
      UseBrakeLeft = 0;
      UseAccelRight = 0;
      UseBrakeRight = 0;
      if (rollDir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2;
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
        stateEndOdometryRight = odometryRight + (odometryTicksPerCm * odometryWheelBaseCm * 2) ;
        stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * odometryWheelBaseCm * 1.5 ) ;
      }
      else
      {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 2 ;
        stateEndOdometryRight = odometryRight + (odometryTicksPerCm * odometryWheelBaseCm * 1.5 ) ;
        stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * odometryWheelBaseCm * 2) ;
      }


      OdoRampCompute();

      break;


    case STATE_STATION_REV: //when start in auto mode the mower first reverse to leave the station
      statusCurr = TRACK_TO_START;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * stationRevDist);
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * stationRevDist);
      OdoRampCompute();
      break;

    case STATE_STATION_ROLL:  //when start in auto after mower reverse it roll for this angle
      if (mowPatternCurr == MOW_LANES)       AngleRotate = 90;
      else AngleRotate = random(30, 160);
      if (startByTimer) AngleRotate = stationRollAngle;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2 ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      OdoRampCompute();

      break;

    case STATE_STATION_FORW: //when start in auto after mower  roll this state accel the 2 wheel before forward

      justChangeLaneDir = true;
      UseAccelLeft = 1;
      UseAccelRight = 1;
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 60) ;//60CM to accel
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 60) ;
      OdoRampCompute();
      //motorMowEnable = true;

      break;

    case STATE_STATION_CHECK:
      //bber3
      if (statusCurr == WIRE_MOWING) { //it is the last status
        Console.print("Total distance drive ");
        Console.print(totalDistDrive / 100);
        Console.println(" meters ");
        Console.print("Total duration ");
        Console.print(int(millis() - stateStartTime) / 1000);
        Console.println(" secondes ");
        nextTimeTimer = millis() + 1200000; // only check again the timer after 20 minutes to avoid repetition
      }
      delayToReadVoltageStation = millis() + 1500; //the battery is read only each 500 ms so need a duration to be sure we have the last voltage
      //bber14 no accel here ?????
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 2;
      stateEndOdometryRight = odometryRight + (odometryTicksPerCm * stationCheckDist);
      stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * stationCheckDist);
      OdoRampCompute();


      break;

    //not use actually
    case STATE_PERI_ROLL:
      stateEndTime = millis() + perimeterTrackRollTime + motorZeroSettleTime;
      if (dir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2;
        motorRightSpeedRpmSet = -motorLeftSpeedRpmSet;
      } else {
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 2;
        motorLeftSpeedRpmSet = -motorRightSpeedRpmSet;
      }
      break;

    case STATE_PERI_OBSTACLE_REV:
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5 ;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * DistPeriObstacleRev);
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * DistPeriObstacleRev);
      OdoRampCompute();
      break;
    case STATE_PERI_OBSTACLE_ROLL:
      AngleRotate = 45;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5 ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ;
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      OdoRampCompute();
      break;

    case STATE_PERI_OBSTACLE_FORW:
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;
      motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriObstacleForw);//50cm
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriObstacleForw);
      OdoRampCompute();

      break;

    case STATE_PERI_OBSTACLE_AVOID:
      UseAccelLeft = 0;
      UseBrakeLeft = 0;
      UseAccelRight = 0;
      UseBrakeRight = 0;
      motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriObstacleAvoid);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriObstacleAvoid);
      OdoRampCompute();



      break;
    /*
        case STATE_STATION_AVOID:
          UseAccelLeft = 0;
          UseBrakeLeft = 0;
          UseAccelRight = 0;
          UseBrakeRight = 0;
          motorRightSpeedRpmSet = motorSpeedMaxRpm ;
          motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5;
          stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriObstacleAvoid);
          stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriObstacleAvoid);
          OdoRampCompute();

         break;
    */
    case STATE_PERI_REV:  //when obstacle in perifind
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight - 1440;
      stateEndOdometryLeft = odometryLeft - 1440;
      OdoRampCompute();
      break;


    case STATE_PERI_OUT_STOP: //in auto mode and forward slow down before stop and reverse
      //-------------------------------Verify if it's time to change mowing pattern
      if (mowPatternDuration > mowPatternDurationMax) {
        Console.println(" mowPatternCurr  change ");
        mowPatternCurr = (mowPatternCurr + 1) % 2; //change the pattern each x minutes
        mowPatternDuration = 0;
      }
      justChangeLaneDir = !justChangeLaneDir;  //use to know if the lane is not limit distance
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();

      break;


    case STATE_SONAR_TRIG: //in auto mode and forward slow down before stop and reverse different than stop because reduce speed during a long time and not immediatly
      justChangeLaneDir = !justChangeLaneDir;
      distToObstacle = distToObstacle - sonarToFrontDist; //   the distance between sonar and front of mower
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * distToObstacle);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * distToObstacle);
      OdoRampCompute();

      break;

    case STATE_STOP_TO_FIND_YAW:

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();

      break;

    case STATE_STOP_ON_BUMPER:

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm / 2 );
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm / 2);
      OdoRampCompute();

      break;

    case STATE_PERI_STOP_TOTRACK:
      //bber100 err here
      if (statusCurr == TRACK_TO_START) {
        if (mowPatternCurr == MOW_WIRE) {
          motorMowEnable = true; //time to start the blade
          statusCurr = WIRE_MOWING;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
        }
      }
      else if (statusCurr == WIRE_MOWING) {
        motorMowEnable = true; //time to start the blade
      }
      else {
        statusCurr = BACK_TO_STATION;
        if (RaspberryPIUse) MyRpi.SendStatusToPi();
      }

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;




    case STATE_PERI_STOP_TOROLL:
      imu.run(); //31/08/19 In peritrack the imu is stop so try to add this to start it now and avoid imu tilt error (occur once per week or less) ??????
      if (statusCurr == TRACK_TO_START) {
        startByTimer = false; // cancel because we have reach the start point and avoid repeat search entry
        justChangeLaneDir = false; //the first lane need to be distance control
        motorMowEnable = true; //time to start the blade
      }

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_PERI_STOP_TO_FAST_START:

      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_PERI_STOP_TO_NEWAREA:
      if ((statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START)) {
        statusCurr = REMOTE;
        if (RaspberryPIUse) MyRpi.SendStatusToPi();
        //startByTimer = false; // ?? not here                         cancel because we have reach the start point and avoid repeat search entry
        justChangeLaneDir = false; //the first lane need to be distance control
        perimeterUse = false; //disable the perimeter use to leave the area
        Console.println("Stop to read the perimeter wire");
        rollDir = LEFT;

      }
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_ROLL1_TO_NEWAREA:  // when find a tag the mower roll with new heading and drive in straight line
      AngleRotate = abs(newtagRotAngle1);
      newtagRotAngle1Radian = newtagRotAngle1 * PI / 180.0;
      Console.print("Actual Heading ");
      Console.println(imu.ypr.yaw * 180 / PI);
      remoteDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      Console.print("New Remote Heading ");
      Console.println(remoteDriveHeading * 180 / PI);
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      //Always rotate LEFT to leave mowing area
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = -motorSpeedMaxRpm  ;
      motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      OdoRampCompute();
      break;

    case STATE_ROLL2_TO_NEWAREA:  // when find a tag the mower roll with new heading and drive in straight line
      AngleRotate = newtagRotAngle2;
      newtagRotAngle1Radian = newtagRotAngle2 * PI / 180.0;
      Console.print("Actual Heading ");
      Console.println(imu.ypr.yaw * 180 / PI);
      remoteDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      Console.print("New Remote Heading ");
      Console.println(remoteDriveHeading * 180 / PI);
      if (AngleRotate >= 0) {
        rollDir = RIGHT;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
      }
      else {
        rollDir = LEFT;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
      }

      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;

      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later  HERE IT CAN BE NEGATIVE WHEN ROLL LEFT
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);





      OdoRampCompute();
      break;

    case STATE_DRIVE1_TO_NEWAREA:
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;
      currDistToDrive = 0; // when use the IMU the distance is not check on each wheel but on averrage
      // newtagDistance1 is the distance to drive
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * newtagDistance1);
      stateEndOdometryLeft = odometryLeft +  (int)(odometryTicksPerCm * newtagDistance1) ;
      OdoRampCompute();

      break;
    case STATE_DRIVE2_TO_NEWAREA:
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;
      currDistToDrive = 0;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * newtagDistance2);
      stateEndOdometryLeft = odometryLeft +  (int)(odometryTicksPerCm * newtagDistance2) ;
      OdoRampCompute();
      break;

    case STATE_STOP_TO_NEWAREA:
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutStop);
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutStop);
      OdoRampCompute();
      break;

    case STATE_WAIT_FOR_SIG2:
      statusCurr = WAITSIG2;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      //when the raspberry receive this new status it start the sender with the correct area sigcode
      totalDistDrive = 0; //reset the distance to track on the new area
      perimeterUse = true;
      Console.println("Start to read the Perimeter wire");
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight;//+ (odometryTicksPerCm * 10);
      stateEndOdometryLeft = odometryLeft;//+ (odometryTicksPerCm * 10);
      OdoRampCompute();

      break;

    case STATE_ROLL_TONEXTTAG:  // when find a tag the mower roll to leave the wire and go again in peirfind with new heading
      AngleRotate = newtagRotAngle1;
      newtagRotAngle1Radian = newtagRotAngle1 * PI / 180.0;
      Console.print("Actual Heading ");
      Console.println(imu.ypr.yaw * 180 / PI);
      periFindDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      Console.print("New PeriFind Heading ");
      Console.println(periFindDriveHeading * 180 / PI);
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      //Always rotate RIGHT to leave the wire
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      OdoRampCompute();
      break;

    case STATE_AUTO_CALIBRATE:
      nextTimeAddYawMedian = millis() + 500; //wait 500 ms to stabilize before record first yaw
      nextTimeToDmpAutoCalibration = millis() + delayBetweenTwoDmpAutocalib * 1000; //set the next time for calib
      //needDmpAutoCalibration = false;  //to avoid repetition
      endTimeCalibration = millis() + maxDurationDmpAutocalib * 1000;  //max duration calibration
      compassYawMedian.clear();
      accelGyroYawMedian.clear();

      break;


    case STATE_STOP_CALIBRATE:
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      if (actualRollDirToCalibrate != LEFT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm / 2 );
        stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm / 2 );

      } else {
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm / 2 ) ;
        stateEndOdometryLeft = odometryLeft - (int)(odometryTicksPerCm / 2 ) ;
      }
      //bber50
      OdoRampCompute();
      break;

    case STATE_STOP_BEFORE_SPIRALE:
      statusCurr = SPIRALE_MOWING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 20 ); //brake in 20CM
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 20);  //brake in 20CM
      OdoRampCompute();
      break;


    case STATE_ROTATE_RIGHT_360:
      spiraleNbTurn = 0;
      halfLaneNb = 0;
      highGrassDetect = false;
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5 ;
      stateEndOdometryRight = odometryRight - (int)36000 * (odometryTicksPerCm * PI * odometryWheelBaseCm / 36000);
      stateEndOdometryLeft = odometryLeft + (int)36000 * (odometryTicksPerCm * PI * odometryWheelBaseCm / 36000);
      OdoRampCompute();
      break;

    case STATE_NEXT_SPIRE:
      if (spiraleNbTurn == 0) {
        UseAccelLeft = 1;
        UseAccelRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ; ///adjust to change the access to next arc
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
      }
      else {
        UseAccelLeft = 0;
        UseAccelRight = 0;
      }
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      stateEndOdometryRight = odometryRight +  (odometryTicksPerCm * odometryWheelBaseCm / 2);
      stateEndOdometryLeft = odometryLeft +  (odometryTicksPerCm * odometryWheelBaseCm / 2);
      OdoRampCompute();

      break;

    case STATE_MOW_SPIRALE:
      float DistToDriveRight;
      float DistToDriveLeft;
      float Tmp;
      float Tmp1;
      setBeeper(0, 0, 0, 0, 0);
      UseAccelLeft = 0;
      UseAccelRight = 0;
      UseBrakeLeft = 0;
      UseBrakeRight = 0;

      if (spiraleNbTurn == 0) {
        R = (float)(odometryWheelBaseCm * 1.2); //*1.2 to avoid that wheel is completly stop
        DistToDriveRight = PI * (R / 2.00);
        DistToDriveLeft = PI * (R * 1.50);
      }
      else {
        //Console.println(R);
        R = R + (float)(odometryWheelBaseCm / 2);
        //Console.println(R);
        DistToDriveRight = PI * (R - ((float)odometryWheelBaseCm / 2.00));
        DistToDriveLeft = PI * (R + ((float)odometryWheelBaseCm / 2.00));
      }

      motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
      Tmp = 2 * (R - float(odometryWheelBaseCm));
      Tmp1 = 2.00 * (R + float(odometryWheelBaseCm));
      motorRightSpeedRpmSet = (int) (motorLeftSpeedRpmSet * Tmp / Tmp1) ;


      stateEndOdometryRight = odometryRight +  (odometryTicksPerCm * DistToDriveRight);
      stateEndOdometryLeft = odometryLeft +  (odometryTicksPerCm * DistToDriveLeft);
      /*
            Console.print("MOW SPIRALE R ");
            Console.print(R);
            Console.print(" Tmp ");
            Console.print(Tmp);
            Console.print(" Tmp1 ");
            Console.print(Tmp1);
            Console.print(" motorLeftSpeedRpmSet ");
            Console.print(motorLeftSpeedRpmSet);
            Console.print(" motorRightSpeedRpmSet ");
            Console.print(motorRightSpeedRpmSet);
            Console.print(" stateEndOdometryRight ");
            Console.print(stateEndOdometryRight);
            Console.print(" stateEndOdometryLeft ");
            Console.print(stateEndOdometryLeft);
            Console.print(" spiraleNbTurn ");
            Console.println(spiraleNbTurn);

      */



      OdoRampCompute();
      spiraleNbTurn = spiraleNbTurn + 1;

      break;

    case STATE_PERI_OUT_REV: //in normal mowing reverse after the wire trigger
      readDHT22(); // here the mower is stop so can spend 250ms  for reading
      setBeeper(0, 0, 0, 0, 0);
      perimeter.lastInsideTime[0] = millis(); //use to avoid perimetertimeout when mower outside perimeter
      if (mowPatternCurr == MOW_LANES) {
        PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
        PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      }
      else
      {
        PrevStateOdoDepassLeft = 0;
        PrevStateOdoDepassRight = 0;
      }
      if (rollDir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        if (mowPatternCurr == MOW_LANES)   UseBrakeRight = 1;
        else UseBrakeRight = 0;
      }
      else
      {
        UseAccelLeft = 1;
        if (mowPatternCurr == MOW_LANES)  UseBrakeLeft = 1;
        else UseBrakeLeft = 0;
        UseAccelRight = 1;
        UseBrakeRight = 1;
      }
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * DistPeriOutRev) - PrevStateOdoDepassRight;
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * DistPeriOutRev) - PrevStateOdoDepassLeft;
      OdoRampCompute();
      break;

    case STATE_PERI_OUT_ROLL: //roll left or right in normal mode
      if (mowPatternCurr == MOW_RANDOM) AngleRotate = random(motorRollDegMin, motorRollDegMax);

      if (dir == RIGHT) {
        if (mowPatternCurr == MOW_ZIGZAG) AngleRotate = imu.scale180(imuDriveHeading + 135); //need limit value to valib the rebon
        UseAccelLeft = 1;
        //bb6
        //UseBrakeLeft = 1;
        UseBrakeLeft = 0;
        UseAccelRight = 0;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
        stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      } else {
        if (mowPatternCurr == MOW_ZIGZAG) AngleRotate = imu.scale180(imuDriveHeading - 135); //need limit value to valib the rebon
        UseAccelLeft = 0;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        //bb6 =1
        UseBrakeRight = 0;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) ;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) ;
      }
      OdoRampCompute();
      break;

    case STATE_PERI_OUT_ROLL_TOINSIDE:  //roll left or right in normal mode
      //bber2
      perimeter.lastInsideTime[0] = millis(); //use to avoid perimetertimeout when mower outside perimeter
      if (stateCurr == STATE_WAIT_AND_REPEAT) {
        RollToInsideQty = RollToInsideQty + 1;
        Console.print("Not Inside roll nb: ");
        Console.println(RollToInsideQty);
      }
      else {
        RollToInsideQty = 0;
        Console.print("Find Inside roll nb: ");
        Console.println(RollToInsideQty);
      }

      if (mowPatternCurr == MOW_LANES) {
        //bber201
        if (autoBylaneToRandom) {
          mowPatternDuration = mowPatternDurationMax - 3 ; //set the mow_random for the next 3 minutes
          Console.println("We are in a corner mowPatternCurr change to Random for the next 3 minutes ");
          mowPatternCurr = MOW_RANDOM; //change the pattern each x minutes
        }

        laneUseNr = laneUseNr + 1;
        findedYaw = 999;
        justChangeLaneDir = true;
        nextTimeToDmpAutoCalibration = millis(); // so the at the end of the next line a calibration occur
        if (laneUseNr > 3) laneUseNr = 1;
      }

      AngleRotate = 50;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      if (dir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5 ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
        stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      } else {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
        motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) ;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);


      }
      OdoRampCompute();
      break;
    case STATE_PERI_OUT_ROLL_TOTRACK:  //roll left or right in normal mode

      AngleRotate = 180;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later

      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);


      OdoRampCompute();
      break;

    case STATE_PERI_OUT_STOP_ROLL_TOTRACK:  //roll right in normal mode when find wire
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.5;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
      stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm * 5); //stop on 5 cm
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 5);


      OdoRampCompute();
      break;
    case STATE_PERI_OUT_FORW:  //Accel after roll so the 2 wheel have the same speed when reach the forward state

      if ((mowPatternCurr == MOW_LANES) || (mowPatternCurr == MOW_ZIGZAG)) {
        PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
        PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      }
      else
      {
        PrevStateOdoDepassLeft = 0;
        PrevStateOdoDepassRight = 0;
      }
      if (dir == RIGHT) {
        if ((mowPatternCurr == MOW_LANES) || (mowPatternCurr == MOW_ZIGZAG))   UseAccelLeft = 1;
        else  UseAccelLeft = 0;
        UseAccelRight = 1;
      } else {

        if ((mowPatternCurr == MOW_LANES) || (mowPatternCurr == MOW_ZIGZAG))   UseAccelRight = 1;
        else  UseAccelRight = 0;
        UseAccelLeft = 1;

      }
      UseBrakeLeft = 0;
      UseBrakeRight = 0;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * DistPeriOutForw) - PrevStateOdoDepassRight;
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * DistPeriOutForw) - PrevStateOdoDepassLeft;
      OdoRampCompute();

      break;


    case STATE_PERI_OUT_LANE_ROLL1: //roll left or right in normal mode for 135 deg

      PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
      PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      AngleRotate = 135;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      if (dir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight =  odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight ;
        stateEndOdometryLeft =  odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft ;
      } else {
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft;
      }
      OdoRampCompute();
      break;

    case STATE_NEXT_LANE_FORW:  //small move to reach the next  parallel lane
      PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
      PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      UseAccelLeft = 1;
      UseAccelRight = 1;
      UseAccelLeft = 1;
      UseAccelRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;

      //************************************same as spirale in by lane mowing*******************************
      if (highGrassDetect) {
        Tempovar = DistBetweenLane / 2;
        halfLaneNb++; //count the nb of mowing lane in half lenght same as spirale into lane mowing
        Console.print("Hight grass detected actual halfLaneNb ");
        Console.println(halfLaneNb);
      }
      else Tempovar = DistBetweenLane;
      //****************************************************************************************************

      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * Tempovar) - PrevStateOdoDepassRight; //forward for  distance between lane
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * Tempovar) - PrevStateOdoDepassLeft;

      OdoRampCompute();
      break;

    case STATE_PERI_OUT_LANE_ROLL2: //roll left or right in normal mode

      PrevStateOdoDepassLeft = odometryLeft - stateEndOdometryLeft;
      PrevStateOdoDepassRight = odometryRight - stateEndOdometryRight;
      AngleRotate = 45;
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;

      if (dir == RIGHT) {
        motorLeftSpeedRpmSet = motorSpeedMaxRpm;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight =  odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight ;
        stateEndOdometryLeft =  odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft ;
      } else {
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm;
        motorRightSpeedRpmSet = motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassRight;
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar) - PrevStateOdoDepassLeft;
      }
      OdoRampCompute();
      break;

    case STATE_REVERSE:  //hit obstacle in forward state
      if (dir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 0;
      }
      if (dir == LEFT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 0;
        UseAccelRight = 1;
        UseBrakeRight = 1;
      }

      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm ;
      stateEndOdometryRight = odometryRight - (odometryTicksPerCm * DistPeriObstacleRev);
      stateEndOdometryLeft = odometryLeft - (odometryTicksPerCm * DistPeriObstacleRev);
      OdoRampCompute();
      /*
        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.25;
        stateEndTime = millis() + motorReverseTime + motorZeroSettleTime;
      */
      break;
    case STATE_ROLL:  // when hit obstacle in forward mode
      AngleRotate = random(50, 180);
      Tempovar = 36000 / AngleRotate; //need a value*100 for integer division later
      if (dir == RIGHT) {
        UseAccelLeft = 1;
        UseBrakeLeft = 0;
        UseAccelRight = 0;
        UseBrakeRight = 1;
        motorLeftSpeedRpmSet = motorSpeedMaxRpm ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);

      } else {
        UseAccelLeft = 0;
        UseBrakeLeft = 1;
        UseAccelRight = 1;
        UseBrakeRight = 0;
        motorRightSpeedRpmSet = motorSpeedMaxRpm ;
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm;
        stateEndOdometryRight = odometryRight + (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
        stateEndOdometryLeft = odometryLeft - (int)100 * (odometryTicksPerCm * PI * odometryWheelBaseCm / Tempovar);
      }
      OdoRampCompute();
      break;




    case STATE_TEST_COMPASS:  // to test the imu
      statusCurr = TESTING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
      stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm * 2 * PI * odometryWheelBaseCm );
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 2 *  PI * odometryWheelBaseCm );



      OdoRampCompute();


      break;


    case STATE_CALIB_MOTOR_SPEED:
      statusCurr = TESTING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      OdoRampCompute();

      break;

    case STATE_TEST_MOTOR:
      statusCurr = TESTING;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      OdoRampCompute();

      break;


    case STATE_ROLL_TO_FIND_YAW:  // roll slowly 720 deg until find the positive yaw, state will be changed by the IMU
      if (stopMotorDuringCalib) motorMowEnable = false;//stop the mow motor
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      /*
        Console.print(" imu.comYaw ");
        Console.print(abs(100 * imu.comYaw));
        Console.print(" imu.ypr.yaw ");
        Console.print(abs(100 * imu.ypr.yaw));
        Console.print(" distancePI(imu.comYaw, imu.ypr.yaw) ");
        Console.println(distancePI(imu.comYaw, imu.ypr.yaw));
      */


      if (distancePI(imu.comYaw, yawCiblePos * PI / 180) > 0) { //rotate in the nearest direction
        actualRollDirToCalibrate = RIGHT;
        //Console.println(" >>> >>> >>> >>> >>> >>> 0");
        motorLeftSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100 ;
        motorRightSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
        stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
        stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
      }
      else
      {
        actualRollDirToCalibrate = LEFT;
        //Console.println(" <<< <<< <<< <<< <<< << 0");
        motorLeftSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100 ;
        motorRightSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
        stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
        stateEndOdometryLeft = odometryLeft - (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
      }


      OdoRampCompute();
      break;




    case STATE_ROLL_WAIT:
      ///use to make test with o in console but never call in normal mode

      //roll slowly 360 deg to find the yaw state is stopped by the IMU
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 2;
      stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm *  2 * PI * odometryWheelBaseCm );
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm *  2 * PI * odometryWheelBaseCm );

      OdoRampCompute();
      break;
    case STATE_MANUAL:
      statusCurr = MANUAL;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      break;
    case STATE_REMOTE:
      statusCurr = REMOTE;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      motorMowEnable = false;
      break;
    case STATE_STATION: //stop immediatly
      areaInMowing = 1;
      //ignoreRfidTag = false;
      motorMowEnable = false;
      startByTimer = false;
      totalDistDrive = 0; //reset the tracking distance travel
      whereToResetSpeed = 50000; // initial value to 500 meters
      ActualSpeedPeriPWM = MaxSpeedperiPwm; //reset the tracking speed
      statusCurr = IN_STATION;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      //time to reset the speed because the Peri find can use very high speed
      motorSpeedMaxPwm = motorInitialSpeedMaxPwm;
      stateEndOdometryRight = odometryRight;
      stateEndOdometryLeft = odometryLeft ;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
      setMotorPWM(0, 0, false);
      setActuator(ACT_CHGRELAY, 0);
      setDefaults();
      statsMowTimeTotalStart = false;  // stop stats mowTime counter
      //bber30
      loadSaveRobotStats(false);        //save robot stats

      break;

    case STATE_STATION_CHARGING:
      setActuator(ACT_CHGRELAY, 1);
      setDefaults();
      break;

    case STATE_OFF:
      statusCurr = WAIT;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();

      startByTimer = false;// reset the start timer
      setActuator(ACT_CHGRELAY, 0);
      setDefaults();
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
      stateEndOdometryRight = odometryRight + (odometryTicksPerCm * 20);
      stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * 20);
      // stateMaxiTime = millis() + 5000;
      OdoRampCompute();
      statsMowTimeTotalStart = false; // stop stats mowTime counter
      loadSaveRobotStats(false);      //save robot stats
      break;

    case STATE_ERROR:
      statusCurr = IN_ERROR;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      setActuator(ACT_CHGRELAY, 0);
      motorMowEnable = false;
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
      stateEndOdometryRight = odometryRight + (odometryTicksPerCm * 20);
      stateEndOdometryLeft = odometryLeft + (odometryTicksPerCm * 20);
      // stateMaxiTime = millis() + 5000;
      OdoRampCompute();
      statsMowTimeTotalStart = false; // stop stats mowTime counter
      loadSaveRobotStats(false);      //save robot stats
      break;


    case STATE_PERI_FIND:
      //Don't Use accel when start from forward_odo because the 2 wheels are already running
      //if status is change in pfod need to refresh it in PI
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      Console.print("Area In Mowing ");
      Console.print(areaInMowing);
      Console.print(" Area To Go ");
      Console.println(areaToGo);

      if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_OBSTACLE_AVOID)) {
        UseAccelRight = 0;
        UseAccelLeft = 0;
      }
      else {
        UseAccelRight = 1;
        UseAccelLeft = 1;
      }

      UseBrakeLeft = 0;
      UseBrakeRight = 0;

      motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.2;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 1.2;
      stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm * 30000);//300 ml
      stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm * 30000);
      OdoRampCompute();


      motorMowEnable = false;
      break;


    case STATE_PERI_TRACK:
      //motorMowEnable = false;     // FIXME: should be an option?
      perimeterPID.reset();

      break;

    case STATE_WAIT_AND_REPEAT:
      //Console.println("WAIT AND REPEAT  ");

      break;

    //bber202
    case STATE_ACCEL_FRWRD:
      //use to start mow motor at low speed and limit noise on perimeter reading on startup
      motorMowSpeedPWMSet = motorMowSpeedMinPwm;
      motorMowPowerMedian.clear();
      
      // after this state the mower use pid imu to drive straight so accelerate only at half the max speed
      UseAccelLeft = 1;
      UseBrakeLeft = 0;
      UseAccelRight = 1;
      UseBrakeRight = 0;
      motorRightSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm / 2 ;
      stateEndOdometryRight = odometryRight + int(odometryTicksPerRevolution / 4) ;
      stateEndOdometryLeft = odometryLeft + int(odometryTicksPerRevolution / 4) ;
      OdoRampCompute();

      break;

  }  // end switch

  sonarObstacleTimeout = 0;
  // state has changed

  stateStartTime = millis();

  stateNext = stateNew;
  stateLast = stateCurr;
  stateCurr = stateNext;
  perimeterTriggerTime = 0;
  Console.print (F(statusNames[statusCurr]));
  Console.print (" / ");
  Console.println (F(stateNames[stateCurr]));
  //Console.print (" Dir ");
  //Console.print (rollDir);
  //Console.print (" State changed at ");
  //Console.print (stateStartTime);
  //Console.print (" From state ");
  //Console.println (F(stateNames[stateLast]));

}
// check battery voltage and decide what to do
void Robot::checkBattery() {

  if ((millis() < nextTimeCheckBattery) || (millis() < 30000)) return; //  wait 30 sec after the initial power on before first check to avoid read bad battery voltage
  nextTimeCheckBattery = millis() + 1000; //if change need to adjust the line idleTimeSec= idleTimeSec+1;

  if (batMonitor) {
    // if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_OFF) && (stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING))  {
    if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_OFF))   {
      Console.print(F("Batterie Voltage : "));
      Console.print(batVoltage);
      Console.print(F(" -- > Switch OFF Voltage : "));
      Console.print(batSwitchOffIfBelow);
      Console.println(F("  Bat Voltage is very low the state is changed to OFF, so the undervoltage timer start"));
      addErrorCounter(ERR_BATTERY);
      setBeeper(100, 25, 25, 200, 0 );
      setNextState(STATE_OFF, 0);
    }
    else if ((batVoltage < batGoHomeIfBelow) && (stateCurr == STATE_FORWARD_ODO) && (perimeterUse)) {    //actualy in mowing mode with station and perimeter
      Console.print(F("Batterie Voltage : "));
      Console.print(batVoltage);
      Console.print(F(" -- > Minimum Mowing Voltage : "));
      Console.println(batGoHomeIfBelow);
      Console.println(F(" Bat Voltage is low : The mower search the charging Station"));
      setBeeper(100, 25, 25, 200, 0 );
      statusCurr = BACK_TO_STATION;
      areaToGo = 1;
      if (RaspberryPIUse) MyRpi.SendStatusToPi();
      setNextState(STATE_PERI_FIND, 0);
    }


    // if robot is OFF or Error  we can start to count before shutdown
    if ( (stateCurr == STATE_OFF) || (stateCurr == STATE_ERROR)) {
      //if ( (stateCurr == STATE_OFF) || (stateCurr == STATE_ERROR) || ((stateCurr == STATE_STATION) && !timerUse)) {
      /*
        Console.print("Count before power OFF  ");
        Console.print(idleTimeSec);
        Console.print(" / ");
        Console.println(batSwitchOffIfIdle * 60);
      */
      if (idleTimeSec != BATTERY_SW_OFF) { // battery already switched off?
        idleTimeSec = idleTimeSec + 1; // add 1 second idle time because check only each 1 secondes
        if (idleTimeSec > batSwitchOffIfIdle * 60) {

          if (RaspberryPIUse) {
            Console.println(F("Battery IDLE trigger "));
            Console.println(F("PCB power OFF after 30 secondes Wait Until PI Stop "));
            MyRpi.sendCommandToPi("PowerOffPi");
            delayWithWatchdog(30000);//wait 30Sec  until pi is OFF or the USB native power again the due and the undervoltage never switch OFF
          }
          else
          {
            Console.println(F("PCB power OFF immediatly"));
          }
          setBeeper(200, 50, 50, 200, 100 );
          loadSaveErrorCounters(false); // saves error counters
          loadSaveRobotStats(false);    // saves robot stats
          idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
          Console.println(F("BATTERY switching OFF"));
          setActuator(ACT_BATTERY_SW, 0);  // switch off battery
        }
      }
    }
    else
    {
      resetIdleTime();
    }
  }
}


// check robot stats
void Robot::checkRobotStats() {
  if (millis() < nextTimeRobotStats) return;
  nextTimeRobotStats = millis() + 60000;

  //----------------stats mow time------------------------------------------------------
  statsMowTimeHoursTotal = double(statsMowTimeMinutesTotal) / 60;
  if (statsMowTimeTotalStart) {
    statsMowTimeMinutesTripCounter++;
    statsMowTimeMinutesTrip = statsMowTimeMinutesTripCounter;
    statsMowTimeMinutesTotal++;
  }
  else if (statsMowTimeMinutesTripCounter != 0) {
    statsMowTimeMinutesTripCounter = 0;

  }

  //---------------stats Battery---------------------------------------------------------
  if ((stateCurr == STATE_STATION_CHARGING) && (stateTime >= 60000)) { // count only if mower is charged longer then 60sec
    statsBatteryChargingCounter++; // temporary counter
    if (statsBatteryChargingCounter == 1) statsBatteryChargingCounterTotal += 1;
    statsBatteryChargingCapacityTrip = batCapacity;
    statsBatteryChargingCapacityTotal += (batCapacity - lastTimeBatCapacity); // summ up only the difference between actual batCapacity and last batCapacity
    lastTimeBatCapacity = batCapacity;
  }
  else {                        // resets values to 0 when mower is not charging
    statsBatteryChargingCounter = 0;
    batCapacity = 0;
  }

  if (isnan(statsBatteryChargingCapacityTrip)) statsBatteryChargingCapacityTrip = 0;
  if (isnan(statsBatteryChargingCounterTotal)) statsBatteryChargingCounterTotal = 0; // for first run ensures that the counter is 0
  if (isnan(statsBatteryChargingCapacityTotal)) statsBatteryChargingCapacityTotal = 0; // for first run ensures that the counter is 0
  if (statsBatteryChargingCapacityTotal <= 0 || statsBatteryChargingCounterTotal == 0) statsBatteryChargingCapacityAverage = 0; // make sure that there is no dividing by zero
  else statsBatteryChargingCapacityAverage = statsBatteryChargingCapacityTotal / statsBatteryChargingCounterTotal;

  //----------------new stats goes here------------------------------------------------------
  //mowPatternJustChange;
  mowPatternDuration++;


}


void Robot::reverseOrBidir(byte aRollDir) {

  if (mowPatternCurr == MOW_LANES) setNextState(STATE_STOP_ON_BUMPER, rollDir);
  else  setNextState(STATE_STOP_ON_BUMPER, aRollDir);
}

// check motor current
void Robot::checkCurrent() {
  if (millis() < nextTimeCheckCurrent) return;
  nextTimeCheckCurrent = millis() + 100;
  if (statusCurr == NORMAL_MOWING) {  //do not start the spirale if in tracking and motor detect high grass
    if (motorMowPower >= 0.8 * motorMowPowerMax) {
      spiraleNbTurn = 0;
      halfLaneNb = 0;
      highGrassDetect = true;
      Console.println("Warning  motorMowPower >= 0.8 * motorMowPowerMax ");
      ////  http://forums.parallax.com/discussion/comment/1326585#Comment_1326585
    }
    else {
      if ((spiraleNbTurn >= 8) || (halfLaneNb >= 8)) {
        spiraleNbTurn = 0;
        halfLaneNb = 0;
        highGrassDetect = false; //stop the spirale
      }
    }
  }

  // if (motorMowPower >= motorMowPowerMax)
  if ((motorMowEnable) && (motorMowPower >= motorMowPowerMax))
  {
    motorMowSenseCounter++;
    Console.print("Warning  motorMowPower >= motorMowPowerMax and Counter time is ");
    Console.println(motorMowSenseCounter);
  }
  else
  {
    errorCounterMax[ERR_MOW_SENSE] = 0;
    motorMowSenseCounter = 0;
    if ((lastTimeMotorMowStuck != 0) && (millis() >= lastTimeMotorMowStuck + 60000)) { // wait 60 seconds before switching on again
      errorCounter[ERR_MOW_SENSE] = 0;
      if ((stateCurr == STATE_FORWARD_ODO)) { //avoid risq of restart not allowed
        motorMowEnable = true;
        lastTimeMotorMowStuck = 0;
        Console.println("Time to restart the mow motor after the 60 secondes pause");
      }
    }
  }
  //need to check this
  if (motorMowSenseCounter >= 10) { //ignore motorMowPower for 1 seconds
    motorMowEnable = false;
    Console.println("Motor mow power overload. Motor STOP and try to start again after 1 minute");
    addErrorCounter(ERR_MOW_SENSE);
    lastTimeMotorMowStuck = millis();
  }

  //bb add test current in manual mode and stop immediatly
  if (statusCurr == MANUAL) {
    if (motorLeftPower >= 0.8 * motorPowerMax) {
      Console.print("Motor Left power is 80 % of the max, value --> ");
      Console.println(motorLeftPower);
      setMotorPWM( 0, 0, false );
      setNextState(STATE_OFF, 0);

    }
    if (motorRightPower >= 0.8 * motorPowerMax) {
      Console.print("Motor Right power is 80 % of the max, value --> ");
      Console.println(motorRightPower);
      setMotorPWM( 0, 0, false );
      setNextState(STATE_OFF, 0);

    }
  }


  // here in auto mode
  if (millis() > stateStartTime + motorPowerIgnoreTime) {

    //Motor right****************************************************************
    //First react test to 80 % powerMax
    if (motorRightPower >= 0.8 * motorPowerMax)
    {
      motorRightSenseCounter++;
      setBeeper(1000, 50, 50, 200, 100);
      setMotorPWM( 0, 0, false );
      Console.print("Motor Right power is 80 % of the max, value --> ");
      Console.println(motorRightPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          Console.println("Power motor left warning ");
          setNextState(STATE_STATION_CHECK, rollDir);
          return;
        }
        else
        {
          if (mowPatternCurr == MOW_LANES) reverseOrBidir(rollDir);
          else reverseOrBidir(LEFT);
        }
      }
    }
    else
    {
      setBeeper(0, 0, 0, 0, 0);
      motorRightSenseCounter = 0; // the sense is OK reset all the counter
    }
    //Second test at powerMax by increase the counter to stop to error
    if (motorRightPower >= motorPowerMax) {
      motorRightSenseCounter++;
      //setMotorPWM( 0, 0, false );
      //addErrorCounter(ERR_MOTOR_RIGHT);
      //setNextState(STATE_ERROR, 0);
      Console.print("Warning: Motor Right power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      Console.println(motorRightSenseCounter);

    }


    //Motor left****************************************************************
    //First react test to 80 % powerMax
    if (motorLeftPower >= 0.8 * motorPowerMax)
    {
      motorLeftSenseCounter++;
      setBeeper(1000, 50, 50, 100, 50);
      setMotorPWM( 0, 0, false );
      Console.print("Motor Left power is 80 % of the max, value --> ");
      Console.println(motorLeftPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          Console.println("Power motor left warning ");
          setNextState(STATE_STATION_CHECK, rollDir);
          return;
        }
        else
        {
          if (mowPatternCurr == MOW_LANES) reverseOrBidir(rollDir);
          else reverseOrBidir(RIGHT);
        }
      }
    }
    else
    {
      setBeeper(0, 0, 0, 0, 0);
      motorLeftSenseCounter = 0; // the sense is OK reset the counter
    }
    //Second test at powerMax by increase the counter to stop to error
    if (motorLeftPower >= motorPowerMax) {
      motorLeftSenseCounter++;
      //setMotorPWM( 0, 0, false );
      //addErrorCounter(ERR_MOTOR_LEFT);
      //setNextState(STATE_ERROR, 0);
      Console.print("Warning: Motor Left power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      Console.println(motorLeftSenseCounter);
    }
    //final test on the counter to generate the error and stop the mower
    if (motorLeftSenseCounter >= 10) { //the motor is stuck for more than 1 seconde 10 * 100 ms go to error.
      Console.print("Fatal Error: Motor Left power over 100% for more than 1 seconde last power --> ");
      Console.println(motorLeftPower);
      addErrorCounter(ERR_MOTOR_LEFT);
      setMotorPWM( 0, 0, false );
      setNextState(STATE_ERROR, 0);
    }
    if (motorRightSenseCounter >= 10) { //the motor is stuck for more than 1 seconde go to error.
      Console.print("Fatal Error: Motor Right power over 100% for more than 1 seconde last power --> ");
      Console.println(motorRightPower);
      addErrorCounter(ERR_MOTOR_RIGHT);
      setMotorPWM( 0, 0, false );
      setNextState(STATE_ERROR, 0);
    }

  } //motorpower ignore time

}

// check bumpers
void Robot::checkBumpers() {
  if ((millis() < 3000) || (!bumperUse)) return;

  if ((bumperLeft || bumperRight)) {
    if (statusCurr == MANUAL) {
      Console.println("Bumper trigger in Manual mode ?????????");
      setNextState(STATE_OFF, 0); //the bumper stop all in manual mode
    }
    else
    {
      spiraleNbTurn = 0;
      highGrassDetect = false;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
      setMotorPWM( 0, 0, false );
      if (bumperLeft) {
        Console.println("Bumper left trigger");
        reverseOrBidir(LEFT);
      } else {
        Console.println("Bumper right trigger");
        reverseOrBidir(RIGHT);
      }
    }

  }
}

// check drop
void Robot::checkDrop() {  //the drop is used as a contact in front of the robot to detect the charging station
  if ((millis() < 3000) || (!dropUse)) return;
  if ((dropLeft || dropRight)) {
    if (statusCurr == MANUAL) {
      Console.println("Drop trigger in Manual mode ?????????");
      setNextState(STATE_OFF, 0); //the drop stop all in manual mode
    }
    else
    {
      spiraleNbTurn = 0;
      highGrassDetect = false;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
      setMotorPWM( 0, 0, false );
      if (dropLeft) {
        reverseOrBidir(RIGHT);
      } else {
        reverseOrBidir(LEFT);
      }
    }
  }
}                                                                                                                                   // Dropsensor - Absturzsensor

// check bumpers while tracking perimeter
void Robot::checkBumpersPerimeter() {
  if ((bumperLeft || bumperRight)) { // the bumper is used to detect the station
    motorLeftRpmCurr = motorRightRpmCurr = 0 ;
    setMotorPWM( 0, 0, false );//stop immediatly and station check to see if voltage on pin
    nextTimeBattery = millis();
    readSensors();  //read the chgVoltage
    Console.println("Bump on Something check if it's the station");
    setNextState(STATE_STATION_CHECK, rollDir);
    return;
  }

  if (!UseBumperDock) {   // run slower because read fast the station voltage but we don't use bumper to detect station we use only charging voltage
    //bber30
    nextTimeBattery = millis();
    readSensors();  //read the chgVoltage immediatly
    if (chgVoltage > 5) {
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      setMotorPWM( 0, 0, false );//stop immediatly and wait 2 sec to see if voltage on pin
      Console.println("Detect a voltage on charging contact check if it's the station");
      setNextState(STATE_STATION_CHECK, rollDir);
    }
  }
}

// check perimeter as a boundary
void Robot::checkPerimeterBoundary() {

  if ((millis() >= nextTimeRotationChange) && (stateCurr == STATE_FORWARD_ODO) && (mowPatternCurr != MOW_LANES)) {// change only when in straight line and random mode
    nextTimeRotationChange = millis() + 600000;  // in random change each 10 minutes
    if (rollDir == LEFT) rollDir = RIGHT; //invert the next rotate
    else rollDir = LEFT;
    Console.print(millis());
    Console.println(" Rotation direction Left / Right change ");
  }
  //bber2
  if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_MOW_SPIRALE) ) {
    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        Console.print(F("Perimeter trigger at : "));
        Console.println(millis());
        //reinit spirale mowing
        spiraleNbTurn = 0;
        halfLaneNb = 0;
        highGrassDetect = false; //stop the spirale
        setNextState(STATE_PERI_OUT_STOP, rollDir);
        return;
      }
    }
  }

  if ((stateCurr == STATE_PERI_OBSTACLE_AVOID)) { //when start in auto mode and quit the station
    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        setNextState(STATE_PERI_STOP_TOTRACK, rollDir);
        return;
      }
    }
  }


  if (stateCurr == STATE_SONAR_TRIG) {  //wire is detected during the sonar braking need to stop immediatly
    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        setNextState(STATE_STOP_ON_BUMPER, rollDir);  //use to stop immediatly
        return;
      }
    }
  }

  else if ((stateCurr == STATE_ROLL)) {
    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        Console.println("Pourquoi je suis la ?? ?? ?? ?? ?? ?? ?? ?? ");
        setMotorPWM( 0, 0, false );
        setNextState(STATE_PERI_OUT_REV, rollDir);
        return;

      }
    }
  }

}




// check lawn
void Robot::checkLawn() {
  if (!lawnSensorUse) return;
  if ( (lawnSensor) && (millis() > stateStartTime + 3000) ) {
    if (rollDir == RIGHT) reverseOrBidir(LEFT); // toggle roll dir
    else reverseOrBidir(RIGHT);
  } else lawnSensor = false;
}

void Robot::checkRain() {
  if (!rainUse) return;
  if (rain) {
    Console.println(F("RAIN"));
    areaToGo = 1;
    if (perimeterUse) {
      setNextState(STATE_PERI_FIND, 0);
    }
    else {
      setNextState(STATE_OFF, 0);
    }
  }
}
void Robot::checkSonarPeriTrack() {
  if (!sonarUse) return;
  if (millis() < nextTimeCheckSonar) return;
  nextTimeCheckSonar = millis() + 200;

  /*
    if (millis() > timeToResetSpeedPeri) {
    timeToResetSpeedPeri = 0; //brake the tracking during 6 secondes
    ActualSpeedPeriPWM = MaxSpeedperiPwm ;
    }
  */
  if (sonarRightUse) sonarDistRight = readSensor(SEN_SONAR_RIGHT);
  else sonarDistRight = NO_ECHO;
  //if (sonarLeftUse) sonarDistLeft = readSensor(SEN_SONAR_LEFT);
  //if (sonarCenterUse) sonarDistCenter = readSensor(SEN_SONAR_CENTER);

  //if (sonarDistCenter < 30 || sonarDistCenter > 150) sonarDistCenter = NO_ECHO; //need to be adjust if sonar is directly in front of mower 25Cm in my case
  if (sonarDistRight < 30 || sonarDistRight > 150) sonarDistRight = NO_ECHO; // Object is too close to the sensor JSN SR04T can't read <20 CM . Sensor value is useless
  //if (sonarDistLeft < 30 || sonarDistLeft  > 150) sonarDistLeft = NO_ECHO;

  //disabled the left sonar during tracking with this line
  sonarDistLeft = NO_ECHO;
  sonarDistCenter = NO_ECHO;

  if ((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow))  {
    //if (((sonarDistCenter != NO_ECHO) && (sonarDistCenter < sonarTriggerBelow))  ||  ((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow)) ||  ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow))  ) {
    //setBeeper(1000, 50, 50, 60, 60);
    Console.println("Sonar reduce speed on tracking for 2 meters");
    whereToResetSpeed =  totalDistDrive + 200; // when a speed tag is read it's where the speed is back to maxpwm value

    nextTimeCheckSonar = millis() + 4000;  //wait before next reading
    // timeToResetSpeedPeri = millis() + 10000; //brake the tracking during 10 secondes
    ActualSpeedPeriPWM = MaxSpeedperiPwm * dockingSpeed / 100;
    trakBlockInnerWheel = 1; //don't want that a wheel reverse just before station check   /bber30


  }
}

// check sonar
void Robot::checkSonar() {
  if (!sonarUse) return;
  if (millis() < nextTimeCheckSonar) return;
  nextTimeCheckSonar = millis() + 100;
  sonarSpeedCoeff = 1;
  if (sonarRightUse) sonarDistRight = readSensor(SEN_SONAR_RIGHT);
  else sonarDistRight = NO_ECHO;
  if (sonarLeftUse) sonarDistLeft = readSensor(SEN_SONAR_LEFT);
  else sonarDistLeft = NO_ECHO ;
  if (sonarCenterUse) sonarDistCenter = readSensor(SEN_SONAR_CENTER);
  else sonarDistCenter = NO_ECHO;

  if (stateCurr == STATE_OFF) return; //avoid the mower move when testing

  if (sonarDistCenter < 25 || sonarDistCenter > 90) sonarDistCenter = NO_ECHO; //need to be adjust if sonar is directly in front of mower 25Cm in my case
  if (sonarDistRight < 25 || sonarDistRight > 90) sonarDistRight = NO_ECHO; // Object is too close to the sensor JSN SR04T can't read <20 CM . Sensor value is useless
  if (sonarDistLeft < 25 || sonarDistLeft  > 90) sonarDistLeft = NO_ECHO;

  if (((sonarDistCenter != NO_ECHO) && (sonarDistCenter < sonarTriggerBelow))  ||  ((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow)) ||  ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow))  ) {
    setBeeper(2000, 500, 500, 60, 60);
    nextTimeCheckSonar = millis() + 1500;  //wait before next reading

    //**************************if sonar during spirale reinit spirale variable*****************
    spiraleNbTurn = 0;
    halfLaneNb = 0;
    highGrassDetect = false; //stop the spirale
    //*********************************************************************************
    if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_MOW_SPIRALE)) {
      //avoid the mower move when testing
      if ((sonarDistCenter != NO_ECHO) && (sonarDistCenter < sonarTriggerBelow)) {  //center
        //bber200
        if (!sonarLikeBumper) {
          sonarSpeedCoeff = 0.70;
          nextTimeCheckSonar = millis() + 3000;
        }
        else {

          distToObstacle =  sonarDistCenter;
          Console.print("Sonar Center Trigger at cm : ");
          Console.println (distToObstacle);
          setNextState(STATE_SONAR_TRIG, rollDir);  //don't change the rotation if center
          return;
        }
      }
      if ((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow)) {  //right
        if (!sonarLikeBumper) {
          sonarSpeedCoeff = 0.70;
          nextTimeCheckSonar = millis() + 3000;
        }
        else {
          distToObstacle =  sonarDistRight;
          Console.print("Sonar Right Trigger at cm : ");
          Console.println (distToObstacle);
          if (mowPatternCurr == MOW_LANES) setNextState(STATE_SONAR_TRIG, rollDir); //don't change the rotation if lane mowing
          else setNextState(STATE_SONAR_TRIG, LEFT);
          return;
        }
      }
      if ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow)) {  //LEFT
         if (!sonarLikeBumper) {
          sonarSpeedCoeff = 0.70;
          nextTimeCheckSonar = millis() + 3000;
        }
        else {
          distToObstacle =  sonarDistLeft;
          Console.print("Sonar Left Trigger at cm : ");
          Console.println (distToObstacle);
          if (mowPatternCurr == MOW_LANES) setNextState(STATE_SONAR_TRIG, rollDir); //don't change the rotation if lane mowing
          else setNextState(STATE_SONAR_TRIG, RIGHT);
          return;
        }
      }
    }
  }

}






// check IMU (tilt)
void Robot::checkTilt() {
  if (!imuUse) return;
  if (millis() < nextTimeCheckTilt) return;
  nextTimeCheckTilt = millis() + 50; // 5Hz same as nextTimeImu
  int pitchAngle = (imu.ypr.pitch / PI * 180.0);
  int rollAngle  = (imu.ypr.roll / PI * 180.0);
  //bber4
  if ( (stateCurr != STATE_OFF) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING)) {
    if ( (abs(pitchAngle) > 40) || (abs(rollAngle) > 40) ) {
      Console.print(F("Error : IMU Roll / Tilt---------------------------------------------------------------------------- -- > "));
      Console.print(rollAngle);
      Console.print(F(" / "));
      Console.println(pitchAngle);
      addErrorCounter(ERR_IMU_TILT);
      setNextState(STATE_ERROR, 0);
    }
  }
}

// check if mower is stuck ToDo: take HDOP into consideration if gpsSpeed is reliable
void Robot::checkIfStuck() {
  if (millis() < nextTimeCheckIfStuck) return;
  nextTimeCheckIfStuck = millis() + 500;
  /*
    if ((gpsUse) && (gps.hdop() < 500))  {
    //float gpsSpeedRead = gps.f_speed_kmph();
    float gpsSpeed = gps.f_speed_kmph();
    //bb
    //if (gpsSpeedIgnoreTime >= motorReverseTime) gpsSpeedIgnoreTime = motorReverseTime - 500;
    // low-pass filter
    // double accel = 0.1;
    // float gpsSpeed = (1.0-accel) * gpsSpeed + accel * gpsSpeedRead;
    // Console.println(gpsSpeed);
    // Console.println(robotIsStuckCounter);
    // Console.println(errorCounter[ERR_STUCK]);
    if ((stateCurr != STATE_MANUAL) && (stateCurr != STATE_REMOTE) && (gpsSpeed <= stuckIfGpsSpeedBelow)    // checks if mower is stuck and counts up
    && ((motorLeftRpmCurr && motorRightRpmCurr) != 0) && (millis() > stateStartTime + gpsSpeedIgnoreTime) ) {
    robotIsStuckCounter++;
    }


    else {                         // if mower gets unstuck it resets errorCounterMax to zero and reenabling motorMow
    robotIsStuckCounter = 0;    // resets temporary counter to zero
    if ( (errorCounter[ERR_STUCK] == 0) && (stateCurr != STATE_OFF) && (stateCurr != STATE_MANUAL) && (stateCurr != STATE_STATION)
    && (stateCurr != STATE_STATION_CHARGING) && (stateCurr != STATE_STATION_CHECK)
    && (stateCurr != STATE_STATION_REV) && (stateCurr != STATE_STATION_ROLL)
    && (stateCurr != STATE_REMOTE) && (stateCurr != STATE_ERROR)) {
    Console.println("Mower not stuck       MOW can start");
    // motorMowEnable = true;
    errorCounterMax[ERR_STUCK] = 0;
    }

    return;
    }

    if (robotIsStuckCounter >= 5) {
    motorMowEnable = false;
    if (errorCounterMax[ERR_STUCK] >= 3) {  // robot is definately stuck and unable to move
    Console.println(F("Error : Mower is stuck"));
    addErrorCounter(ERR_STUCK);
    setNextState(STATE_ERROR, 0);   //mower is switched into ERROR
    //robotIsStuckCounter = 0;
    }
    else if (errorCounter[ERR_STUCK] < 3) {   // mower tries 3 times to get unstuck
    if (stateCurr == STATE_FORWARD) {
    motorMowEnable = false;
    addErrorCounter(ERR_STUCK);
    setMotorPWM( 0, 0, false );
    reverseOrBidir(RIGHT);
    }
    else if (stateCurr == STATE_ROLL) {
    motorMowEnable = false;
    addErrorCounter(ERR_STUCK);
    setMotorPWM( 0, 0, false );
    setNextState (STATE_FORWARD, 0);
    }
    }
    }
    }
  */
}


void Robot::processGPSData()
{
  /*
    if (millis() < nextTimeGPS) return;
    nextTimeGPS = millis() + 1000;
    float nlat, nlon;
    unsigned long age;
    gps.f_get_position(&nlat, &nlon, &age);
    if (nlat == GPS::GPS_INVALID_F_ANGLE ) return;
    if (gpsLon == 0) {
    gpsLon = nlon;  // this is xy (0,0)
    gpsLat = nlat;
    return;
    }
    gpsX = (float)gps.distance_between(nlat,  gpsLon,  gpsLat, gpsLon);
    gpsY = (float)gps.distance_between(gpsLat, nlon,   gpsLat, gpsLon);
  */
}

// calculate map position by odometry sensors
void Robot::calcOdometry() {

  if ((millis() < nextTimeOdometry) || (stateCurr == STATE_OFF)) return;
  nextTimeOdometry = millis() + 100; //bb 300 at the original but test less
  static int lastOdoLeft = 0;
  static int lastOdoRight = 0;
  int odoLeft = odometryLeft;
  int odoRight = odometryRight;
  int ticksLeft = odoLeft - lastOdoLeft;
  int ticksRight = odoRight - lastOdoRight;
  lastOdoLeft = odoLeft;
  lastOdoRight = odoRight;
  double left_cm = ((double)ticksLeft) / ((double)odometryTicksPerCm);
  double right_cm = ((double)ticksRight) / ((double)odometryTicksPerCm);
  double avg_cm  = (left_cm + right_cm) / 2.0;
  double wheel_theta = (left_cm - right_cm) / ((double)odometryWheelBaseCm);
  //odometryTheta += wheel_theta;
  odometryTheta = scalePI(odometryTheta - wheel_theta);


  motorLeftRpmCurr  = double ((( ((double)ticksLeft) / ((double)odometryTicksPerRevolution)) / ((double)(millis() - lastMotorRpmTime))) * 60000.0);
  motorRightRpmCurr = double ((( ((double)ticksRight) / ((double)odometryTicksPerRevolution)) / ((double)(millis() - lastMotorRpmTime))) * 60000.0);
  lastMotorRpmTime = millis();
  if (stateCurr == STATE_PERI_TRACK)  totalDistDrive = totalDistDrive + int(avg_cm);
  currDistToDrive = currDistToDrive + int(avg_cm);
  if (imuUse) {
    odometryX += avg_cm * sin(prevYawCalcOdo);
    odometryY += avg_cm * cos(prevYawCalcOdo);
    prevYawCalcOdo = imu.ypr.yaw;

  } else {
    // FIXME: theta should be old theta, not new theta?
    odometryX += avg_cm * sin(odometryTheta);
    odometryY += avg_cm * cos(odometryTheta);
  }


}
void Robot::readDHT22() {
  //read only the temperature when no motor control.
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  if ((DHT22Use) && (millis() > nextTimeReadDHT22)) { //read only each 60 Secondes
    nextTimeReadDHT22 = nextTimeReadDHT22 + 60000;
    humidityDht = dht.readHumidity();
    temperatureDht = dht.readTemperature();
    if (temperatureDht >= maxTemperature) {
      Console.println("Temperature too high *************** Need to stop all the PCB in the next 2 minutes");
      Console.print("Maxi Setting = ");
      Console.print(maxTemperature);
      Console.print(" Actual Temperature = ");
      Console.println(temperatureDht);

      nextTimeReadDHT22 = nextTimeReadDHT22 + 180000; // do not read again the temp for the next 3 minute and set the idle bat to 2 minute to poweroff the PCB
      batSwitchOffIfIdle = 2; //use to switch off after 1 minute
      setNextState(STATE_ERROR, 0);
      return;
    }
    /*
      //to check if the 8 minutes overload can be caused by dht
      if (developerActive) {
      Console.print(" Read DHT22 temperature : ");
      Console.print(temperatureDht);
      Console.print("   Humidity : ");
      Console.println(humidityDht);
      }
    */
    if (isnan(humidityDht) || isnan(temperatureDht) ) {
      Console.println("Failed to read from DHT sensor!");
      humidityDht = 0.00;
      temperatureDht = 0.00;
    }
  }
}
void Robot::checkTimeout() {
  if (stateTime > motorForwTimeMax) {
    Console.println("Timeout on state the mower run for a too long duration ???????????????????????");
    setNextState(STATE_PERI_OUT_STOP, !rollDir); // toggle roll dir
  }
}






void Robot::loop()  {
  stateTime = millis() - stateStartTime;
  int steer;

  ADCMan.run();
  if (perimeterUse) perimeter.run();
  if (RaspberryPIUse) {
    MyRpi.run();
    if ((millis() > 60000) && (!MyrpiStatusSync)) { // on initial powerON DUE start faster than PI , so need to send again the status to refresh
      MyRpi.SendStatusToPi();
      MyrpiStatusSync = true;
    }
  }
  else {
    readSerial();
  }

 if (bluetoothUse || esp8266Use) {
    rc.readSerial();
  }
  readSensors();
  //checkIfStuck();
  checkRobotStats();
  checkPerimeterBoundary();
  calcOdometry();
  //checkOdometryFaults();
  checkButton();
  motorMowControl();
  checkTilt();
  if ((stateCurr == STATE_PERI_OUT_STOP) && (statusCurr == NORMAL_MOWING)) { //read only timer here for fast processing on odo
    checkTimer();
  }
  beeper();

  if ((stateCurr != STATE_STATION_CHARGING) || (stateCurr != STATE_STATION) || (stateCurr != STATE_PERI_TRACK)) {
    if ((imuUse) && (millis() >= nextTimeImuLoop)) {
      imu.run();
      nextTimeImuLoop = millis() + 50;
      /* Console.print(" Yaw ");
        Console.print(imu.ypr.yaw);
        Console.print(" Pitch ");
        Console.print(imu.ypr.pitch);
        Console.print(" Roll ");
        Console.println(imu.ypr.roll);
      */

    }

  }

  if (gpsUse && gpsReady && (millis() >= nextTimeGpsRead)) {
    nextTimeGpsRead = millis() + 1000;
    gps.run();
  }



  if (millis() >= nextTimeInfo) {
    if ((millis() - nextTimeInfo > 250)) {
      if (developerActive) {
        Console.print("------ LOOP NOT OK DUE IS OVERLOAD -- Over 1 sec ");
        Console.println((millis() - nextTimeInfo));
      }
    }
    nextTimeInfo = millis() + 1000; //1000
    printInfo(Console);
    checkErrorCounter();
    if (stateCurr == STATE_REMOTE) printRemote();
    loopsPerSec = loopsPerSecCounter;
    loopsPerSecCounter = 0;
  }

  if (millis() >= nextTimePfodLoop) {
    nextTimePfodLoop = millis() + 200;
    rc.run();
  }

  // state machine - things to do *PERMANENTLY* for current state
  // robot state machine

  switch (stateCurr) {

    case STATE_ERROR:
      // fatal-error
      checkBattery();
      if (millis() >= nextTimeErrorBeep) {
        nextTimeErrorBeep = millis() + 5000;
        setBeeper(600, 50, 50, 200, 0 );//error
      }
      motorControlOdo();
      break;

    case STATE_OFF:
      // robot is turned off
      if ((batMonitor) && (millis() - stateStartTime > 2000)) { //the charger is plug
        if (chgVoltage > 5.0)   {
          setNextState(STATE_STATION, 0);
          return;
        }
      }
      imuDriveHeading = imu.ypr.yaw / PI * 180;
      motorControlOdo();
      //bber13
      motorMowEnable = false; //to stop mow motor in OFF mode by pressing OFF again (the one shot OFF is bypass)
      checkSonar();  // only for test never use or the mower can't stay into the station
      readDHT22();
      checkBattery();


      break;

    case STATE_REMOTE:
      // remote control mode (RC)
      //if (remoteSwitch > 50) setNextState(STATE_FORWARD, 0);
      steer = ((double)motorSpeedMaxRpm / 2) * (((double)remoteSteer) / 100.0);
      if (remoteSpeed < 0) steer *= -1;
      motorLeftSpeedRpmSet  = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed) / 100.0) - steer;
      motorRightSpeedRpmSet = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed) / 100.0) + steer;
      motorLeftSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorLeftSpeedRpmSet));
      motorRightSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorRightSpeedRpmSet));
      motorMowSpeedPWMSet = ((double)motorMowSpeedMaxPwm) * (((double)remoteMow) / 100.0);
      motorControl();


      break;

    case STATE_MANUAL:
      checkCurrent();
      checkBumpers();
      checkDrop();
      motorControl();
      break;

    case STATE_FORWARD:
      // driving forward


      checkRain();
      checkCurrent();
      checkBumpers();
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      // checkSonar();
      checkLawn();
      checkTimeout();
      motorControl();
      break;







    case STATE_FORWARD_ODO:
      // driving forward with odometry control



      motorControlOdo();


      //manage the imu////////////////////////////////////////////////////////////
      if (imuUse ) {
        //when findedYaw = 999 it's mean that the lane is changed and the imu need to be adjusted to the compass
        if ((findedYaw == 999) && (imu.ypr.yaw > 0) && ((millis() - stateStartTime) > 4000) && ((millis() - stateStartTime) < 5000) && (mowPatternCurr == MOW_LANES)) { //try to find compass yaw
          setNextState(STATE_STOP_TO_FIND_YAW, rollDir);
          return;
        }

        //-----------here and before reverse the mower is stop so mark a pause to autocalibrate DMP-----------
        if ((millis() > nextTimeToDmpAutoCalibration) && (mowPatternCurr == MOW_LANES) && (imu.ypr.yaw > 0) && ((millis() - stateStartTime) > 4000) && ((millis() - stateStartTime) < 5000)  ) {
          setNextState(STATE_STOP_TO_FIND_YAW, rollDir);
          return;

        }
      }
      //-----------------------------------------------------------------------------
      ////////////////////////////////////////////////////////////////////////////

      //the normal state traitement alternatively the lenght is 300ml or 10 ml for example
      if ((odometryRight > stateEndOdometryRight) || (odometryLeft > stateEndOdometryLeft))
      {
        if ((mowPatternCurr == MOW_LANES) && (!justChangeLaneDir)) {
          Console.println("MAX LANE LENGHT TRIGGER time to reverse");
          setNextState(STATE_PERI_OUT_STOP, rollDir);
        }
        else {
          Console.println("more than 300 ML in straight line ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ? ");
          setBeeper(300, 150, 150, 160, 0);
          setNextState(STATE_PERI_OUT_STOP, rollDir);
        }
      }

      //-----------here need to start to mow in spirale or half lane lenght-----------
      if (highGrassDetect) {

        if ((mowPatternCurr == MOW_LANES)) {

          //if (halfLaneNb == 0) setNextState(STATE_ESCAPE_LANE, rollDir); //don't work need to check
          //if (halfLaneNb == 0) setNextState(STATE_STOP_BEFORE_SPIRALE, rollDir);
          setNextState(STATE_STOP_BEFORE_SPIRALE, rollDir);

        }
        else
        {
          setNextState(STATE_STOP_BEFORE_SPIRALE, rollDir);
        }
        return;
      }


      checkRain();
      checkCurrent();
      checkBumpers();
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      checkSonar();

      //checkLawn();
      checkTimeout();
      checkBattery();

      break;

    case STATE_ESCAPE_LANE:
      motorControlOdo();
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) setNextState(STATE_PERI_OUT_STOP, rollDir);
      checkCurrent();
      checkBumpers();
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      //checkSonar();

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t escape_lane in time ");
        }
        setNextState(STATE_PERI_OUT_STOP, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_ROLL_WAIT: //not use ??
      if ((odometryLeft >= stateEndOdometryLeft) || (odometryRight <= stateEndOdometryRight)) {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          Console.print(" OdometryLeft ");
          Console.print(odometryLeft);
          Console.print(" / stateEndOdometryLeft ");
          Console.print(stateEndOdometryLeft);
          Console.print(" OdometryRight ");
          Console.print(odometryRight);
          Console.print(" / stateEndOdometryRight ");
          Console.print(stateEndOdometryRight);
          Console.print(" yawtofind ");
          Console.println(findedYaw);
          Console.print(" odometry find the Opposit Yaw at ");
          Console.println((imu.ypr.yaw / PI * 180));
          setNextState(STATE_OFF, rollDir);
          //setNextState(STATE_FORWARD_ODO, rollDir);
        }
      }
      motorControlOdo();
      break;

    case STATE_CIRCLE:
      // not use
      motorControl();
      break;

    case STATE_PERI_OBSTACLE_REV:
      // perimeter tracking reverse for  x cm
      motorControlOdo();
      if ((odometryRight <= stateEndOdometryRight) || (odometryLeft <= stateEndOdometryLeft) )
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0) { //wait until the 2 motors completly stop
          setNextState(STATE_PERI_OBSTACLE_ROLL, RIGHT);
        }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t PERI_OBSTACLE_REV in time ");
        }
        setNextState(STATE_PERI_OBSTACLE_ROLL, RIGHT);
      }

      break;

    case STATE_PERI_OBSTACLE_ROLL:
      motorControlOdo();
      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0) { //wait until the 2 motors completly stop
          setNextState(STATE_PERI_OBSTACLE_FORW, 0);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t PERI_OBSTACLE_ROLL in time ");
        }
        setNextState(STATE_PERI_OBSTACLE_FORW, RIGHT);
      }
      checkCurrent();
      checkBumpersPerimeter();
      break;


    case STATE_PERI_OBSTACLE_FORW:
      //forward
      motorControlOdo();
      if ((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)) {

        setNextState(STATE_PERI_OBSTACLE_AVOID, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t PERI_OBSTACLE_FORW in time ");
        }
        setNextState(STATE_PERI_OBSTACLE_AVOID, RIGHT);
      }
      checkCurrent();
      checkBumpersPerimeter();
      break;


    case STATE_PERI_OBSTACLE_AVOID:
      //circle arc
      motorControlOdo();
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft)) {
        setNextState(STATE_PERI_FIND, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t PERI_OBSTACLE_AVOID in time ");
        }
        setNextState(STATE_PERI_FIND, 0);
      }
      checkCurrent();
      checkBumpersPerimeter();
      break;


    case STATE_REVERSE:
      motorControlOdo();
      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
        if (rollDir == RIGHT) {
          if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            setNextState(STATE_ROLL, rollDir);
          }
        }
        else
        {
          if (motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            setNextState(STATE_ROLL, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t reverse in time ");
        }
        setNextState(STATE_ROLL, rollDir);//if the motor can't rech the odocible in slope
      }


      break;

    case STATE_ROLL:
      motorControlOdo();
      if (rollDir == RIGHT) {
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
          if (motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            setNextState(STATE_FORWARD_ODO, rollDir);
          }
        }
      }
      else {
        if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
          if (motorLeftPWMCurr == 0 ) {
            setNextState(STATE_FORWARD_ODO, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t roll in time ");
        }
        setNextState(STATE_FORWARD_ODO, rollDir);
      }

      break;


    case STATE_ROLL_TONEXTTAG:
      motorControlOdo();

      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
        if (motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted

          setNextState(STATE_PERI_FIND, rollDir);
        }
      }


      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t roll in time ");
        }
        setNextState(STATE_PERI_FIND, rollDir);//if the motor can't rech the odocible in slope
      }

      break;

    case STATE_ROLL1_TO_NEWAREA:
      motorControlOdo();

      if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0) { //wait until the left motor completly stop because rotation is inverted
          setNextState(STATE_DRIVE1_TO_NEWAREA, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t roll in time ");
        }
        setNextState(STATE_DRIVE1_TO_NEWAREA, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_ROLL2_TO_NEWAREA:
      motorControlOdo();
      if (rollDir == RIGHT) {
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
          if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            setNextState(STATE_DRIVE2_TO_NEWAREA, rollDir);
          }
        }
      }
      else {
        if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
          if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0 ) {
            setNextState(STATE_DRIVE2_TO_NEWAREA, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t roll in time ");
        }
        setNextState(STATE_DRIVE2_TO_NEWAREA, rollDir);
      }
      break;

    case STATE_DRIVE1_TO_NEWAREA:
      motorControlOdo();
      if (currDistToDrive >= newtagDistance1) { // time to brake
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t DRIVE1_TO_NEWAREA in time ");
        }
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      break;

    case STATE_DRIVE2_TO_NEWAREA:
      motorControlOdo();
      if (currDistToDrive >= newtagDistance2) { // time to brake
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t DRIVE2_TO_NEWAREA in time ");
        }
        setNextState(STATE_STOP_TO_NEWAREA, rollDir);
      }
      break;



    case STATE_STOP_TO_NEWAREA:
      motorControlOdo();

      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (stateLast == STATE_DRIVE1_TO_NEWAREA) {  //2 possibility
            setNextState(STATE_ROLL2_TO_NEWAREA, rollDir);
          }
          else {
            setNextState(STATE_WAIT_FOR_SIG2, rollDir);
          }

        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t  stop ON BUMPER in time ");
        }
        if (stateLast == STATE_DRIVE1_TO_NEWAREA) {
          setNextState(STATE_ROLL2_TO_NEWAREA, rollDir);
        }
        else {
          setNextState(STATE_WAIT_FOR_SIG2, rollDir);
        }
      }


      break;

    case STATE_WAIT_FOR_SIG2:
      motorControlOdo();

      if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted

        if (millis() >= nextTimeReadSmoothPeriMag) {
          nextTimeReadSmoothPeriMag = millis() + 1000;
          smoothPeriMag = perimeter.getSmoothMagnitude(0);
          Console.print("SmoothMagnitude =  ");
          Console.println(smoothPeriMag);
          if ((perimeterInside) && (smoothPeriMag > 250)) //check if signal here and inside need a big value to be sure it is not only noise
          {
            if (areaToGo == 1) {
              statusCurr = BACK_TO_STATION; //if we are in the area1 it is to go to station
              periFindDriveHeading = imu.ypr.yaw;
            }
            else
            {
              areaInMowing = areaToGo;
              statusCurr = TRACK_TO_START;
            }
            if (RaspberryPIUse) MyRpi.SendStatusToPi();
            setNextState(STATE_PERI_FIND, rollDir);
            return;
          }
        }



      }
      if (millis() > (stateStartTime + 180000)) {  //wait the signal for 3 minutes
        Console.println ("Warning can t find the signal for area2 ");
        setNextState(STATE_ERROR, rollDir);
      }

      break;


    case STATE_TEST_COMPASS:
      motorControlOdo();

      YawActualDeg = (imu.ypr.yaw / PI * 180);

      if ((imu.distance180(YawActualDeg, yawToFind)) < 30) { //reduce speed to be sure stop
        PwmLeftSpeed = SpeedOdoMin / 2;
        PwmRightSpeed = -SpeedOdoMin / 2;
      }
      else {
        PwmLeftSpeed = SpeedOdoMin;
        PwmRightSpeed = -SpeedOdoMin;
      }


      if ((YawActualDeg >= yawToFind - 1) && (YawActualDeg <= yawToFind + 1))  {
        Console.print(" OdometryLeft ");
        Console.print(odometryLeft);
        Console.print(" OdometryRight ");
        Console.print(odometryRight);
        Console.print(" Find YAW ****************************************  ");
        Console.println((imu.ypr.yaw / PI * 180));
        setNextState(STATE_OFF, rollDir);

      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        Console.println ("Warning can t TestCompass in time ");
        setNextState(STATE_OFF, rollDir);
      }

      break;

    case STATE_CALIB_MOTOR_SPEED:
      motorControlOdo();
      if ((motorRightPWMCurr == 0 ) && (motorLeftPWMCurr == 0 )) {
        Console.println("Calibration finish ");
        Console.print("Real State Duration : ");
        Tempovar = millis() - stateStartTime;
        Console.println(Tempovar);
        Console.print("Compute Max State Duration : ");
        Console.println(MaxOdoStateDuration);
        motorTickPerSecond = 1000 * stateEndOdometryRight / Tempovar;

        Console.print(" motorTickPerSecond ");
        Console.println(motorTickPerSecond);
        setNextState(STATE_OFF, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        Console.println ("Warning can t TestMotor in time please check your Odometry or speed setting ");
        setNextState(STATE_OFF, rollDir);
      }

      break;

    case STATE_TEST_MOTOR:
      motorControlOdo();
      if ((motorRightPWMCurr == 0 ) && (motorLeftPWMCurr == 0 )) {
        Console.println("Test finish ");
        Console.print("Real State Duration : ");
        Console.println(millis() - stateStartTime);
        Console.print("Compute Max State Duration : ");
        Console.println(MaxOdoStateDuration);
        setNextState(STATE_OFF, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        Console.println ("Warning can t TestMotor in time please check your Odometry or speed setting ");
        setNextState(STATE_OFF, rollDir);
      }
      break;




    case STATE_ROLL_TO_FIND_YAW:
      boolean finish_4rev;
      finish_4rev = false;
      motorControlOdo();
      imu.run();
      //it's ok
      if (CompassUse) {
        if ((yawToFind - 2 < (imu.comYaw / PI * 180)) && (yawToFind + 2 > (imu.comYaw / PI * 180)))  { //at +-2 degres
          findedYaw = (imu.comYaw / PI * 180);
          setNextState(STATE_STOP_CALIBRATE, rollDir);
          return;
        }
      }
      else //without compass
      {
        if ((yawToFind - 2 < (imu.ypr.yaw / PI * 180)) && (yawToFind + 2 > (imu.ypr.yaw / PI * 180)))  { //at +-2 degres
          findedYaw = (imu.ypr.yaw / PI * 180);
          setNextState(STATE_STOP_CALIBRATE, rollDir);
          return;
        }
      }


      //it's not ok
      if ((actualRollDirToCalibrate == RIGHT) && ((odometryRight <= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft))) finish_4rev = true;
      if ((actualRollDirToCalibrate == LEFT) && ((odometryRight >= stateEndOdometryRight) || (odometryLeft <= stateEndOdometryLeft))) finish_4rev = true;
      if (millis() > (stateStartTime + MaxOdoStateDuration + 6000)) finish_4rev = true;
      if (finish_4rev == true) {
        if (developerActive) {
          Console.println ("Warning can t roll to find yaw The Compass is certainly not calibrate correctly ");
          Console.println ("Continue to mow in random mode without compass ");
        }
        if (stopMotorDuringCalib) motorMowEnable = true;//restart the mow motor
        endTimeCalibration = millis();
        compassYawMedian.clear();
        accelGyroYawMedian.clear();
        mowPatternCurr = MOW_RANDOM;
        findedYaw = yawToFind;
        nextTimeToDmpAutoCalibration = millis() + 21600 * 1000; //do not try to calibration for the next 6 hours
        setBeeper(0, 0, 0, 0, 0);
        if (perimeterInside) setNextState(STATE_ACCEL_FRWRD, rollDir);
        else setNextState(STATE_PERI_OUT_REV, rollDir);
        return;
      }


      break;

    //not use actually
    case STATE_PERI_ROLL:
      // perimeter find  roll
      if (millis() >= stateEndTime) setNextState(STATE_PERI_FIND, 0);
      motorControl();
      break;


    //not use actually
    case STATE_PERI_REV:  //obstacle in perifind
      // perimeter tracking reverse
      //bb
      Console.println(odometryRight);

      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft))  setNextState(STATE_PERI_ROLL, rollDir);
      motorControlOdo();


      break;

    case STATE_PERI_FIND:
      // find perimeter
      if (!perimeterInside) {
        Console.println("Not inside so start to track the wire");
        setNextState(STATE_PERI_STOP_TOTRACK, 0);
        return;
      }

      checkSonar();
      checkBumpersPerimeter();
      checkCurrent();
      motorControlOdo();
      break;

    case STATE_PERI_TRACK:
      // track perimeter
      checkCurrent();
      checkBumpersPerimeter();
      checkSonarPeriTrack();

      //bber50
      if (ActualSpeedPeriPWM != MaxSpeedperiPwm) {
        if (totalDistDrive > whereToResetSpeed) {
          Console.print("Distance OK, time to reset the initial Speed : ");
          Console.println(ActualSpeedPeriPWM);
          ActualSpeedPeriPWM = MaxSpeedperiPwm;
        }
      }

      //********************************* if start by timer
      if (statusCurr == TRACK_TO_START) {
        //bber11
        //areaToGo need to be use here to avoid start mowing before reach the rfid tag in area1
        // if ((areaToGo == areaInMowing) && (startByTimer) && (totalDistDrive > whereToStart * 100)) {
        //bber35

        if ((areaToGo == areaInMowing) && (totalDistDrive >= whereToStart * 100)) {
          startByTimer = false;
          Console.print("Distance OK, time to start mowing into new area ");
          Console.println(areaInMowing);
          areaToGo = 1; //after mowing the mower need to back to station
          ActualSpeedPeriPWM = MaxSpeedperiPwm;
          setNextState(STATE_PERI_STOP_TOROLL, rollDir);
          return;
        }


      }


      motorControlPerimeter();
      break;

    case STATE_STATION:
      // waiting until auto-start by user or timer triggered
      if (batMonitor) {
        if (chgVoltage > 5.0) {
          if (batVoltage < startChargingIfBelow) { //read the battery voltage immediatly before it increase
            setNextState(STATE_STATION_CHARGING, 0);
            return;
          }
          else
          {
            if (millis() - stateStartTime > 10000) checkTimer(); //only check timer after 10 second to avoid restart before charging and check non stop after but real only 60 sec
          }
        }
        else
        {
          Console.println("We are in station but ChargeVoltage is lost ??? ");
          setNextState(STATE_OFF, 0);
          return;
        }
      }
      else {
        if (millis() - stateStartTime > 10000) checkTimer(); //only check timer after 10 second to avoid restart before charging
      }
      readDHT22();
      break;

    case STATE_STATION_CHARGING:
      // waiting until charging completed
      if (batMonitor) {
        if ((chgCurrent < batFullCurrent) && (millis() - stateStartTime > 2000)) {
          if ((autoResetActive) && (millis() - stateStartTime > 3600000)) { // only reboot if the mower is charging for more 1 hour
            Console.println("End of charge by batfullcurrent Time to Restart PI and Due");
            autoReboot();
          }
          setNextState(STATE_STATION, 0);
          return;
        }
        if (millis() - stateStartTime > chargingTimeout)
        {
          Console.println("End of charging duration check the batfullCurrent to try to stop before");
          if (autoResetActive) {
            Console.println("Time to Restart PI and Due");
            autoReboot();
          }


          setNextState(STATE_STATION, 0);
          return;
        }
      }
      readDHT22();

      break;

    case STATE_STOP_ON_BUMPER:
      motorControlOdo();

      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (statusCurr == BACK_TO_STATION) {
            setNextState(STATE_PERI_OBSTACLE_REV, rollDir);
          }
          else {
            setNextState(STATE_PERI_OUT_REV, rollDir);
          }
          return;

        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t  stop ON BUMPER in time ");
        }
        setNextState(STATE_PERI_OUT_REV, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_PERI_OUT_STOP:
      motorControlOdo();
      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_PERI_OUT_REV, rollDir);
        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t peri out stop in time ");
        }
        setNextState(STATE_PERI_OUT_REV, rollDir);//if the motor can't rech the odocible in slope
      }
      break;


    case STATE_SONAR_TRIG:
      motorControlOdo();
      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft))) {
        setBeeper(0, 0, 0, 0, 0);

        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          //bber10
          if (stateLast == STATE_PERI_FIND) {
            setNextState(STATE_PERI_OBSTACLE_REV, rollDir);
          }
          else {
            setNextState(STATE_PERI_OUT_REV, rollDir);
          }
          return;

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t sonar trig in time ");
        }
        if (stateCurr == STATE_PERI_FIND) {
          setNextState(STATE_PERI_OBSTACLE_REV, rollDir);
        }
        else {
          setNextState(STATE_PERI_OUT_REV, rollDir);
        }
        return;
      }
      checkCurrent();
      checkBumpers();
      break;


    case STATE_STOP_TO_FIND_YAW:
      motorControlOdo();
      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (laneUseNr == 1) yawToFind = yawSet1 ;
          if (laneUseNr == 2) yawToFind = yawSet2 ;
          if (laneUseNr == 3) yawToFind = yawSet3 ;
          if (CompassUse) {
            setNextState(STATE_ROLL_TO_FIND_YAW, rollDir);
          }
          else
          {
            findedYaw = (imu.ypr.yaw / PI * 180);
            setNextState(STATE_STOP_CALIBRATE, rollDir);
          }
        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t peri out stop in time ");
        }
        if (laneUseNr == 1) yawToFind = yawSet1 ;
        if (laneUseNr == 2) yawToFind = yawSet2 ;
        if (laneUseNr == 3) yawToFind = yawSet3 ;
        if (CompassUse) {
          setNextState(STATE_ROLL_TO_FIND_YAW, rollDir);//if the motor can't rech the odocible in slope
        }
        else
        {
          findedYaw = (imu.ypr.yaw / PI * 180);
          setNextState(STATE_STOP_CALIBRATE, rollDir);
        }
      }
      break;

    case STATE_PERI_STOP_TOROLL:
      motorControlOdo();

      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          if (statusCurr == TRACK_TO_START) setNextState(STATE_STATION_ROLL, rollDir);
          else setNextState(STATE_ROLL_TONEXTTAG, rollDir);
        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t stop to track in time ");
        }
        if (statusCurr == TRACK_TO_START) setNextState(STATE_STATION_ROLL, rollDir);
        else setNextState(STATE_ROLL_TONEXTTAG, rollDir);
      }
      break;

    case STATE_PERI_STOP_TO_FAST_START:
      motorControlOdo();
      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_ROLL_TONEXTTAG, rollDir);
        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t stop to track in time ");
        }
        setNextState(STATE_ROLL_TONEXTTAG, rollDir);

      }
      break;

    case STATE_PERI_STOP_TO_NEWAREA:
      motorControlOdo();

      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_ROLL1_TO_NEWAREA, rollDir);
        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t stop  in time ");
        }

        setNextState(STATE_ROLL1_TO_NEWAREA, rollDir);
      }
      break;

    case STATE_PERI_STOP_TOTRACK:
      motorControlOdo();
      if (((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)))
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);
        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t stop to track in time ");
        }
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_AUTO_CALIBRATE:
      setBeeper(2000, 150, 150, 160, 50);
      if (millis() > nextTimeAddYawMedian) {  // compute a median of accelGyro and Compass  yaw
        compassYawMedian.add(imu.comYaw);
        accelGyroYawMedian.add(imu.ypr.yaw);
        nextTimeAddYawMedian = millis() + 70;  // the value are read each 70ms
      }
      if (accelGyroYawMedian.getCount() > 56) { //we have the value of 4 secondes try to verify if the drift is less than x deg/sec
        Console.println("4 sec of read value, verify if the drift is stop");
        if  (abs(accelGyroYawMedian.getHighest() - accelGyroYawMedian.getLowest()) < 4 * maxDriftPerSecond * PI / 180) { //drift is OK restart mowing
          if (CompassUse) {
            imu.CompassGyroOffset = distancePI( scalePI(accelGyroYawMedian.getMedian() -  imu.CompassGyroOffset), compassYawMedian.getMedian()); //change the Gyro offset according to Compass Yaw
          }
          else
          {
            imu.CompassGyroOffset = 0;
          }
          Console.println("Drift is OK");
          setBeeper(0, 0, 0, 0, 0); //stop sound immediatly

          if (stopMotorDuringCalib) motorMowEnable = true;//restart the mow motor
          if (perimeterInside) {
            setNextState(STATE_ACCEL_FRWRD, rollDir); //if not outside continue in forward
          }
          else
          {
            setNextState(STATE_PERI_OUT_REV, rollDir);
          }
          return;
        }
        else {   //not OK try to wait 4 secondes more
          Console.println("Drift not Stop wait again 4 sec");
          compassYawMedian.clear();
          accelGyroYawMedian.clear();
        }

      }
      if (millis() > endTimeCalibration) { //we have wait enought and the result is not OK start to mow in random mode or make a total calibration
        mowPatternCurr == MOW_RANDOM;
        if (stopMotorDuringCalib) motorMowEnable = true;//stop the mow motor
        Console.println("WAIT to stop Drift of GYRO : is not OK mowing Drift too important");
        nextTimeToDmpAutoCalibration = millis() + delayBetweenTwoDmpAutocalib * 1000;
        setBeeper(0, 0, 0, 0, 0);
        if (perimeterInside) {
          if (mowPatternCurr == MOW_LANES) {  //change the rolldir now because again when new forward_odo only in lane mowing
            // if (rollDir == 0) rollDir = 1;
            // else rollDir = 0;
          }
          setNextState(STATE_ACCEL_FRWRD, rollDir);
        }
        else
        {
          setNextState(STATE_PERI_OUT_REV, rollDir);
        }


      }

      break;

    case STATE_STOP_CALIBRATE:
      motorControlOdo();
      if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
        setNextState(STATE_AUTO_CALIBRATE, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t  stop to calibrate in time ");
        }
        setNextState(STATE_AUTO_CALIBRATE, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_STOP_BEFORE_SPIRALE:
      motorControlOdo();
      if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
        setNextState(STATE_ROTATE_RIGHT_360, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning cant stop before spirale in time");
        }
        setNextState(STATE_ROTATE_RIGHT_360, rollDir);    //if the motor can't rech the odocible in slope
      }
      break;

    case STATE_ROTATE_RIGHT_360:
      motorControlOdo();
      checkCurrent();
      if ((odometryRight <= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        if (motorLeftPWMCurr == 0 && motorRightPWMCurr == 0)  { //wait until the 2 motors completly stop because rotation is inverted
          setNextState(STATE_MOW_SPIRALE, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning cant rotate right 360 in time ");
        }
        setNextState(STATE_MOW_SPIRALE, rollDir);
      }

      break;
    case STATE_NEXT_SPIRE:
      motorControlOdo();
      checkCurrent();
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        setNextState(STATE_MOW_SPIRALE, rollDir);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t  stop before next spire in time ");
        }
        setNextState(STATE_MOW_SPIRALE, rollDir);//if the motor can't rech the odocible in slope
      }

      break;
    case STATE_MOW_SPIRALE:
      motorControlOdo();
      checkCurrent();
      checkBumpers();
      //checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      checkSonar();

      //checkLawn();
      checkTimeout();

      //*************************************end of the spirale ***********************************************
      if (spiraleNbTurn >= 8) {
        spiraleNbTurn = 0;
        highGrassDetect = false;
        setNextState(STATE_PERI_OUT_STOP, RIGHT); //stop the spirale or setNextState(STATE_PERI_OUT_FORW, rollDir)
        return;
      }
      //********************************************************************************************
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        if (!perimeterInside) {
          setNextState(STATE_PERI_OUT_STOP, rollDir);
        }
        else
        {
          setNextState(STATE_NEXT_SPIRE, rollDir);
        }
        return;
      }


      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t MOW_SPIRALE in time ");
        }
        setNextState(STATE_NEXT_SPIRE, rollDir);//if the motor can't rech the odocible in slope
      }


      break;

    case STATE_PERI_OUT_REV:
      motorControlOdo();

      if (mowPatternCurr == MOW_LANES) {  //  *************************LANE***************************************
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
          if (rollDir == RIGHT) {
            if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) { //wait until the 2 motor completly stop because need precision
              setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
            }
          }
          else
          {
            if ((motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0)) {
              setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);
            }
          }
        }
      }
      else
      { //  *************************RANDOM***************************************
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
          if (rollDir == RIGHT) {
            if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted

              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
          else
          {
            if (motorRightPWMCurr == 0 ) { //wait until the right motor completly stop because rotation is inverted
              setNextState(STATE_PERI_OUT_ROLL, rollDir);
            }
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t peri out rev in time ");
        }
        setNextState(STATE_PERI_OUT_LANE_ROLL1, rollDir);//if the motor can't rech the odocible in slope
      }

      break;

    case STATE_PERI_OUT_ROLL:
      motorControlOdo();
      if (rollDir == RIGHT) {
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
          if (motorRightPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_PERI_OUT_FORW, rollDir);
          }
        }
      }
      else {
        if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
          if (motorLeftPWMCurr == 0 ) { //wait until the left motor completly stop because rotation is inverted
            if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_PERI_OUT_FORW, rollDir);
          }

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t peri out roll in time ");
        }
        setNextState(STATE_PERI_OUT_FORW, rollDir);//if the motor can't rech the odocible in slope
      }


      break;

    case STATE_PERI_OUT_ROLL_TOINSIDE:
      motorControlOdo();
      //bber17
      if (RollToInsideQty >= 10) {
        Console.println("ERROR Mower is lost out the wire and can't find the signal. Roll to inside occur more than 10 Time");
        setNextState(STATE_ERROR, rollDir);
        return;
      }

      if (rollDir == RIGHT) {
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
            if (!perimeterInside) setNextState(STATE_WAIT_AND_REPEAT, rollDir);//again until find the inside
            else setNextState(STATE_PERI_OUT_FORW, rollDir);
          }
        }
      }
      else {
        if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft) ) {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 ) ) { //wait until the 2 motor completly stop
            if (!perimeterInside) setNextState(STATE_WAIT_AND_REPEAT, rollDir);//again until find the inside
            else setNextState(STATE_PERI_OUT_FORW, rollDir);
          }

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t Roll to inside in time ");
        }
        if (!perimeterInside) setNextState(STATE_WAIT_AND_REPEAT, rollDir);//again until find the inside
        else setNextState(STATE_PERI_OUT_FORW, rollDir);
      }
      break;

    case STATE_PERI_OUT_ROLL_TOTRACK:
      motorControlOdo();

      if (perimeterInside) {
        setNextState(STATE_PERI_OUT_STOP_ROLL_TOTRACK, 0);
        return;
      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t find perimeter Wire while PERI_OUT_ROLL_TOTRACK in time ");
        }
        if (!perimeterInside) setNextState(STATE_WAIT_AND_REPEAT, 0);//again until find the inside
        else setNextState(STATE_PERI_OUT_STOP_ROLL_TOTRACK, 0);;
      }
      break;

    case STATE_PERI_OUT_STOP_ROLL_TOTRACK:
      motorControlOdo();

      if (perimeterInside) {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) {
          lastTimeForgetWire = millis(); //avoid motor reverse on tracking startup
          setNextState(STATE_PERI_TRACK, 0);
          return;
        }
      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t PERI_OUT_STOP_ROLL_TOTRACK in time ");
        }
        if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOTRACK, 0);//again until find the inside
        else setNextState(STATE_PERI_TRACK, 0);
      }
      break;

    case STATE_PERI_OUT_LANE_ROLL1:
      motorControlOdo();
      checkCurrent();
      checkBumpers();

      if (rollDir == RIGHT) {
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft))
        {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the left motor completly stop because rotation is inverted
            if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_NEXT_LANE_FORW, rollDir);
          }
        }
      }
      else
      {
        if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft))
        {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the left motor completly stop because rotation is inverted
            if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_NEXT_LANE_FORW, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t Roll1 by lane in time ");
        }
        setNextState(STATE_NEXT_LANE_FORW, rollDir);//if the motor can't reach the odocible in slope
      }
      break;

    case STATE_NEXT_LANE_FORW:
      motorControlOdo();
      checkCurrent();
      checkBumpers();

      //bber14
      //if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
      if (!perimeterInside) {
        setNextState(STATE_PERI_OUT_STOP, rollDir);
        return;
      }

      if ((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft) ) {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) {
          setNextState(STATE_PERI_OUT_LANE_ROLL2, rollDir);

        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          Console.println ("Warning can t reach next lane in time ");
        }
        setNextState(STATE_PERI_OUT_LANE_ROLL2, rollDir);//if the motor can't reach the odocible in slope for example

      }

      break;

    case STATE_PERI_OUT_LANE_ROLL2:
      motorControlOdo();
      checkCurrent();
      checkBumpers();

      if (rollDir == RIGHT) {
        if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft))
        {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
            if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_FORWARD_ODO, rollDir);// forward odo to straight line
            rollDir = LEFT;//invert the next rotate
          }
        }
      }

      else
      {
        if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft))
        {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
            if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            else setNextState(STATE_FORWARD_ODO, rollDir);
            rollDir = RIGHT;// invert the next rotate
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          Console.println ("Warning can t make the roll2 in time ");
        }
        if (rollDir == RIGHT) {
          if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
          else setNextState(STATE_FORWARD_ODO, rollDir);// forward odo to straight line
          rollDir = LEFT;//invert the next rotate
        }
        else
        {
          if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
          else setNextState(STATE_FORWARD_ODO, rollDir);
          rollDir = RIGHT;// invert the next rotate
        }
      }
      break;




    case STATE_PERI_OUT_FORW:
      motorControlOdo();
      if (!perimeterInside) setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
      if ((millis() > (stateStartTime + MaxOdoStateDuration)) || (odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        setNextState(STATE_FORWARD_ODO, rollDir);

      }
      break;

    case STATE_STATION_CHECK:

      // check for charging voltage here after detect station
      if ((odometryRight >= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft)) //move some CM to be sure the contact is OK
      {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          //need to adapt if station is traversante
          if (millis() >= delayToReadVoltageStation) { //wait 1.5 sec after all stop and before read voltage
            //bber30
            nextTimeBattery = millis();
            readSensors();  //read the chgVoltage immediatly
            if (chgVoltage > 5.0)  {
              Console.println ("Charge Voltage detected ");
              setNextState(STATE_STATION, rollDir);// we are into the station
              return;
            }
            else {
              Console.println ("No Voltage detected so certainly Obstacle ");
              setNextState(STATE_PERI_OBSTACLE_REV, rollDir);// not into the station so avoid obstacle
              return;
            }
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          Console.println ("Warning can t make the station check in time ");
        }
        if (millis() >= delayToReadVoltageStation) {
          nextTimeBattery = millis();
          readSensors();  //read the chgVoltage
          if (chgVoltage > 5.0) {
            setNextState(STATE_STATION, rollDir);// we are into the station
            return;
          }
          else {
            setNextState(STATE_PERI_OBSTACLE_REV, rollDir);// not into the station so avoid obstacle
            return;
          }
        }
      }
      motorControlOdo();
      break;

    case STATE_STATION_REV:

      motorControlOdo();
      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft))
      {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          setNextState(STATE_STATION_ROLL, 1);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          Console.print ("Warning station rev not in time Max Compute duration in ms :");
        }
        setNextState(STATE_STATION_ROLL, 1);//if the motor can't reach the odocible in slope
      }
      break;

    case STATE_STATION_ROLL:
      motorControlOdo();
      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft >= stateEndOdometryLeft))
      {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          setNextState(STATE_STATION_FORW, rollDir);
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          Console.println ("Warning can t make the station roll in time ");
        }
        setNextState(STATE_STATION_FORW, rollDir);//if the motor can't reach the odocible in slope
      }
      break;

    case STATE_STATION_FORW:
      // forward (charge station)
      //disabble the sonar during 10 seconds

      nextTimeCheckSonar = millis() + 10000;  //Do not check the sonar during 30 second  to avoid detect the station


      //justChangeLaneDir=false;
      motorControlOdo();

      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft))
      {
        if ((whereToStart != 0) && (startByTimer)) { //if ((whereToStart != 0) make a circle arround the station if not start immediatly
          setNextState(STATE_PERI_OBSTACLE_AVOID, rollDir);
        }
        else
        {
          //020919 to check but never call and not sure it's ok
          statusCurr = NORMAL_MOWING;
          if (RaspberryPIUse) MyRpi.SendStatusToPi();
          setNextState(STATE_FORWARD_ODO, rollDir);
        }

      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          Console.println ("Warning can t make the station forw in time ");
        }
        if ((whereToStart != 0) && (startByTimer)) {
          setNextState(STATE_PERI_OBSTACLE_AVOID, rollDir);
        }
        else  setNextState(STATE_FORWARD_ODO, rollDir);
      }
      break;

    case STATE_WAIT_AND_REPEAT:
      if (millis() > (stateStartTime + 500)) setNextState(stateLast, rollDir);//1000
      break;

    //bber50
    case STATE_ACCEL_FRWRD:
      
      motorControlOdo();
      if (!perimeterInside) {
        Console.println("Try to start at other location : We are not inside perimeter");
        setNextState(STATE_PERI_OUT_STOP, rollDir);
        return;
      }
      if ((millis() > (stateStartTime + MaxOdoStateDuration)) || (odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        imuDirPID.reset();
        motorRightPID.reset();
        motorLeftPID.reset();
        setNextState(STATE_FORWARD_ODO, rollDir);
      }

      break;


  } // end switch

  bumperRight = false;
  bumperLeft = false;
  dropRight = false;                                                                                                                             // Dropsensor - Absturzsensor
  dropLeft = false;                                                                                                                              // Dropsensor - Absturzsensor

  loopsPerSecCounter++;
  watchdogReset();
  //perimeter.speedTest();
  /*
    StartReadAt = millis();
    distance_find = sensor.readRangeSingleMillimeters();
    EndReadAt = millis();
    ReadDuration = EndReadAt - StartReadAt;
    Console.print("Dist :    ");
    Console.print(distance_find);
    Console.print("         Read Duration in ms ");
    Console.println(ReadDuration);
  */

}
