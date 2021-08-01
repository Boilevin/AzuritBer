/*
  Private-use only! (you need to ask for a commercial-use)
*/



#include "robot.h"
#include "mower.h"

//#define Console Serial
//#define Bluetooth Serial1  //  for ESP32 communication

#define MAGIC 52  //value 52 is only use to know if the eeprom is OK : 52 is save and read at first byte of memory location
#define ADDR_USER_SETTINGS 2000 //New adress to avoid issue if Azurit1.09 is already instaled
#define ADDR_ERR_COUNTERS 500 //same adress as azurit
//carrefull that the  ADDR 600 is used by the IMU calibration
#define ADDR_ROBOT_STATS 800


const char* stateNames[] = {"OFF ", "RC  ", "FORW", "ROLL", "REV ", "CIRC", "ERR ", "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK", "STREV",
                            "STROL", "STFOR", "MANU", "ROLW", "POUTFOR", "POUTREV", "POUTROLL", "POBSREV", "POBSROLL", "POBSFRWD", "POBSCIRC", "NEXTLANE", "POUTSTOP", "LANEROL1", "LANEROL2",
                            "ROLLTOIN", "WAITREPEAT", "FRWODO", "TESTCOMPAS", "ROLLTOTRACK",
                            "STOPTOTRACK", "AUTOCALIB", "ROLLTOFINDYAW", "TESTMOTOR", "FINDYAWSTOP", "STOPONBUMPER",
                            "STOPCALIB", "SONARTRIG", "STOPSPIRAL", "MOWSPIRAL", "ROT360", "NEXTSPIRE", "ESCAPLANE",
                            "TRACKSTOP", "ROLLTOTAG", "STOPTONEWAREA", "ROLL1TONEWAREA", "DRIVE1TONEWAREA", "ROLL2TONEWAREA", "DRIVE2TONEWAREA", "WAITSIG2", "STOPTONEWAREA", "ROLLSTOPTOTRACK",
                            "STOPTOFASTSTART", "CALIBMOTORSPEED", "ACCELFRWRD"
                           };

const char* statusNames[] = {"WAIT", "NORMALMOWING", "SPIRALEMOWING", "BACKTOSTATION", "TRACKTOSTART", "MANUAL", "REMOTE", "ERROR", "STATION", "TESTING", "SIGWAIT" , "WIREMOWING"
                            };


const char* mowPatternNames[] = {"RAND", "LANE",  "WIRE" , "ZIGZAG"};
const char* consoleModeNames[] = {"sen_counters", "sen_values", "perimeter", "off", "Tracking"};


unsigned long StartReadAt;
unsigned long EndReadAt;
unsigned long ReadDuration;



Robot::Robot() {

  name = "Generic";
  developerActive = false;
  rc.setRobot(this);
  //MyRpi.setRobot(this);

  stateLast = stateCurr = stateNext = STATE_OFF;
  statusCurr = WAIT; //initialise the status on power up


  stateLast = stateCurr = stateNext = STATE_OFF;
  statusCurr = WAIT; //initialise the status on power up

  stateTime = 0;
  idleTimeSec = 0;
  statsMowTimeTotalStart = false;


  odometryLeft = odometryRight = 0;
  PeriOdoIslandDiff = 0;
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
  motorRpmCoeff = 1;

  remoteSteer = remoteSpeed = remoteMow = remoteSwitch = 0;
  remoteSteerLastTime = remoteSpeedLastTime = remoteMowLastTime = remoteSwitchLastTime = 0;
  remoteSteerLastState = remoteSpeedLastState = remoteMowLastState = remoteSwitchLastState = LOW;

  motorMowRpmCounter = 0;
  motorMowRpmLastState = LOW;
  motorMowEnable = false;
  motorMowForceOff = false;
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
  perimeterSpeedCoeff = 1;

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
  nextTimeScreen = 0;
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

  MyrpiStatusSync = false;
  ConsoleToPfod = false;
}


const char* Robot::stateName() {
  return stateNames[stateCurr];
}
const char* Robot::statusName() {
  return statusNames[statusCurr];
}


const char* Robot::mowPatternName() {
  return mowPatternNames[mowPatternCurr];
}

const char* Robot::mowPatternNameList(byte mowPatternIndex) {
  return mowPatternNames[mowPatternIndex];
}


void Robot::setDefaultTime() {
  datetime.time.hour = 12;
  datetime.time.minute = 0;
  datetime.date.dayOfWeek = 0;
  datetime.date.day = 1;
  datetime.date.month = 1;
  datetime.date.year = 2013;
  timer[0].active = false;
  timer[0].daysOfWeek = B01111110;
  timer[0].startTime.hour = 9;
  timer[0].stopTime.hour = 11;
}



// check timer
void Robot::checkTimer() {

  if (millis() < nextTimeTimer) return;
  nextTimeTimer = millis() + 60000;  // one minute check
  srand(time2minutes(datetime.time)); // initializes the pseudo-random number generator for c++ rand()
  randomSeed(time2minutes(datetime.time)); // initializes the pseudo-random number generator for arduino random()
  boolean stopTimerTriggered = true;
  if (timerUse) {
    Console.println("checktimer");
    for (int i = 0; i < MAX_TIMERS; i++) {
      if (timer[i].active) {
        if  ( (timer[i].daysOfWeek & (1 << datetime.date.dayOfWeek)) != 0) {
          int startmin = time2minutes(timer[i].startTime);
          int stopmin =  time2minutes(timer[i].stopTime);
          int currmin =  time2minutes(datetime.time);

          Console.print("Timer ");
          Console.print(i);
          Console.print(" startmin ");
          Console.print(startmin);

          Console.print(" stopmin ");
          Console.print(stopmin);

          Console.print(" currmin ");
          Console.println(currmin);

          if ((currmin >= startmin) && (currmin < stopmin)) {
            // start timer triggered
            stopTimerTriggered = false;
            if ((stateCurr == STATE_STATION)) {
              Console.print("Timer ");
              Console.print(i);
              Console.println(F(" start triggered"));
              ActualRunningTimer = i;
              //motorMowEnable = true;
              findedYaw = 999;
              imuDirPID.reset();
              mowPatternCurr = timer[i].startMowPattern;
              laneUseNr = timer[i].startNrLane;
              rollDir = timer[i].startRollDir;
              whereToStart = timer[i].startDistance;
              areaToGo = timer[i].startArea;
              actualLenghtByLane = timer[i].startLaneMaxlengh;
              beaconToStart = timer[i].rfidBeacon;
              startByTimer = true;
              mowPatternDuration = 0;
              totalDistDrive = 0;
              Console.print(F(" Track for area "));
              Console.println(areaToGo);
              Console.print(F(" Distance before start "));
              Console.println(whereToStart);


              setNextState(STATE_STATION_REV, 0);

            }
          }
        }
        if ((stateCurr != STATE_STATION) && (stopTimerTriggered) && (ActualRunningTimer == i)) { //Stop only the running timer

          Console.println(F("timer stop triggered"));
          ActualRunningTimer = 99;
          if (perimeterUse) {
            setNextState(STATE_PERI_FIND, 0);
          }
          else {
            setNextState(STATE_OFF, 0);
          }
        }

      }
    }
  }
}

void Robot::loadSaveRobotStats(boolean readflag) {
  /*
    int addr = ADDR_ROBOT_STATS;

    if (readflag) {
     ShowMessageln(F("Load Robot Stats"));
    }
    else {
     ShowMessageln(F("Save Robot Stats"));
    }

    short magic = 0;
    if (!readflag) magic = MAGIC;
    eereadwrite(readflag, addr, magic); // magic
    if ((readflag) && (magic != MAGIC)) {
     ShowMessageln(F("PLEASE CHECK IF YOUR ROBOT STATS ARE CORRECT"));
    }
    eereadwrite(readflag, addr, statsMowTimeMinutesTrip);
    eereadwrite(readflag, addr, statsMowTimeMinutesTotal);
    eereadwrite(readflag, addr, statsBatteryChargingCounterTotal);
    eereadwrite(readflag, addr, statsBatteryChargingCapacityTrip);
    eereadwrite(readflag, addr, statsBatteryChargingCapacityTotal);
    eereadwrite(readflag, addr, statsBatteryChargingCapacityAverage);
    // <----------------------------new robot stats to save goes here!----------------
    ShowMessage(F("Robot Stats address Start = "));
    ShowMessageln(ADDR_ROBOT_STATS);
    ShowMessage(F("Robot Stats address Stop = "));
    ShowMessageln(addr);
  */
}

void Robot::loadSaveErrorCounters(boolean readflag) {
  /*
    if (readflag) ShowMessageln(F("Load ErrorCounters"));
    else ShowMessageln(F("Save ErrorCounters"));
    int addr = ADDR_ERR_COUNTERS;
    short magic = 0;
    if (!readflag) magic = MAGIC;
    eereadwrite(readflag, addr, magic); // magic
    if ((readflag) && (magic != MAGIC)) {
     ShowMessageln(F("EEPROM ERR COUNTERS: NO EEPROM ERROR DATA"));
     ShowMessageln(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
     addErrorCounter(ERR_EEPROM_DATA);
     setNextState(STATE_ERROR, 0);
     return;
    }
    eereadwrite(readflag, addr, errorCounterMax);
    ShowMessage(F("ErrorCounters address Start="));
    ShowMessageln(ADDR_ERR_COUNTERS);
    ShowMessage(F("ErrorCounters address Stop="));
    ShowMessageln(addr);
  */
}

void Robot::loadSaveUserSettings(boolean readflag) {
  /*

    int addr = ADDR_USER_SETTINGS;
    short magic = 0;
    if (!readflag) magic = MAGIC;
    eereadwrite(readflag, addr, magic); // magic

    if ((readflag) && (magic != MAGIC)) {

      ShowMessageln(F("EEPROM USERDATA: NO EEPROM USER DATA"));
      ShowMessageln(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
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
    eereadwrite(readflag, addr, reduceSpeedNearPerimeter);
    eereadwrite(readflag, addr, autoAdjustSlopeSpeed);
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
      ShowMessage(F("UserSettings are read from EEprom Address : "));
      ShowMessage(ADDR_USER_SETTINGS);
      ShowMessage(F(" To "));
      ShowMessageln(addr);
      motorInitialSpeedMaxPwm = motorSpeedMaxPwm; //the Pi can change the speed so store the initial value to restore after PFND for example
    }
    else
    {
      ShowMessage(F("UserSettings are saved from EEprom Address : "));
      ShowMessage(ADDR_USER_SETTINGS);
      ShowMessage(F(" To "));
      ShowMessageln(addr);
    }
  */
}

void Robot::loadUserSettings() {
  //return; // use in one shot to reset all the usersetting if acces on console is not possible
  // loadSaveUserSettings(true);
}


void Robot::printSettingSerial() {

  // ------- wheel motors ---------------------------------------------------------
  ShowMessageln("---------- wheel motors -----------");
  ShowMessage  ("motorAccel                 : ");
  ShowMessageln(motorAccel);
  ShowMessage  ("motorSpeedMaxRpm           : ");
  ShowMessageln(motorSpeedMaxRpm);
  ShowMessage  ("motorSpeedMaxPwm           : ");
  ShowMessageln(motorSpeedMaxPwm);
  ShowMessage  ("motorPowerMax              : ");
  ShowMessageln(motorPowerMax);
  ShowMessage  ("motorSenseRightScale       : ");
  ShowMessageln(motorSenseRightScale);
  ShowMessage  ("motorSenseLeftScale        : ");
  ShowMessageln(motorSenseLeftScale);
  //watchdogReset();
  ShowMessage  ("motorPowerIgnoreTime       : ");
  ShowMessageln(motorPowerIgnoreTime);
  ShowMessage  ("motorZeroSettleTime        : ");
  ShowMessageln(motorZeroSettleTime);
  ShowMessage  ("motorRollDegMax            : ");
  ShowMessageln(motorRollDegMax);
  ShowMessage  ("motorRollDegMin            : ");
  ShowMessageln(motorRollDegMin);
  ShowMessage  ("DistPeriOutRev             : ");
  ShowMessageln(DistPeriOutRev);
  //watchdogReset();
  ShowMessage  ("DistPeriOutStop            : ");
  ShowMessageln(DistPeriOutStop);
  ShowMessage  ("motorForwTimeMax           : ");
  ShowMessageln(motorForwTimeMax);
  ShowMessage  ("DistPeriObstacleAvoid      : ");
  ShowMessageln(DistPeriObstacleAvoid);
  ShowMessage  ("circleTimeForObstacle      : ");
  ShowMessageln(circleTimeForObstacle);
  ShowMessage  ("motorRightOffsetFwd        : ");
  ShowMessageln(motorRightOffsetFwd);
  //watchdogReset();
  ShowMessage  ("motorRightOffsetRev        : ");
  ShowMessageln(motorRightOffsetRev);
  ShowMessage  ("SpeedOdoMin                : ");
  ShowMessageln(SpeedOdoMin);
  ShowMessage  ("SpeedOdoMax                : ");
  ShowMessageln(SpeedOdoMax);
  ShowMessage  ("motorTickPerSecond         : ");
  ShowMessageln(motorTickPerSecond);

  ShowMessage  ("motorLeftPID.Kp            : ");
  ShowMessageln(motorLeftPID.Kp);
  ShowMessage  ("motorLeftPID.Ki            : ");
  ShowMessageln(motorLeftPID.Ki);
  ShowMessage  ("motorLeftPID.Kd            : ");
  ShowMessageln(motorLeftPID.Kd);

  ShowMessage  ("motorRightSwapDir          : ");
  ShowMessageln(motorRightSwapDir);
  ShowMessage  ("motorLeftSwapDir           : ");
  ShowMessageln(motorLeftSwapDir);
  ShowMessage  ("motorRightOffsetFwd        : ");
  ShowMessageln(motorRightOffsetFwd);
  ShowMessage  ("motorRightOffsetRev        : ");
  ShowMessageln(motorRightOffsetRev);
  ShowMessage  ("autoAdjustSlopeSpeed       : ");
  ShowMessageln(autoAdjustSlopeSpeed);



  //watchdogReset();
  delayWithWatchdog (500);
  // ------ mower motor -----------------------------------
  ShowMessageln("---------- mower motor -----------------");
  ShowMessage  ("motorMowForceOff         : ");
  ShowMessageln(motorMowForceOff);
  ShowMessage  ("motorMowAccel            : ");
  ShowMessageln(motorMowAccel);
  ShowMessage  ("motorMowSpeedMaxPwm      : ");
  ShowMessageln(motorMowSpeedMaxPwm);
  ShowMessage  ("(motorMowSpeedMinPwm     : ");
  ShowMessageln(motorMowSpeedMinPwm);
  ShowMessage  ("motorMowPowerMax         : ");
  ShowMessageln(motorMowPowerMax);
  ShowMessage  ("motorMowSenseScale       : ");
  ShowMessageln(motorMowSenseScale);

  //watchdogReset();
  // ------ bumper ------------------------------------
  ShowMessageln("---------- bumper -----------------");
  ShowMessage  ("bumperUse           : ");
  ShowMessageln(bumperUse);

  // ------ drop -------------------------------------
  ShowMessageln("---------- drop -----------------");
  ShowMessage  ("dropUse            : ");
  ShowMessageln(dropUse);
  ShowMessage  ("dropContact        : ");
  ShowMessageln(dropcontact);
  delayWithWatchdog (500);
  // ------ rain -------------------------------------
  ShowMessageln("---------- rain ----------------");
  ShowMessage  ("rainUse             : ");
  ShowMessageln(rainUse);

  // ------ DHT22 Temperature -----------------------
  ShowMessageln("----------  DHT22 Temperature ---");
  ShowMessage  ("DHT22Use           : ");
  ShowMessageln(DHT22Use);
  ShowMessage  ("MaxTemperature     : ");
  ShowMessageln(maxTemperature);

  //watchdogReset();

  // ------ sonar -----------------------------------
  ShowMessageln(F("---------- sonar ---------------"));
  ShowMessage  ("sonarUse              : ");
  ShowMessageln(sonarUse);
  ShowMessage  ("sonarLikeBumper       : ");
  ShowMessageln(sonarLikeBumper);
  ShowMessage  ("sonarLeftUse        : ");
  ShowMessageln(sonarLeftUse);
  ShowMessage  ("sonarRightUse       : ");
  ShowMessageln(sonarRightUse);
  ShowMessage  ("sonarCenterUse      : ");
  ShowMessageln(sonarCenterUse);
  ShowMessage  ("sonarTriggerBelow   : ");
  ShowMessageln(sonarTriggerBelow);
  ShowMessage  ("sonarToFrontDist    : ");
  ShowMessageln(sonarToFrontDist);

  //watchdogReset();
  delayWithWatchdog (500);
  // ------ perimeter --------------------------
  ShowMessageln("---------- perimeter ------");
  ShowMessage  ("perimeterUse             : ");
  ShowMessageln(perimeterUse);
  ShowMessage  ("perimeterTriggerMinSmag  : ");
  ShowMessageln(perimeterTriggerMinSmag);
  ShowMessage  ("MaxSpeedperiPwm          : ");
  ShowMessageln(MaxSpeedperiPwm);
  ShowMessage  ("perimeterTrackRollTime   : ");
  ShowMessageln(perimeterTrackRollTime);
  ShowMessage  ("perimeterTrackRevTime    : ");
  ShowMessageln(perimeterTrackRevTime);
  ShowMessage  ("perimeterPID.Kp          : ");
  ShowMessageln(perimeterPID.Kp);
  ShowMessage  ("perimeterPID.Ki          : ");
  ShowMessageln( perimeterPID.Ki);
  //watchdogReset();
  ShowMessage  ("perimeterPID.Kd          : ");
  ShowMessageln(perimeterPID.Kd);
  ShowMessage  ("trackingPerimeterTransitionTimeOut: ");
  ShowMessageln(trackingPerimeterTransitionTimeOut);
  ShowMessage  ("trackingErrorTimeOut     : ");
  ShowMessageln(trackingErrorTimeOut);
  ShowMessage  ("perimeterMagMaxValue     : ");
  ShowMessageln(perimeterMagMaxValue);
  //ShowMessage  ("swapCoilPolarityRight    : ");
  //watchdogReset();
  //ShowMessageln(perimeter.swapCoilPolarityRight);
  //ShowMessage  ("swapCoilPolarityLeft     : ");
  //ShowMessageln(perimeter.swapCoilPolarityLeft);
  //ShowMessage  ("read2Coil                : ");
  //ShowMessageln(perimeter.read2Coil);
  ShowMessage  ("trackingBlockInnerWheelWhilePerimeterStrug : ");
  ShowMessageln(trakBlockInnerWheel);
  ShowMessage  ("DistPeriOutRev           : ");
  ShowMessageln(DistPeriOutRev);
  ShowMessage  ("DistPeriObstacleRev      : ");
  ShowMessageln(DistPeriObstacleRev);
  ShowMessage  ("DistPeriOutForw          : ");
  ShowMessageln(DistPeriOutForw);
  ShowMessage  ("DistPeriObstacleForw     : ");
  ShowMessageln(DistPeriObstacleForw);
  //watchdogReset();
  delayWithWatchdog (500);
  // ------ By Lanes mowing ---------------------
  ShowMessageln(F("---------- By Lanes mowing ----------"));
  ShowMessage  (F("yawSet1                   : "));
  ShowMessageln(yawSet1);
  ShowMessage  (F("yawSet2                   : "));
  ShowMessageln(yawSet2);
  ShowMessage  (F("yawSet3                   : "));
  ShowMessageln(yawSet3);
  ShowMessage  (F("yawOppositeLane1RollRight : "));
  ShowMessageln(yawOppositeLane1RollRight);
  ShowMessage  (F("yawOppositeLane2RollRight : "));
  ShowMessageln(yawOppositeLane2RollRight);
  ShowMessage  (F("yawOppositeLane3RollRight : "));
  ShowMessageln(yawOppositeLane3RollRight);
  ShowMessage  (F("yawOppositeLane1RollLeft  : "));
  ShowMessageln(yawOppositeLane1RollLeft);
  //watchdogReset();
  ShowMessage  (F("yawOppositeLane2RollLeft  : "));
  ShowMessageln(yawOppositeLane2RollLeft);
  ShowMessage  (F("yawOppositeLane3RollLeft  : "));
  ShowMessageln(yawOppositeLane3RollLeft);
  ShowMessage  (F("DistBetweenLane           : "));
  ShowMessageln(DistBetweenLane);
  ShowMessage  (F("maxLenghtByLane           : "));
  ShowMessageln(maxLenghtByLane);
  //watchdogReset();
  // ------ lawn sensor ----------------------------
  ShowMessageln(F("---------- lawn sensor---------"));
  ShowMessage  (F("lawnSensorUse            : "));
  ShowMessageln(lawnSensorUse);

  // ------  IMU (compass/accel/gyro) ------
  ShowMessageln(F("---------- IMU (compass/accel/gyro) ---- "));
  ShowMessage  (F("imuUse                : "));
  ShowMessageln( imuUse);
  ShowMessage  (F("CompassUse            : "));
  ShowMessageln(CompassUse);
  ShowMessage  (F("stopMotorDuringCalib  : "));
  ShowMessageln(stopMotorDuringCalib);
  ShowMessage  (F("imuDirPID.Kp          : "));
  ShowMessageln(imuDirPID.Kp);
  ShowMessage  (F("imuDirPID.Ki          : "));
  ShowMessageln(imuDirPID.Ki);
  ShowMessage  (F("imuDirPID.Kd          : "));
  ShowMessageln( imuDirPID.Kd);
  //watchdogReset();
  ShowMessage  (F("maxDriftPerSecond     : "));
  ShowMessageln(maxDriftPerSecond);
  ShowMessage  (F("delayBetweenTwoDmpAutocalib : "));
  ShowMessageln(delayBetweenTwoDmpAutocalib);
  ShowMessage  (F("maxDurationDmpAutocalib     : "));
  ShowMessageln(maxDurationDmpAutocalib);
  ShowMessage  (F("compassRollSpeedCoeff       : "));
  ShowMessageln(compassRollSpeedCoeff);
  delayWithWatchdog (500);
  //watchdogReset();
  // ------ model R/C ------------------------------
  ShowMessageln(F("---------- model R/C ---------"));
  ShowMessage  (F("remoteUse                   : "));
  ShowMessageln(remoteUse);

  // ------ battery ----------------------------
  ShowMessageln(F("---------- battery --------  "));
  ShowMessage  (F("batMonitor           : "));
  ShowMessageln( batMonitor);
  ShowMessage  (F("batGoHomeIfBelow     : "));
  ShowMessageln(batGoHomeIfBelow);
  ShowMessage  (F("batSwitchOffIfBelow  : "));
  ShowMessageln(batSwitchOffIfBelow);
  ShowMessage  (F("batSwitchOffIfIdle   : "));
  ShowMessageln(batSwitchOffIfIdle);
  ShowMessage  (F("batFactor            : "));
  ShowMessageln( batFactor);
  ShowMessage  (F("batChgFactor         : "));
  ShowMessageln( batChgFactor);
  ShowMessage  (F("batFull              : "));
  ShowMessageln( batFull);
  //watchdogReset();
  ShowMessage  (F("batChargingCurrentMax: "));
  ShowMessageln(batChargingCurrentMax);
  ShowMessage  (F("batFullCurrent       : "));
  ShowMessageln(batFullCurrent);
  ShowMessage  (F("startChargingIfBelow : "));
  ShowMessageln(startChargingIfBelow);
  ShowMessage  (F("chargingTimeout      : "));
  ShowMessageln(chargingTimeout);
  ShowMessage  (F("chgSenseZero         : "));
  ShowMessageln(chgSenseZero);
  ShowMessage  (F("batSenseFactor       : "));
  ShowMessageln( batSenseFactor);
  ShowMessage  (F("chgSense             : "));
  ShowMessageln(chgSense);
  ShowMessage  (F("chgChange            : "));
  ShowMessageln(chgChange);
  ShowMessage  (F("chgNull              : "));
  ShowMessageln(chgNull);
  //watchdogReset();
  // ------  charging station -----------------------------------------------------
  ShowMessageln(F("---------- charging station ----------------------------------"));
  ShowMessage  (F("stationRevDist     : "));
  ShowMessageln(stationRevDist);
  ShowMessage  (F("stationRollAngle   : "));
  ShowMessageln(stationRollAngle);
  ShowMessage  (F("stationForwDist    : "));
  ShowMessageln(stationForwDist);
  ShowMessage  (F("stationCheckDist   : "));
  ShowMessageln(stationCheckDist);
  ShowMessage  (F("UseBumperDock      : "));
  ShowMessageln(UseBumperDock);
  ShowMessage  (F("dockingSpeed       : "));
  ShowMessageln(dockingSpeed);
  ShowMessage  (F("autoResetActive    : "));
  ShowMessageln(autoResetActive);

  //watchdogReset();


  // ------ odometry --------------------------------------------------------------
  ShowMessageln(F("---------- odometry ------------------------------------------"));
  ShowMessage  (F("odometryTicksPerRevolution : "));
  ShowMessageln( odometryTicksPerRevolution);
  ShowMessage  (F("odometryTicksPerCm         : "));
  ShowMessageln( odometryTicksPerCm);
  ShowMessage  (F("odometryWheelBaseCm        : "));
  ShowMessageln( odometryWheelBaseCm);



  //watchdogReset();

  // ----- GPS ----------------------------------------------------------------------
  ShowMessageln(F("---------- GPS -----------------------------------------------"));
  //ShowMessage  (F("gpsUse                : "));
  //ShowMessageln(gpsUse);
  //ShowMessage  (F("stuckIfGpsSpeedBelow  : "));
  //ShowMessageln(stuckIfGpsSpeedBelow);
  //ShowMessage  (F("gpsBaudrate           : "));
  //ShowMessageln(gpsBaudrate);
  //bber35
  // ----- RFID ----------------------------------------------------------------------
  ShowMessageln(F("---------- RFID ----------- "));
  ShowMessage  (F("rfidUse         : "));
  ShowMessageln(rfidUse);
  //watchdogReset();
  // ----- RASPBERRY PI --------------
  ShowMessageln(F("---------- RASPBERRY PI------ "));
  ShowMessage  (F("RaspberryPIUse  : "));
  ShowMessageln(RaspberryPIUse);

  // ----- other ----------------------------------------------------
  ShowMessageln(F("---------- other ------------"));
  ShowMessage  (F("buttonUse              : "));
  ShowMessageln(buttonUse);
  ShowMessage  (F("mowPatternDurationMax  : "));
  ShowMessageln(mowPatternDurationMax);

  //watchdogReset();

  // ----- user-defined switch ----------------------------------------
  ShowMessageln(F("---------- user-defined switch -------"));
  ShowMessage  (F("userSwitch1       : "));
  ShowMessageln(userSwitch1);
  ShowMessage  (F("userSwitch2       : "));
  ShowMessageln(userSwitch2);
  ShowMessage  (F("userSwitch3       : "));
  ShowMessageln(userSwitch3);
  //watchdogReset();
  // ----- timer --------------------------------------------------------------------
  ShowMessageln(F("---------- timer ----------- "));
  ShowMessage  (F("timerUse       : "));
  ShowMessageln(timerUse);




  //watchdogReset();
  // -------robot stats--------------------------------------------------------------
  ShowMessageln(F("---------- robot stats ---------------------------------------"));
  ShowMessage  (F("statsMowTimeMinutesTrip                    : "));
  ShowMessageln(statsMowTimeMinutesTrip);
  ShowMessage  (F("statsMowTimeMinutesTotal                   : "));
  ShowMessageln(statsMowTimeMinutesTotal);
  ShowMessage  (F("statsBatteryChargingCounterTotal           : "));
  ShowMessageln(statsBatteryChargingCounterTotal);
  ShowMessage  (F("statsBatteryChargingCapacityTrip in mAh    : "));
  ShowMessageln(statsBatteryChargingCapacityTrip);
  ShowMessage  (F("statsBatteryChargingCapacityTotal in Ah    : "));
  ShowMessageln(statsBatteryChargingCapacityTotal / 1000);
  ShowMessage  (F("statsBatteryChargingCapacityAverage in mAh : "));
  ShowMessageln(statsBatteryChargingCapacityAverage);
  //watchdogReset();
  //return;


}


void Robot::saveUserSettings() {
  ShowMessageln(F("START TO SAVE USER SETTINGS PLEASE WAIT"));
  loadSaveUserSettings(false);

}


void Robot::deleteUserSettings() {
  //int addr = ADDR_USER_SETTINGS;
  ShowMessageln(F("ALL USER SETTINGS ARE DELETED PLEASE RESTART THE DUE"));
  // eewrite(addr, (short)0); // magic
}

void Robot::deleteRobotStats() {
  statsMowTimeMinutesTrip = statsMowTimeMinutesTotal = statsBatteryChargingCounterTotal =
                              statsBatteryChargingCapacityTotal = statsBatteryChargingCapacityTrip = 0;
  loadSaveRobotStats(false);
  ShowMessageln(F("ALL ROBOT STATS ARE DELETED"));
}

void Robot::addErrorCounter(byte errType) {
  // increase error counters (both temporary and maximum error counters)
  if (errorCounter[errType] < 255) errorCounter[errType]++;
  if (errorCounterMax[errType] < 255) errorCounterMax[errType]++;
}

void Robot::resetErrorCounters() {
  ShowMessageln(F("resetErrorCounters"));
  for (int i = 0; i < ERR_ENUM_COUNT; i++) errorCounter[i] = errorCounterMax[i] = 0;
  loadSaveErrorCounters(false);
  resetMotorFault();
}

void Robot::resetMotorFault() {
  /*
    if (digitalRead(pinMotorLeftFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    ShowMessageln(F("Reset motor left fault"));
    }
    if  (digitalRead(pinMotorRightFault) == LOW) {
    digitalWrite(pinMotorEnable, LOW);
    digitalWrite(pinMotorEnable, HIGH);
    ShowMessageln(F("Reset motor right fault"));
    }
    if (digitalRead(pinMotorMowFault) == LOW) {
    digitalWrite(pinMotorMowEnable, LOW);
    digitalWrite(pinMotorMowEnable, HIGH);
    ShowMessageln(F("Reset motor mow fault"));
    }
  */
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
        ShowMessage("Error Counter > 10 for counter num ");
        ShowMessageln(i);
        setNextState(STATE_ERROR, 0);
      }
    }
  }
}


void Robot::autoReboot() {
  //this feature use the watchdog to perform a restart of the due
  if (RaspberryPIUse) {
    ShowMessageln(F("Due reset after 1 secondes, send a command to Pi for restart also"));
    // MyRpi.sendCommandToPi("RestartPi");
  }
  else
  {
    ShowMessageln(F("Due reset after 1 secondes"));
  }
  delay(1000);
  //watchdogReset();
  delay(30000); // this IS USED to force watchdog to reset due.
}

// ---- motor RPM (interrupt) --------------------------------------------------------------
// mower motor RPM driver
void Robot::setMotorMowRPMState(boolean motorMowRpmState) {
  /*
    if (motorMowRpmState != motorMowRpmLastState) {
    motorMowRpmLastState = motorMowRpmState;
    if (motorMowRpmLastState) motorMowRpmCounter++;
    }
  */
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
  //setActuator(ACT_MOTOR_MOW, min(motorMowSpeedMaxPwm, max(0, motorMowPWMCurr)));
}


// sets wheel motor actuators
// - driver protection: delays polarity change until motor speed (EMV) is zero
//   http://wiki.ardumower.de/images/a/a5/Motor_polarity_switch_protection.png
// - optional: ensures that the motors (and gears) are not switched to 0% (or 100%) too fast (motorAccel)
void Robot::setMotorPWM(int pwmLeft, int pwmRight, boolean useAccel) {
  int TaC = int(millis() - lastSetMotorSpeedTime);    // sampling time in millis
  lastSetMotorSpeedTime = millis();
  if (TaC > 1000) TaC = 1;

  if (stateCurr != STATE_OFF) {
    /*
      ShowMessage(stateNames[stateCurr]);
      ShowMessage(" Voeux a ");
      ShowMessage (millis());
      ShowMessage(" TaC=");
      ShowMessage (TaC);
      ShowMessage(" Useaccel=");
      ShowMessage (useAccel);
      ShowMessage(" pwmLeft ");
      ShowMessageln (pwmLeft);

      ShowMessage ("  motorLeftZeroTimeout : ");
      ShowMessage (motorLeftZeroTimeout);
      ShowMessage(" motorLeftPWMCurr=");
      ShowMessageln (motorLeftPWMCurr);
    */
  }

  // ----- driver protection (avoids driver explosion) ----------
  if ( ((pwmLeft < 0) && (motorLeftPWMCurr > 0)) || ((pwmLeft > 0) && (motorLeftPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      ShowMessage("WARNING PROTECTION ON LEFT MOTOR ");
      ShowMessage("  motorLeftPWMCurr=");
      ShowMessage (motorLeftPWMCurr);
      ShowMessage("  pwmLeft=");
      ShowMessage (pwmLeft);
      ShowMessage(" state ");
      ShowMessageln(stateNames[stateCurr]);
      if (motorLeftZeroTimeout != 0) pwmLeft = motorLeftPWMCurr - motorLeftPWMCurr * ((float)TaC) / 200.0; // reduce speed
    }
  }
  if ( ((pwmRight < 0) && (motorRightPWMCurr > 0)) || ((pwmRight > 0) && (motorRightPWMCurr < 0)) ) { // slowing before reverse
    if (developerActive) {
      ShowMessage("WARNING PROTECTION ON RIGHT MOTOR ");
      ShowMessage("  motorRightPWMCurr=");
      ShowMessage (motorRightPWMCurr);
      ShowMessage("  pwmRight=");
      ShowMessage (pwmRight);
      ShowMessage("  On state ");
      ShowMessageln(stateNames[stateCurr]);
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
        ShowMessage(" motorLeftZeroTimeout=");
        ShowMessage (motorLeftZeroTimeout);
        ShowMessage(" motorLeftChange=");
        ShowMessage (motorLeftChange);
        ShowMessage(" pwmRight=");
        ShowMessage (pwmRight);
        ShowMessage(" motorRightPWMCurr=");
        ShowMessage (motorRightPWMCurr);
        ShowMessage(" pwmLeft=");
        ShowMessage (pwmLeft);
        ShowMessage(" motorLeftPWMCurr=");
        ShowMessageln (motorLeftPWMCurr);
         if (motorLeftPWMCurr >255) {
          motorLeftPWMCurr=255;
          ShowMessageln ("motorLeftPWMCurr 2555555555555555555555555555555555555555");
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
      ShowMessage(" result ");
      ShowMessage (millis());
      ShowMessage(" Right/Left ");
      ShowMessage (motorRightPWMCurr);
      ShowMessage(" / ");
      ShowMessageln (motorLeftPWMCurr);
    */


  }
  // ---------------------------------
  /*
    if (motorLeftSwapDir)  // swap pin polarity?
    //setActuator(ACT_MOTOR_LEFT, -motorLeftPWMCurr);

    else
      //setActuator(ACT_MOTOR_LEFT, motorLeftPWMCurr);
      if (motorRightSwapDir)   // swap pin polarity?
        //setActuator(ACT_MOTOR_RIGHT, -motorRightPWMCurr);
        else
          //setActuator(ACT_MOTOR_RIGHT, motorRightPWMCurr);
        }

  */
}

void Robot::OdoRampCompute() { //execute only one time when a new state execution
  //Compute the accel duration (very important for small distance)
  //Compute when you need to brake the 2 wheels to stop at the ODO
  //Compute the estimate duration of the state so can force next state if the mower is stuck
  //bber400
  motorSpeedRpmMedian.clear();


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
      if (PwmLeftSpeed <= 0) {
        // SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
        SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      }
      else {
        //SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
        SpeedOdoMaxLeft = map(distToMoveLeft / 2, odometryTicksPerRevolution / 2, 0, PwmLeftSpeed, SpeedOdoMax);
      }
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
      if (PwmRightSpeed <= 0) {
        SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, -SpeedOdoMax); //valeur de vitesse max en fonction de la distance a parcourir
      }
      else {
        SpeedOdoMaxRight = map(distToMoveRight / 2, odometryTicksPerRevolution / 2, 0, PwmRightSpeed, SpeedOdoMax);
      }
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
  //bber500
  if (movingTimeLeft < 4000 ) movingTimeLeft = 4000;

  //for small mouvement need to increase duration
  movingTimeRight = 1000 * distToMoveRight / motorTickPerSecond ;
  movingTimeRight = movingTimeRight * motorSpeedMaxPwm / abs(SpeedOdoMaxRight);
  //bber500 reduce movement shock
  if (movingTimeRight < 4000 ) movingTimeRight = 4000;

  //for small mouvement need to reduce the accel duration
  if (movingTimeLeft >= motorOdoAccel) accelDurationLeft = motorOdoAccel;
  else   accelDurationLeft =  movingTimeLeft / 2;
  if (movingTimeRight >= motorOdoAccel) accelDurationRight = motorOdoAccel;
  else   accelDurationRight =  movingTimeRight / 2;
  if (statusCurr == TESTING) {  //avoid maxduration stop when use test Odo with Pfod
    MaxOdoStateDuration = 30000 + max(movingTimeRight, movingTimeLeft); //add 30 secondes to the max moving duration of the 2 wheels
  }
  else
  {
    MaxOdoStateDuration = 3000 + max(movingTimeRight, movingTimeLeft); //add 3 secondes to the max moving duration of the 2 wheels
  }
  //check to set the correct heading
  // imuDriveHeading = imu.ypr.yaw / PI * 180; //normal mowing heading
  if (statusCurr == BACK_TO_STATION) {  //possible heading change
    imuDriveHeading = periFindDriveHeading / PI * 180;
  }
  if (statusCurr == REMOTE) {   //possible heading change
    imuDriveHeading = remoteDriveHeading / PI * 180;
  }

  /*
    ShowMessage(" **************** compute  at  ");
    ShowMessageln(millis());
    ShowMessage(" UseAccelRight ");
    ShowMessage(UseAccelRight);
    ShowMessage(" UseBrakeRight ");
    ShowMessage(UseBrakeRight);
    ShowMessage(" UseAccelLeft ");
    ShowMessage(UseAccelLeft);
    ShowMessage(" UseBrakeLeft ");
    ShowMessage(UseBrakeLeft);
    ShowMessage(" distToMoveLeft ");
    ShowMessage(distToMoveLeft);
    ShowMessage(" movingTimeLeft ");
    ShowMessage(movingTimeLeft);
    ShowMessage("ms movingTimeRight ");
    ShowMessageln(movingTimeRight);
    ShowMessage("accelDurationLeft ");
    ShowMessage(accelDurationLeft);
    ShowMessage("ms accelDurationRight ");
    ShowMessageln(accelDurationRight);

    ShowMessage (F(stateNames[stateNext]));
    ShowMessage(" RightSpeedRpmSet ");
    ShowMessage(motorRightSpeedRpmSet);
    ShowMessage("  PwmRightSpeed ");
    ShowMessage(PwmRightSpeed);
    ShowMessage("  SpeedOdoMaxRight ");
    ShowMessageln(SpeedOdoMaxRight);






    ShowMessage("OdoStartBrakeLeft ");
    ShowMessage(OdoStartBrakeLeft);
    ShowMessage("Ticks OdoStartBrakeRight ");
    ShowMessageln(OdoStartBrakeRight);
    ShowMessage("MaxOdoStateDuration ");
    ShowMessage(MaxOdoStateDuration);
    ShowMessageln(" ms");
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
    // ShowMessage(" FR rotate ");
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
      // YawActualDeg = imu.ypr.yaw / PI * 180;
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

      //if ((imu.ypr.yaw / PI * 180) > 0 ) imuDriveHeading = yawCiblePos;
      else imuDriveHeading = yawCibleNeg;
      //imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
      imuDirPID.w = 0;
      imuDirPID.y_min = -motorSpeedMaxPwm / 2;
      imuDirPID.y_max = motorSpeedMaxPwm / 2;
      imuDirPID.max_output = motorSpeedMaxPwm / 2;
      imuDirPID.compute();

      if ((millis() - stateStartTime) < 1000) { // acceleration and more influence of PID vs speed
        //bber402

        rightSpeed =  rightSpeed - (66 - (millis() - stateStartTime) / 30);
        leftSpeed =  leftSpeed - (66 - (millis() - stateStartTime) / 30);
        if (rightSpeed < 0 ) rightSpeed = 0;
        if (leftSpeed < 0 ) leftSpeed = 0;
      }
      else //adjust rpm speed only after 1 seconde
      {
        //bber400 //adjust RPM speed
        //PID version
        motorRightPID.x = motorRightRpmCurr;
        motorRightPID.w = motorSpeedMaxRpm;
        motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
        motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
        motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
        motorRightPID.compute();
        //ShowMessageln(motorRightPID.y);
        motorRpmCoeff = (100 + motorRightPID.y) / 100;
        if (motorRpmCoeff < 0.80) motorRpmCoeff = 0.80;
        if (motorRpmCoeff > 1.20) motorRpmCoeff = 1.20;

        /*
                //median version
                //add median on current RPM
                motorSpeedRpmMedian.add(motorRightRpmCurr + motorLeftRpmCurr);
                if (motorSpeedRpmMedian.getCount() >= 33) { //check each 33 * 15 millisecondes = 0.5 secondes
                  //ShowMessageln(motorSpeedRpmMedian.getAverage(8)/2);
                  //motorRpmCoeff = float((2 * motorSpeedMaxRpm / motorSpeedRpmMedian.getAverage(8))) ;
                  //if (motorRpmCoeff < 0.50) motorRpmCoeff = 0.50;
                  //if (motorRpmCoeff > 1.50) motorRpmCoeff = 1.50;
                  motorSpeedRpmMedian.clear();
                  ShowMessage(motorRpmCoeff);
                  ShowMessage(" / ");
                  ShowMessage(motorRightPID.y);
                  ShowMessage(" / ");
                  ShowMessageln(rightSpeed);
                }
        */
      }

      if ((sonarSpeedCoeff != 1) || (!autoAdjustSlopeSpeed)) { //do not change speed if sonar is activate
        motorRpmCoeff = 1;
      }

      rightSpeed =  (motorRpmCoeff  * rightSpeed) + imuDirPID.y / 2;
      leftSpeed =  (motorRpmCoeff  * leftSpeed) - imuDirPID.y / 2;




      //not use ??------------------------------------------------------------try to find the yaw with the odometry-------------------------------------
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
          ShowMessage(" odoDiffRightLeft  ");
          ShowMessage(odoDiffRightLeft);
          ShowMessage(" 2* odoDiffRightLeft /odometryTicksPerCm  / odometryWheelBaseCm ");
          ShowMessage(2* odoDiffRightLeft /odometryTicksPerCm  / odometryWheelBaseCm);
          ShowMessage(" odoTheta  ");
          ShowMessage(odoTheta,4);
          ShowMessage(" straightLineTheta  ");
          ShowMessageln(straightLineTheta,4);
        */
      }
      //----------------------------------------------------------------------------------------------------------------------------------------------------------------

    }
    else
      //// NORMAL MOWING OR PERIFIND
    {
      if (imuUse) /// use the IMU for straight line
      {
        // YawActualDeg = imu.ypr.yaw / PI * 180;
        // if(abs(YawActualDeg) >90) YawMedianDeg = imu.rotate360(YawActualDeg);
        // else YawMedianDeg= YawActualDeg+90;

        //imuDirPID.x = imu.distance180(YawActualDeg, imuDriveHeading);
        imuDirPID.w = 0;
        imuDirPID.y_min = -motorSpeedMaxPwm / 2;
        imuDirPID.y_max = motorSpeedMaxPwm / 2;
        imuDirPID.max_output = motorSpeedMaxPwm / 2;
        imuDirPID.compute();

        //bber400 //adjust RPM speed
        //PID version
        motorRightPID.x = motorRightRpmCurr;
        motorRightPID.w = motorSpeedMaxRpm;
        motorRightPID.y_min = -motorSpeedMaxPwm;       // Regel-MIN
        motorRightPID.y_max = motorSpeedMaxPwm;  // Regel-MAX
        motorRightPID.max_output = motorSpeedMaxPwm;   // Begrenzung
        motorRightPID.compute();
        //ShowMessageln(motorRightPID.y);
        motorRpmCoeff = (100 + motorRightPID.y) / 100;
        if (motorRpmCoeff < 0.80) motorRpmCoeff = 0.80;
        if (motorRpmCoeff > 1.20) motorRpmCoeff = 1.20;

        if ((sonarSpeedCoeff != 1) || (!autoAdjustSlopeSpeed)) { //do not change speed if sonar is activate
          motorRpmCoeff = 1;
        }

        rightSpeed =  (motorRpmCoeff  * rightSpeed) + imuDirPID.y / 2;
        leftSpeed =  (motorRpmCoeff  * leftSpeed) - imuDirPID.y / 2;


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

    //bber200 reduce perimeter speed only if both perimeter and sonar are actif
    if (perimeterSpeedCoeff == 1) {
      rightSpeed = rightSpeed * sonarSpeedCoeff;
      leftSpeed = leftSpeed * sonarSpeedCoeff;
    }
    else
    {
      rightSpeed = rightSpeed * perimeterSpeedCoeff;
      leftSpeed = leftSpeed * perimeterSpeedCoeff;
    }

    if (rightSpeed > 255) rightSpeed = 255;
    if (leftSpeed > 255) leftSpeed = 255;
    if (rightSpeed < 0) rightSpeed = 0;
    if (leftSpeed < 0) leftSpeed = 0;

  }

  if (stateCurr != STATE_OFF) {
    /*
      if (perimeterSpeedCoeff != 1) {
      ShowMessageln(perimeterSpeedCoeff);
      }

        ShowMessage(millis());
        ShowMessage(" Moving Average Dist= ");
        ShowMessage(currDistToDrive);
        ShowMessage(" ODO **** Lspeed= ");
        ShowMessage(leftSpeed);
        ShowMessage(" ODO Start/Actual/End ");
        ShowMessage(stateStartOdometryLeft);
        ShowMessage("/");
        ShowMessage(odometryLeft);
        ShowMessage("/");
        ShowMessage(stateEndOdometryLeft);
        ShowMessage(" ************************* Rspeed= ");
        ShowMessage(rightSpeed);
        ShowMessage(" ODO Start/Actual/End ");
        ShowMessage(stateStartOdometryRight);
        ShowMessage("/");
        ShowMessage(odometryRight);
        ShowMessage("/");
        ShowMessage(stateEndOdometryRight);
        ShowMessage(" PID reel ");
        ShowMessage(motorRightPID.x);
        ShowMessage(" PID resultat du calcul ");
        ShowMessageln(motorRightPID.y);
        ShowMessage("IMU ***** Line use ");
        ShowMessage(laneUseNr);
        ShowMessage(" imuDriveHeading ");
        ShowMessage(imuDriveHeading);
        ShowMessage(" YawMedianDeg ");
        ShowMessage(YawMedianDeg);
        ShowMessage(" YawActualDeg ");
        ShowMessage(YawActualDeg);
        ShowMessage(" correctRight ");
        ShowMessage(correctRight);
        ShowMessage(" correctLeft ");
        ShowMessage(correctLeft);
        ShowMessage(" PID reel ");
        ShowMessage(imuDirPID.x);
        ShowMessage(" PID resultat du calcul ");
        ShowMessageln(imuDirPID.y);
        ShowMessage(" imu.ypr.yaw ");
        ShowMessageln(imu.ypr.yaw);
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
      ShowMessage("SEARCH;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMag);
      ShowMessage(";");
      ShowMessage(perimeterInside);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
    setMotorPWM( leftSpeedperi, rightSpeedperi, false);

    lastTimeForgetWire = millis();

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut) {
      if (perimeterInside) {
        ShowMessageln("Tracking Fail and we are inside, So start to find again the perimeter");
        //   periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      }
      else
      {
        ShowMessageln("Tracking Fail and we are outside, So start to roll to find again the perimeter");
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
      ShowMessage("SLOW;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMag);
      ShowMessage(";");
      ShowMessage(perimeterInside);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
    }
  }
  else
  {
    rightSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5   + perimeterPID.y));
    leftSpeedperi = max(0, min(ActualSpeedPeriPWM, ActualSpeedPeriPWM / 1.5  - perimeterPID.y));

    if (consoleMode == CONSOLE_TRACKING) {
      ShowMessage("FAST;");
      ShowMessage(millis());
      ShowMessage(";");
      ShowMessage (perimeterMag);
      ShowMessage(";");
      ShowMessage(perimeterInside);
      ShowMessage(";");
      ShowMessage (perimeterPID.x);
      ShowMessage(";");
      ShowMessage(perimeterPID.y);
      ShowMessage(";");
      ShowMessage (leftSpeedperi);
      ShowMessage(";");
      ShowMessage (rightSpeedperi);
      ShowMessage(";");
      ShowMessageln(perimeterLastTransitionTime);
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
  /*
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
      ShowMessage("Left odometry error: PWM=");
      ShowMessage(motorLeftPWMCurr);
      ShowMessage("\tRPM=");
      ShowMessageln(motorLeftRpmCurr);
      addErrorCounter(ERR_ODOMETRY_LEFT);
      setNextState(STATE_ERROR, 0);
    }
    if (rightErr) {
      ShowMessage("Right odometry error: PWM=");
      ShowMessage(motorRightPWMCurr);
      ShowMessage("\tRPM=");
      ShowMessageln(motorRightRpmCurr);
      addErrorCounter(ERR_ODOMETRY_RIGHT);
      setNextState(STATE_ERROR, 0);
    }
  */
}

void Robot::motorControl() {
  if (millis() < nextTimeMotorControl) return;
  nextTimeMotorControl = millis() + 200;  // 10 at the original
  //static unsigned long nextMotorControlOutputTime = 0;

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
      ShowMessage("PID x=");
      ShowMessage(motorLeftPID.x);
      ShowMessage("\tPID w=");
      ShowMessage(motorLeftPID.w);
      ShowMessage("\tPID y=");
      ShowMessage(motorLeftPID.y);
      ShowMessage("\tPWM=");
      ShowMessageln(leftSpeed);
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
    ShowMessageln(F("BATTERY switching ON again"));
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
    // Buzzer.noTone();
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
        //   Buzzer.tone(beepfrequenceOn);
      }
      else {
        //  Buzzer.tone(beepfrequenceOff);
      }
    }
  }
}


// set user-defined switches
void Robot::setUserSwitches() {
  /*
    setActuator(ACT_USER_SW1, userSwitch1);
    setActuator(ACT_USER_SW2, userSwitch2);
    setActuator(ACT_USER_SW3, userSwitch3);
  */
}



void Robot::setup()  {

  //  mower.h start before the robot setup

  Console.print("++++++++++++++* Start Robot Setup at ");
  Console.print(millis());
  Console.println(" ++++++++++++");

  rc.initSerial(&Bluetooth, BLUETOOTH_BAUDRATE);

  // if (RaspberryPIUse) MyRpi.init();





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


  // if (imuUse) imu.begin();

  if (perimeterUse) {
    Console.println(" ------- Initialize Perimeter Setting ------- ");
    // perimeter.changeArea(1);
    // perimeter.begin(pinPerimeterLeft, pinPerimeterRight);
  }

  if (!buttonUse) {
    // robot has no ON/OFF button => start immediately
    setNextState(STATE_FORWARD_ODO, 0);
  }


  stateStartTime = millis();
  //setBeeper(100, 50, 50, 200, 200 );//beep for 3 sec

  Console.println(F("START"));
  Console.print(F("Mower "));
  Console.println(VER);
#ifdef USE_DEVELOPER_TEST
  Console.println("Warning: DEVELOPER_TEST activated");
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


  nextTimeInfo = millis();

  
}


void Robot::printOdometry() {
  ShowMessage(F("ODO,"));
  ShowMessage(odometryX);
  ShowMessage(",");
  ShowMessageln(odometryY);
  ShowMessage(F("ODO,"));
  ShowMessage(odometryX);
  ShowMessage(",");
  ShowMessageln(odometryY);
}


void Robot::receivePiPfodCommand (String RpiCmd, float v1, float v2, float v3) {
  //rc.processPI(RpiCmd, v1, v2, v3);
}






void Robot::printInfo(Stream & s) {
  /*
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
  */
}

void Robot::printMenu() {
  Console.println(" ");
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
  Console.println(" ");
}

void Robot::delayWithWatchdog(int ms) {
  unsigned long endtime = millis() + ms;
  while (millis() < endtime) {
    delay(500);
    //watchdogReset();
  }
}

void Robot::delayInfo(int ms) {
  unsigned long endtime = millis() + ms;
  while (millis() < endtime) {
    // readSensors();
    printInfo(Console);
    //watchdogReset();
    delay(1000);
    //watchdogReset();
  }
}
/*odometryTicksPerRevolution = 720;   // encoder ticks per one full resolution
    odometryTicksPerCm = 9.96;  // encoder ticks per cm
    odometryWheelBaseCm = 42;
*/

void Robot::testMotors() {
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  ShowMessageln(F("testing left motor (forward) half speed..."));
  delay(100);
  motorLeftPWMCurr = motorSpeedMaxPwm / 2; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  ShowMessageln(F("testing left motor (reverse) full speed..."));
  delay(100);
  motorLeftPWMCurr = -motorSpeedMaxPwm; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  ShowMessageln(F("testing right motor (forward) half speed..."));
  delay(100);
  motorLeftPWMCurr = 0; motorRightPWMCurr = motorSpeedMaxPwm / 2;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);
  delayInfo(5000);
  motorLeftPWMCurr = 0; motorRightPWMCurr = 0;
  setMotorPWM(motorLeftPWMCurr, motorRightPWMCurr, false);

  ShowMessageln(F("testing right motor (reverse) full speed..."));
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
    //watchdogReset();
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
          //     imu.deleteAccelGyroCalib();
          imuUse = false;
          printMenu();
          break;
        case '6':
          //       imu.deleteCompassCalib();
          CompassUse = false;
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
          ShowMessageln(F("DONE"));
          printMenu();
          break;
        case 'e':
          resetErrorCounters();
          setNextState(STATE_OFF, 0);
          ShowMessageln(F("ALL ERRORS ARE DELETED"));
          printMenu();
          break;
      }
    }
    delay(10);
  }
}


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
        //setBeeper(400, 50, 50, 200, 0 );//error
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
        ShowMessageln(consoleModeNames[consoleMode]);
        break;


    }
  }

}

void Robot::checkButton() {
  if ( (!buttonUse) || (millis() < nextTimeButtonCheck) ) return;
  nextTimeButtonCheck = millis() + 100;
  //boolean buttonPressed = (readSensor(SEN_BUTTON) == LOW);
  boolean buttonPressed = false;
  if ( ((!buttonPressed) && (buttonCounter > 0)) || ((buttonPressed) && (millis() >= nextTimeButton)) )
  {
    nextTimeButton = millis() + 1000;
    if (buttonPressed) {
      //ShowMessage(F("Button Pressed counter : "));
      //ShowMessageln(buttonCounter);
      // ON/OFF button pressed
      //setBeeper(50, 50, 0, 200, 0 );//
      buttonCounter++;
      if (buttonCounter >= 3) buttonCounter = 3;
      //resetIdleTime();
    }
    else
    {
      // ON/OFF button released
      //ShowMessage(F("Button Release counter : "));
      //ShowMessageln(buttonCounter);
      if ((statusCurr == NORMAL_MOWING) || (statusCurr == SPIRALE_MOWING) || (stateCurr == STATE_ERROR) || (statusCurr == WIRE_MOWING) || (statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START)) {
        ShowMessageln(F("ButtonPressed Stop Mowing and Reset Error"));
        motorMowEnable = false;
        buttonCounter = 0;
        setNextState(STATE_OFF, 0);
        return;
      }
      if  ((stateCurr == STATE_OFF) || (stateCurr == STATE_STATION)) {
        if (buttonCounter == 1) {
          motorMowEnable = true;
          ShowMessageln("MANUAL START FROM STATION");
          statusCurr = NORMAL_MOWING;
          findedYaw = 999;
          imuDirPID.reset();
          mowPatternCurr = MOW_LANES;
          laneUseNr = 1;
          rollDir = 1;
          whereToStart = 1;
          areaToGo = 1;
          actualLenghtByLane = 40;
          beaconToStart = 0;
          mowPatternDuration = 0;
          totalDistDrive = 0;
          buttonCounter = 0;
          // if (RaspberryPIUse) MyRpi.SendStatusToPi();

          if (stateCurr == STATE_STATION) {
            setActuator(ACT_CHGRELAY, 0);
            setNextState(STATE_STATION_REV, 0);
          }

          else {
            setNextState(STATE_ACCEL_FRWRD, 0);
            return;
          }



        }
        else if (buttonCounter == 2) {
          // start normal with random mowing
          motorMowEnable = true;
          statusCurr = NORMAL_MOWING;
          mowPatternCurr = MOW_RANDOM;
          buttonCounter = 0;
          // if (RaspberryPIUse) MyRpi.SendStatusToPi();
          if (stateCurr == STATE_STATION) {
            //setActuator(ACT_CHGRELAY, 0);
            setNextState(STATE_STATION_REV, 0);
          }

          else {
            setNextState(STATE_ACCEL_FRWRD, 0);
            return;
          }


        }
        else if (buttonCounter == 3) {
          if (stateCurr == STATE_STATION) return;
          //go to station
          //periFindDriveHeading = scalePI(imu.ypr.yaw);
          areaToGo = 1;
          whereToStart = 99999;
          nextTimeTimer = millis() + 3600000; //avoid the mower start again if timer activate.
          statusCurr = BACK_TO_STATION;
          buttonCounter = 0;
          // if (RaspberryPIUse) MyRpi.SendStatusToPi();
          //periFindDriveHeading = imu.ypr.yaw;
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
    ShowMessage("Find a tag : ");
    ShowMessageln(rfidTagFind);
    if (rfidUse) {
      // if (RaspberryPIUse) MyRpi.SendRfidToPi();
    }
  }



}

void Robot::readSensors() {
  //NOTE: this function should only put sensors value into variables - it should NOT change any state!
  //The ADC return is now 12 bits so 0 to 4096
  if (millis() >= nextTimeMotorSense) {
    nextTimeMotorSense = millis() +  50;
    double accel = 0.05;
    //motorRightSenseADC = readSensor(SEN_MOTOR_RIGHT) ; //return the ADC value,for MC33926 0.525V/1A so ADC=651/1Amp
    //motorLeftSenseADC = readSensor(SEN_MOTOR_LEFT) ;
    //motorMowSenseADC = readSensor(SEN_MOTOR_MOW) ;
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
    /*
      if (perimeter.read2Coil) {
      //perimeterMagRight = readSensor(SEN_PERIM_RIGHT);
      }
    */
    //perimeterMag = readSensor(SEN_PERIM_LEFT);
    /*
      if ((perimeter.isInside(0) != perimeterInside)) {
      perimeterCounter++;
      perimeterLastTransitionTime = millis();
      //perimeterInside = perimeter.isInside(0);
      }
    */
    if ((!perimeterInside) && (perimeterTriggerTime == 0)) {
      // set perimeter trigger time

      //bber2
      //use smooth to avoid big area transition, in the middle of the area with noise the mag can change from + to -
      //smoothPeriMag = perimeter.getSmoothMagnitude(0);
      if (smoothPeriMag > perimeterTriggerMinSmag) {
        perimeterTriggerTime = millis();
      }
      else
      {
        if (millis() >= nextTimePrintConsole) {
          nextTimePrintConsole = millis() + 1000;
          if ((developerActive) && (stateCurr == STATE_FORWARD_ODO)) {
            ShowMessageln("Bad reading perimeter In/Out, certainly we are very far the wire");
          }
        }
      }

    }

    /*
        if (perimeter.signalTimedOut(0) || ((perimeter.read2Coil) && perimeter.signalTimedOut(1) ))  {
          //bber2
          if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_MOW_SPIRALE))   { // all the other state are distance limited
            //need to find a way in tracking mode maybe timeout error if the tracking is perfect, the mower is so near the wire than the mag is near 0 (adjust the timedOutIfBelowSmag)
            //if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_MOW_SPIRALE))   { // all the other state are distance limited
            ShowMessageln("Error: perimeter too far away");
            addErrorCounter(ERR_PERIMETER_TIMEOUT);
            setNextState(STATE_ERROR, 0);
            return;
          }
        }
    */
  }





  if ((bumperUse) && (millis() >= nextTimeBumper)) {
    nextTimeBumper = millis() + 100;
    /*
      if (readSensor(SEN_BUMPER_LEFT) == 0) {
      //ShowMessageln("Bumper left trigger");
      bumperLeftCounter++;
      bumperLeft = true;
      }

      if (readSensor(SEN_BUMPER_RIGHT) == 0) {
      //ShowMessageln("Bumper right trigger");
      bumperRightCounter++;
      bumperRight = true;
      }
    */
  }






  if (millis() >= nextTimeBattery) {
    // read battery
    nextTimeBattery = millis() + 500;
    if ((abs(chgCurrent) > 0.04) && (chgVoltage > 5)) {
      // charging
      batCapacity += (chgCurrent / 36.0);
    }
    //   batADC = readSensor(SEN_BAT_VOLTAGE);


    /*
        double batvolt = batFactor * readSensor(SEN_BAT_VOLTAGE) * 3.3 / 4096 ; //readsensor return the ADC value 0 to 4096 so *3.3/4096=voltage on the arduino pin batfactor depend on the resitor on board
        double chgvolt = batChgFactor * readSensor(SEN_CHG_VOLTAGE) * 3.3 / 4096 ;
        double curramp = batSenseFactor * readSensor(SEN_CHG_CURRENT) * 3.3 / 4096 ;
    */
    double batvolt = 0 ; //readsensor return the ADC value 0 to 4096 so *3.3/4096=voltage on the arduino pin batfactor depend on the resitor on board
    double chgvolt = 0 ;
    double curramp = 0 ;


    /*
      ShowMessage(millis());
      ShowMessage("/batvolt ");
      ShowMessage(batvolt);
      ShowMessage("/chgvolt ");
      ShowMessage(chgvolt);
      ShowMessage("/curramp ");
      ShowMessageln(curramp);
    */
    // low-pass filter
    //double accel = 0.01;
    double accel = 0.05;

    if (abs(batVoltage - batvolt) > 8)   batVoltage = batvolt; else batVoltage = (1.0 - accel) * batVoltage + accel * batvolt;
    if (abs(chgVoltage - chgvolt) > 8)   chgVoltage = chgvolt; else chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
    if (abs(chgCurrent - curramp) > 0.4) chgCurrent = curramp; else chgCurrent = (1.0 - accel) * chgCurrent + accel * curramp; //Deaktiviert fÃ¼r Ladestromsensor berechnung
    //bber30 tracking not ok with this but can check the chgvoltage
    /*
        ShowMessage(millis());
        ShowMessage("/batVoltage ");
        ShowMessage(batVoltage);
        ShowMessage("/chgVoltage ");
        ShowMessage(chgVoltage);
        ShowMessage("/chgCurrent ");
        ShowMessageln(chgCurrent);
    */
  }

  if ((rainUse) && (millis() >= nextTimeRain)) {
    // read rain sensor
    nextTimeRain = millis() + 5000;
    // rain = (readSensor(SEN_RAIN) != 0);
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
        // setActuator(ACT_CHGRELAY, 0);
        motorMowEnable = false;
      }
      motorLeftSpeedRpmSet = motorSpeedMaxRpm; //use RPM instead of PWM to straight line
      motorRightSpeedRpmSet = motorSpeedMaxRpm;
      statsMowTimeTotalStart = true;

      break;


    case STATE_FORWARD_ODO:
      if (statusCurr != NORMAL_MOWING) {
        statusCurr = NORMAL_MOWING;
        // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
      ShowMessageln("Mowing in Half lane width");
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
        ShowMessage("Total distance drive ");
        ShowMessage(totalDistDrive / 100);
        ShowMessageln(" meters ");
        ShowMessage("Total duration ");
        ShowMessage(int(millis() - stateStartTime) / 1000);
        ShowMessageln(" secondes ");
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
        ShowMessageln(" mowPatternCurr  change ");
        mowPatternCurr = (mowPatternCurr + 1) % 2; //change the pattern each x minutes
        mowPatternDuration = 0;
      }
      justChangeLaneDir = !justChangeLaneDir;  //use to know if the lane is not limit distance
      UseAccelLeft = 0;
      UseBrakeLeft = 1;
      UseAccelRight = 0;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm * 0.7 ; //perimeterSpeedCoeff reduce speed near the wire to 70%
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
      //bber500 to stop immediatly
      stateEndOdometryRight = odometryRight;// + (int)(odometryTicksPerCm / 6);
      stateEndOdometryLeft = odometryLeft;// + (int)(odometryTicksPerCm / 6);
      OdoRampCompute();

      break;

    case STATE_PERI_STOP_TOTRACK:
      //bber100 err here
      if (statusCurr == TRACK_TO_START) {
        if (mowPatternCurr == MOW_WIRE) {
          motorMowEnable = true; //time to start the blade
          statusCurr = WIRE_MOWING;
          // if (RaspberryPIUse) MyRpi.SendStatusToPi();
        }
      }
      else if (statusCurr == WIRE_MOWING) {
        motorMowEnable = true; //time to start the blade
      }
      else {
        statusCurr = BACK_TO_STATION;
        // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
      //imu.run(); //31/08/19 In peritrack the imu is stop so try to add this to start it now and avoid imu tilt error (occur once per week or less) ??????
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
        // if (RaspberryPIUse) MyRpi.SendStatusToPi();
        //startByTimer = false; // ?? not here                         cancel because we have reach the start point and avoid repeat search entry
        justChangeLaneDir = false; //the first lane need to be distance control
        perimeterUse = false; //disable the perimeter use to leave the area
        ShowMessageln("Stop to read the perimeter wire");
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
      ShowMessage("Actual Heading ");
      //ShowMessageln(imu.ypr.yaw * 180 / PI);
      //remoteDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      ShowMessage("New Remote Heading ");
      ShowMessageln(remoteDriveHeading * 180 / PI);
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
      ShowMessage("Actual Heading ");
      //ShowMessageln(imu.ypr.yaw * 180 / PI);
      //remoteDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      ShowMessage("New Remote Heading ");
      ShowMessageln(remoteDriveHeading * 180 / PI);
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
      //when the raspberry receive this new status it start the sender with the correct area sigcode
      totalDistDrive = 0; //reset the distance to track on the new area
      perimeterUse = true;
      ShowMessageln("Start to read the Perimeter wire");
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
      ShowMessage("Actual Heading ");
      //ShowMessageln(imu.ypr.yaw * 180 / PI);
      //periFindDriveHeading = scalePI(imu.ypr.yaw + newtagRotAngle1Radian);
      ShowMessage("New PeriFind Heading ");
      ShowMessageln(periFindDriveHeading * 180 / PI);
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
      //highGrassDetect = false;
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      motorLeftSpeedRpmSet = motorSpeedMaxRpm  ;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm  ;
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
        //ShowMessageln(R);
        R = R + (float)(odometryWheelBaseCm / 2);
        //ShowMessageln(R);
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
            ShowMessage("MOW SPIRALE R ");
            ShowMessage(R);
            ShowMessage(" Tmp ");
            ShowMessage(Tmp);
            ShowMessage(" Tmp1 ");
            ShowMessage(Tmp1);
            ShowMessage(" motorLeftSpeedRpmSet ");
            ShowMessage(motorLeftSpeedRpmSet);
            ShowMessage(" motorRightSpeedRpmSet ");
            ShowMessage(motorRightSpeedRpmSet);
            ShowMessage(" stateEndOdometryRight ");
            ShowMessage(stateEndOdometryRight);
            ShowMessage(" stateEndOdometryLeft ");
            ShowMessage(stateEndOdometryLeft);
            ShowMessage(" spiraleNbTurn ");
            ShowMessageln(spiraleNbTurn);

      */



      OdoRampCompute();
      spiraleNbTurn = spiraleNbTurn + 1;

      break;

    case STATE_PERI_OUT_REV: //in normal mowing reverse after the wire trigger
      //setBeeper(0, 0, 0, 0, 0);
      //perimeter.lastInsideTime[0] = millis(); //use to avoid perimetertimeout when mower outside perimeter
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
        // if (mowPatternCurr == MOW_ZIGZAG) AngleRotate = imu.scale180(imuDriveHeading + 135); //need limit value to valib the rebon
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
        // if (mowPatternCurr == MOW_ZIGZAG) AngleRotate = imu.scale180(imuDriveHeading - 135); //need limit value to valib the rebon
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
      //perimeter.lastInsideTime[0] = millis(); //use to avoid perimetertimeout when mower outside perimeter
      if (stateCurr == STATE_WAIT_AND_REPEAT) {
        RollToInsideQty = RollToInsideQty + 1;
        ShowMessage("Not Inside roll nb: ");
        ShowMessageln(RollToInsideQty);
      }
      else {
        RollToInsideQty = 0;
        ShowMessage("Find Inside roll nb: ");
        ShowMessageln(RollToInsideQty);
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
        ShowMessage("Hight grass detected actual halfLaneNb ");
        ShowMessageln(halfLaneNb);
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
      UseAccelLeft = 1;
      UseBrakeLeft = 1;
      UseAccelRight = 1;
      UseBrakeRight = 1;
      OdoRampCompute();

      break;

    case STATE_TEST_MOTOR:
      statusCurr = TESTING;
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
        ShowMessage(" imu.comYaw ");
        ShowMessage(abs(100 * imu.comYaw));
        ShowMessage(" imu.ypr.yaw ");
        ShowMessage(abs(100 * imu.ypr.yaw));
        ShowMessage(" distancePI(imu.comYaw, imu.ypr.yaw) ");
        ShowMessageln(distancePI(imu.comYaw, imu.ypr.yaw));
      */

      /*
            if (distancePI(imu.comYaw, yawCiblePos * PI / 180) > 0) { //rotate in the nearest direction
              actualRollDirToCalibrate = RIGHT;
              //ShowMessageln(" >>> >>> >>> >>> >>> >>> 0");
              motorLeftSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100 ;
              motorRightSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
              stateEndOdometryRight = odometryRight - (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
              stateEndOdometryLeft = odometryLeft + (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
            }
            else
            {
              actualRollDirToCalibrate = LEFT;
              //ShowMessageln(" <<< <<< <<< <<< <<< << 0");
              motorLeftSpeedRpmSet = -motorSpeedMaxRpm * compassRollSpeedCoeff / 100 ;
              motorRightSpeedRpmSet = motorSpeedMaxRpm * compassRollSpeedCoeff / 100;
              stateEndOdometryRight = odometryRight + (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
              stateEndOdometryLeft = odometryLeft - (int)(odometryTicksPerCm *  4 * PI * odometryWheelBaseCm );
            }
      */

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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
      break;
    case STATE_REMOTE:
      statusCurr = REMOTE;
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();

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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
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
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
      ShowMessage("Area In Mowing ");
      ShowMessage(areaInMowing);
      ShowMessage(" Area To Go ");
      ShowMessageln(areaToGo);

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
      PeriOdoIslandDiff =  odometryRight - odometryLeft;
      break;

    case STATE_WAIT_AND_REPEAT:
      //ShowMessageln("WAIT AND REPEAT  ");

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
  //ShowMessage (F(statusNames[statusCurr]));
  //ShowMessage (" / ");
  //ShowMessageln (F(stateNames[stateCurr]));
  ShowMessage (F(statusNames[statusCurr]));
  ShowMessage (" / ");
  ShowMessageln (F(stateNames[stateCurr]));







  //ShowMessage (" Dir ");
  //ShowMessage (rollDir);
  //ShowMessage (" State changed at ");
  //ShowMessage (stateStartTime);
  //ShowMessage (" From state ");
  //ShowMessageln (F(stateNames[stateLast]));

}

void Robot::ShowMessage(String message) {
  Console.print (message);
  if (ConsoleToPfod) {
    Bluetooth.print (message);
  }
}
void Robot::ShowMessageln(String message) {
  Console.println(message);
  if (ConsoleToPfod) {
    Bluetooth.println(message);
  }
}

void Robot::ShowMessage(float value) {
  Console.print (value);
  if (ConsoleToPfod) {
    Bluetooth.print (value);
  }
}
void Robot::ShowMessageln(float value) {
  Console.println(value);
  if (ConsoleToPfod) {
    Bluetooth.println(value);
  }
}

// check battery voltage and decide what to do


void Robot::checkBattery() {

  if ((millis() < nextTimeCheckBattery) || (millis() < 30000)) return; //  wait 30 sec after the initial power on before first check to avoid read bad battery voltage
  nextTimeCheckBattery = millis() + 1000; //if change need to adjust the line idleTimeSec= idleTimeSec+1;

  if (batMonitor) {
    // if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_OFF) && (stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING))  {
    if ((batVoltage < batSwitchOffIfBelow) && (stateCurr != STATE_OFF))   {
      ShowMessage(F("Batterie Voltage : "));
      ShowMessage(batVoltage);
      ShowMessage(F(" -- > Switch OFF Voltage : "));
      ShowMessage(batSwitchOffIfBelow);
      ShowMessageln(F("  Bat Voltage is very low the state is changed to OFF, so the undervoltage timer start"));
      addErrorCounter(ERR_BATTERY);
      //setBeeper(100, 25, 25, 200, 0 );
      setNextState(STATE_OFF, 0);
    }
    else if ((batVoltage < batGoHomeIfBelow) && (stateCurr == STATE_FORWARD_ODO) && (perimeterUse)) {    //actualy in mowing mode with station and perimeter
      ShowMessage(F("Batterie Voltage : "));
      ShowMessage(batVoltage);
      ShowMessage(F(" -- > Minimum Mowing Voltage : "));
      ShowMessageln(batGoHomeIfBelow);
      ShowMessageln(F(" Bat Voltage is low : The mower search the charging Station"));
      //setBeeper(100, 25, 25, 200, 0 );
      statusCurr = BACK_TO_STATION;
      areaToGo = 1;
      // if (RaspberryPIUse) MyRpi.SendStatusToPi();
      //periFindDriveHeading = imu.ypr.yaw;
      setNextState(STATE_PERI_FIND, 0);
    }


    // if robot is OFF or Error  we can start to count before shutdown
    if ( (stateCurr == STATE_OFF) || (stateCurr == STATE_ERROR)) {
      /*
        ShowMessage("Count before power OFF  ");
        ShowMessage(idleTimeSec);
        ShowMessage(" / ");
        ShowMessageln(batSwitchOffIfIdle * 60);
      */
      if (idleTimeSec != BATTERY_SW_OFF) { // battery already switched off?
        idleTimeSec = idleTimeSec + 1; // add 1 second idle time because check only each 1 secondes
        if (idleTimeSec > batSwitchOffIfIdle * 60) {

          // if (RaspberryPIUse) {
          ShowMessageln(F("Battery IDLE trigger "));
          ShowMessageln(F("PCB power OFF after 30 secondes Wait Until PI Stop "));
          //MyRpi.sendCommandToPi("PowerOffPi");
          delayWithWatchdog(30000);//wait 30Sec  until pi is OFF or the USB native power again the due and the undervoltage never switch OFF
        }
        else
        {
          ShowMessageln(F("PCB power OFF immediatly"));
        }
        setBeeper(200, 50, 50, 200, 100 );
        loadSaveErrorCounters(false); // saves error counters
        loadSaveRobotStats(false);    // saves robot stats
        idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
        ShowMessageln(F("BATTERY switching OFF"));
        setActuator(ACT_BATTERY_SW, 0);  // switch off battery
      }
    }
  }
  else
  {
    resetIdleTime();
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
  /*
    if (isnan(statsBatteryChargingCapacityTrip)) statsBatteryChargingCapacityTrip = 0;
    if (isnan(statsBatteryChargingCounterTotal)) statsBatteryChargingCounterTotal = 0; // for first run ensures that the counter is 0
    if (isnan(statsBatteryChargingCapacityTotal)) statsBatteryChargingCapacityTotal = 0; // for first run ensures that the counter is 0
  */
  if (statsBatteryChargingCapacityTotal <= 0 || statsBatteryChargingCounterTotal == 0) statsBatteryChargingCapacityAverage = 0; // make sure that there is no dividing by zero
  else statsBatteryChargingCapacityAverage = statsBatteryChargingCapacityTotal / statsBatteryChargingCounterTotal;

  //----------------new stats goes here------------------------------------------------------
  //mowPatternJustChange;
  mowPatternDuration++;


}


void Robot::reverseOrBidir(byte aRollDir) {

  if (stateCurr == STATE_PERI_OUT_ROLL_TOINSIDE) {
    ShowMessageln("Bumper hit ! try roll in other dir");
    setNextState(STATE_WAIT_AND_REPEAT, aRollDir);
    return;
  }

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
      ShowMessageln("Warning  motorMowPower >= 0.8 * motorMowPowerMax ");
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
    ShowMessage("Warning  motorMowPower >= motorMowPowerMax and Counter time is ");
    ShowMessageln(motorMowSenseCounter);
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
        ShowMessageln("Time to restart the mow motor after the 60 secondes pause");
      }
    }
  }
  //need to check this
  if (motorMowSenseCounter >= 10) { //ignore motorMowPower for 1 seconds
    motorMowEnable = false;
    ShowMessageln("Motor mow power overload. Motor STOP and try to start again after 1 minute");
    addErrorCounter(ERR_MOW_SENSE);
    lastTimeMotorMowStuck = millis();
  }

  //bb add test current in manual mode and stop immediatly
  if (statusCurr == MANUAL) {
    if (motorLeftPower >= 0.8 * motorPowerMax) {
      ShowMessage("Motor Left power is 80 % of the max, value --> ");
      ShowMessageln(motorLeftPower);
      setMotorPWM( 0, 0, false );
      setNextState(STATE_OFF, 0);

    }
    if (motorRightPower >= 0.8 * motorPowerMax) {
      ShowMessage("Motor Right power is 80 % of the max, value --> ");
      ShowMessageln(motorRightPower);
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
      ShowMessage("Motor Right power is 80 % of the max, value --> ");
      ShowMessageln(motorRightPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          ShowMessageln("Power motor left warning ");
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
      ShowMessage("Warning: Motor Right power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      ShowMessageln(motorRightSenseCounter);

    }


    //Motor left****************************************************************
    //First react test to 80 % powerMax
    if (motorLeftPower >= 0.8 * motorPowerMax)
    {
      motorLeftSenseCounter++;
      setBeeper(1000, 50, 50, 100, 50);
      setMotorPWM( 0, 0, false );
      ShowMessage("Motor Left power is 80 % of the max, value --> ");
      ShowMessageln(motorLeftPower);

      if (stateCurr != STATE_ERROR) {
        if ((stateCurr == STATE_PERI_TRACK) || (stateCurr == STATE_PERI_FIND)) {
          ShowMessageln("Power motor left warning ");
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
      ShowMessage("Warning: Motor Left power over 100% , Max possible 10 time in 1 seconde. Actual count --> ");
      ShowMessageln(motorLeftSenseCounter);
    }
    //final test on the counter to generate the error and stop the mower
    if (motorLeftSenseCounter >= 10) { //the motor is stuck for more than 1 seconde 10 * 100 ms go to error.
      ShowMessage("Fatal Error: Motor Left power over 100% for more than 1 seconde last power --> ");
      ShowMessageln(motorLeftPower);
      addErrorCounter(ERR_MOTOR_LEFT);
      setMotorPWM( 0, 0, false );
      setNextState(STATE_ERROR, 0);
    }
    if (motorRightSenseCounter >= 10) { //the motor is stuck for more than 1 seconde go to error.
      ShowMessage("Fatal Error: Motor Right power over 100% for more than 1 seconde last power --> ");
      ShowMessageln(motorRightPower);
      addErrorCounter(ERR_MOTOR_RIGHT);
      setMotorPWM( 0, 0, false );
      setNextState(STATE_ERROR, 0);
    }

  } //motorpower ignore time

}

// check bumpers
void Robot::checkBumpers() {
  if ((millis() < 3000) || (!bumperUse)) return;

  /*
    if (stateCurr=STATE_PERI_OUT_ROLL_TOINSIDE){
    if (bumperLeft) {
      rollDir=RIGHT;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
      setMotorPWM( 0, 0, false );

      return;
    }
    if (bumperRight){
      rollDir=LEFT;
      motorLeftRpmCurr = motorRightRpmCurr = 0 ;
      motorLeftPWMCurr = motorRightPWMCurr = 0;
      setMotorPWM( 0, 0, false );

      return;
    }



    }

  */

  if ((bumperLeft || bumperRight)) {
    if (statusCurr == MANUAL) {
      ShowMessageln("Bumper trigger in Manual mode ?????????");
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
        ShowMessageln("Bumper left trigger");
        reverseOrBidir(LEFT);
      } else {
        ShowMessageln("Bumper right trigger");
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
      ShowMessageln("Drop trigger in Manual mode ?????????");
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
    ShowMessageln("Bump on Something check if it's the station");
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
      ShowMessageln("Detect a voltage on charging contact check if it's the station");
      setNextState(STATE_STATION_CHECK, rollDir);
    }
  }
}

//bber401
void Robot::checkStuckOnIsland() {
  //6 * is a test value
  if ((odometryRight - odometryLeft) - PeriOdoIslandDiff > 6 * odometryTicksPerRevolution) {
    ShowMessageln("Right wheel is 6 full revolution more than left one --> Island  ??? ");
    newtagRotAngle1 = 90;
    setNextState(STATE_PERI_STOP_TOROLL, 0);
    return;
  }
}

// check perimeter as a boundary
void Robot::checkPerimeterBoundary() {

  if ((millis() >= nextTimeRotationChange) && (stateCurr == STATE_FORWARD_ODO) && (mowPatternCurr != MOW_LANES)) {// change only when in straight line and random mode
    nextTimeRotationChange = millis() + 600000;  // in random change each 10 minutes
    if (rollDir == LEFT) rollDir = RIGHT; //invert the next rotate
    else rollDir = LEFT;
    ShowMessage(millis());
    ShowMessageln(" Rotation direction Left / Right change ");
  }
  //bber2
  if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_MOW_SPIRALE) ) {
    //bber200
    //speed coeff between 0.7 and 1 according 50% of perimetermagmax
    if ((millis() >= nextTimeCheckperimeterSpeedCoeff) && (reduceSpeedNearPerimeter)) {
      //int miniValue = (int)perimeterMagMaxValue / 2;
      // perimeterSpeedCoeff = (float) map(perimeter.getSmoothMagnitude(0), miniValue, perimeterMagMaxValue, 100, 70) / 100;
      if (perimeterSpeedCoeff < 0.7) {
        perimeterSpeedCoeff = 0.7;
        nextTimeCheckperimeterSpeedCoeff = millis() + 500; //avoid speed coeff increase when mower go accross the wire
      }
      else
      {
        nextTimeCheckperimeterSpeedCoeff = millis() + 15;
      }
      if (perimeterSpeedCoeff > 1) perimeterSpeedCoeff = 1;
    }


    if (perimeterTriggerTime != 0) {
      if (millis() >= perimeterTriggerTime) {
        perimeterTriggerTime = 0;
        ShowMessage(F("Perimeter trigger at : "));
        ShowMessageln(millis());
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
        ShowMessageln("Pourquoi je suis la ?? ?? ?? ?? ?? ?? ?? ?? ");
        setMotorPWM( 0, 0, false );
        setNextState(STATE_PERI_OUT_REV, rollDir);
        return;

      }
    }
  }

}






void Robot::checkRain() {
  if (!rainUse) return;
  if (rain) {
    ShowMessageln(F("RAIN"));
    areaToGo = 1;
    if (perimeterUse) {
      // periFindDriveHeading = imu.ypr.yaw;
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
      ShowMessageln("Sonar reduce speed on tracking for 2 meters");
      whereToResetSpeed =  totalDistDrive + 200; // when a speed tag is read it's where the speed is back to maxpwm value

      nextTimeCheckSonar = millis() + 4000;  //wait before next reading
      // timeToResetSpeedPeri = millis() + 10000; //brake the tracking during 10 secondes
      ActualSpeedPeriPWM = MaxSpeedperiPwm * dockingSpeed / 100;
      trakBlockInnerWheel = 1; //don't want that a wheel reverse just before station check   /bber30


    }
  */
}

// check sonar
void Robot::checkSonar() {
  if (!sonarUse) return;
  if (millis() < nextTimeCheckSonar) return;
  nextTimeCheckSonar = millis() + 100;
  sonarSpeedCoeff = 1;
  /*
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
    setBeeper(1000, 500, 500, 60, 60);
    nextTimeCheckSonar = millis() + 1500;  //wait before next reading

    **************************if sonar during spirale reinit spirale variable*****************
    spiraleNbTurn = 0;
    halfLaneNb = 0;
    highGrassDetect = false; //stop the spirale
    *********************************************************************************
    if ((stateCurr == STATE_FORWARD_ODO) || (stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_MOW_SPIRALE)) {
      //avoid the mower move when testing
      if ((sonarDistCenter != NO_ECHO) && (sonarDistCenter < sonarTriggerBelow)) {  //center
        //bber200
        if (!sonarLikeBumper) {
          sonarSpeedCoeff = 0.80;
          nextTimeCheckSonar = millis() + 3000;
        }
        else {

          distToObstacle =  sonarDistCenter;
          ShowMessage("Sonar Center Trigger at cm : ");
          ShowMessageln (distToObstacle);
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
          ShowMessage("Sonar Right Trigger at cm : ");
          ShowMessageln (distToObstacle);
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
          ShowMessage("Sonar Left Trigger at cm : ");
          ShowMessageln (distToObstacle);
          if (mowPatternCurr == MOW_LANES) setNextState(STATE_SONAR_TRIG, rollDir); //don't change the rotation if lane mowing
          else setNextState(STATE_SONAR_TRIG, RIGHT);
          return;
        }
      }
    }
    }
  */
}






// check IMU (tilt)
void Robot::checkTilt() {
  if (!imuUse) return;
  if (millis() < nextTimeCheckTilt) return;
  /*
    nextTimeCheckTilt = millis() + 50; // 5Hz same as nextTimeImu
    int pitchAngle = (imu.ypr.pitch / PI * 180.0);
    int rollAngle  = (imu.ypr.roll / PI * 180.0);
    //bber4
    if ( (stateCurr != STATE_OFF) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_STATION) && (stateCurr != STATE_STATION_CHARGING)) {
    if ( (abs(pitchAngle) > 40) || (abs(rollAngle) > 40) ) {
      nextTimeCheckTilt = millis() + 5000; // avoid repeat
      ShowMessage(F("Warning : IMU Roll / Tilt -- > "));
      ShowMessage(rollAngle);
      ShowMessage(F(" / "));
      ShowMessageln(pitchAngle);
      addErrorCounter(ERR_IMU_TILT);
      //bber500


      ShowMessageln("Motor mow STOP start again after 1 minute");
      motorMowEnable = false;
      lastTimeMotorMowStuck = millis();
      reverseOrBidir(rollDir);

    }
    }
  */
}

// check if mower is stuck ToDo: take HDOP into consideration if gpsSpeed is reliable
void Robot::checkIfStuck() {
  /*
    if (millis() < nextTimeCheckIfStuck) return;
    nextTimeCheckIfStuck = millis() + 500;

    if ((gpsUse) && (gps.hdop() < 500))  {
    //float gpsSpeedRead = gps.f_speed_kmph();
    float gpsSpeed = gps.f_speed_kmph();
    //bb
    //if (gpsSpeedIgnoreTime >= motorReverseTime) gpsSpeedIgnoreTime = motorReverseTime - 500;
    // low-pass filter
    // double accel = 0.1;
    // float gpsSpeed = (1.0-accel) * gpsSpeed + accel * gpsSpeedRead;
    // ShowMessageln(gpsSpeed);
    // ShowMessageln(robotIsStuckCounter);
    // ShowMessageln(errorCounter[ERR_STUCK]);
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
    ShowMessageln("Mower not stuck       MOW can start");
    // motorMowEnable = true;
    errorCounterMax[ERR_STUCK] = 0;
    }

    return;
    }

    if (robotIsStuckCounter >= 5) {
    motorMowEnable = false;
    if (errorCounterMax[ERR_STUCK] >= 3) {  // robot is definately stuck and unable to move
    ShowMessageln(F("Error : Mower is stuck"));
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
    //prevYawCalcOdo = imu.ypr.yaw;

  } else {
    // FIXME: theta should be old theta, not new theta?
    odometryX += avg_cm * sin(odometryTheta);
    odometryY += avg_cm * cos(odometryTheta);
  }


}
void Robot::readDHT22() {

}
void Robot::checkTimeout() {
  if (stateTime > motorForwTimeMax) {
    ShowMessageln("Timeout on state the mower run for a too long duration ???????????????????????");
    setNextState(STATE_PERI_OUT_STOP, !rollDir); // toggle roll dir
  }
}






void Robot::loop()  {
  stateTime = millis() - stateStartTime;
  int steer;

  //ADCMan.run();
  //if (perimeterUse) perimeter.run();
  /*
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
  */

  rc.readSerial();

  //readSensors();

  //checkRobotStats();
  //checkPerimeterBoundary();
  //calcOdometry();
  //checkOdometryFaults();
  //checkButton();
  //motorMowControl();
  //checkTilt();
  if ((stateCurr == STATE_PERI_OUT_STOP) && (statusCurr == NORMAL_MOWING)) { //read only timer here for fast processing on odo
    //checkTimer();
  }
  //beeper();
  if (stateCurr != STATE_PERI_TRACK) {
    //if ((stateCurr != STATE_STATION_CHARGING) || (stateCurr != STATE_STATION) || (stateCurr != STATE_PERI_TRACK)) {
    if ((imuUse) && (millis() >= nextTimeImuLoop)) {
      nextTimeImuLoop = millis() + 50;
      StartReadAt = millis();
      //imu.run();
      EndReadAt = millis();
      ReadDuration = EndReadAt - StartReadAt;
      if ( ReadDuration > 30) {
        ShowMessage("Error reading imu too long duration : ");
        ShowMessageln(ReadDuration);
        ShowMessageln ("IMU and RFID are DEACTIVATE Mow in safe mode");
        imuUse = false;
        rfidUse = false;
        addErrorCounter(ERR_IMU_COMM);
      }
    }
  }


  /*
    if ((Enable_Screen) && (millis() >= nextTimeScreen))   { // warning : refresh screen take 40 ms
      nextTimeScreen = millis() + 250;
      StartReadAt = millis();

      if ((statusCurr == WAIT) || (statusCurr == MANUAL) || (statusCurr == REMOTE) || (statusCurr == TESTING) || (statusCurr == WAITSIG2)) {
        MyScreen.refreshWaitScreen();
      }
      if ((statusCurr == NORMAL_MOWING) || (statusCurr == SPIRALE_MOWING) || (statusCurr == WIRE_MOWING)) {
        MyScreen.refreshMowScreen();
        nextTimeScreen = millis() + 500; // in mowing mode don't need a big refresh rate and avoid trouble on loop
      }
      if ((statusCurr == BACK_TO_STATION) || (statusCurr == TRACK_TO_START) ) {
        MyScreen.refreshTrackScreen();
        nextTimeScreen = millis() + 500;
      }
      if (statusCurr == IN_ERROR ) {
        MyScreen.refreshErrorScreen();
      }
      if (statusCurr == IN_STATION) {
        MyScreen.refreshStationScreen();
      }

      EndReadAt = millis();
      ReadDuration = EndReadAt - StartReadAt;
      //ShowMessage("Screen Duration ");
      //ShowMessageln(ReadDuration);



    }
  */

  if (millis() >= nextTimeInfo) {
    if ((millis() - nextTimeInfo > 250)) {
      if (developerActive) {
        ShowMessage("------ LOOP NOT OK DUE IS OVERLOAD -- Over 1 sec ");
        ShowMessageln((millis() - nextTimeInfo));
      }
    }
    nextTimeInfo = millis() + 1000; //1000
    printInfo(Console);
    checkErrorCounter();
    //if (stateCurr == STATE_REMOTE) printRemote();
    loopsPerSec = loopsPerSecCounter;
    loopsPerSecCounter = 0;
  }

  if (millis() >= nextTimePfodLoop) {
    nextTimePfodLoop = millis() + 100;
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
      //imuDriveHeading = imu.ypr.yaw / PI * 180;
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

      checkTimeout();
      motorControl();
      break;







    case STATE_FORWARD_ODO:
      // driving forward with odometry control



      motorControlOdo();


      //manage the imu////////////////////////////////////////////////////////////
      if (imuUse ) {
        //when findedYaw = 999 it's mean that the lane is changed and the imu need to be adjusted to the compass
        /*
          if ((findedYaw == 999) && (imu.ypr.yaw > 0) && ((millis() - stateStartTime) > 4000) && ((millis() - stateStartTime) < 5000) && (mowPatternCurr == MOW_LANES)) { //try to find compass yaw
          setNextState(STATE_STOP_TO_FIND_YAW, rollDir);
          return;
          }

          //-----------here and before reverse the mower is stop so mark a pause to autocalibrate DMP-----------
          if ((millis() > nextTimeToDmpAutoCalibration) && (mowPatternCurr == MOW_LANES) && (imu.ypr.yaw > 0) && ((millis() - stateStartTime) > 4000) && ((millis() - stateStartTime) < 5000)  ) {
          setNextState(STATE_STOP_TO_FIND_YAW, rollDir);
          return;

          }
        */
      }
      //-----------------------------------------------------------------------------
      ////////////////////////////////////////////////////////////////////////////

      //the normal state traitement alternatively the lenght is 300ml or 10 ml for example
      if ((odometryRight > stateEndOdometryRight) || (odometryLeft > stateEndOdometryLeft))
      {
        if ((mowPatternCurr == MOW_LANES) && (!justChangeLaneDir)) {
          ShowMessageln("MAX LANE LENGHT TRIGGER time to reverse");
          setNextState(STATE_PERI_OUT_STOP, rollDir);
        }
        else {
          ShowMessageln("more than 300 ML in straight line ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ?? ? ");
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
          ShowMessageln ("Warning can t escape_lane in time ");
        }
        setNextState(STATE_PERI_OUT_STOP, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_ROLL_WAIT: //not use ??
      if ((odometryLeft >= stateEndOdometryLeft) || (odometryRight <= stateEndOdometryRight)) {
        if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the 2 motor completly stop
          ShowMessage(" OdometryLeft ");
          ShowMessage(odometryLeft);
          ShowMessage(" / stateEndOdometryLeft ");
          ShowMessage(stateEndOdometryLeft);
          ShowMessage(" OdometryRight ");
          ShowMessage(odometryRight);
          ShowMessage(" / stateEndOdometryRight ");
          ShowMessage(stateEndOdometryRight);
          ShowMessage(" yawtofind ");
          ShowMessageln(findedYaw);
          ShowMessage(" odometry find the Opposit Yaw at ");
          //ShowMessageln((imu.ypr.yaw / PI * 180));
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
          ShowMessageln ("Warning can t PERI_OBSTACLE_REV in time ");
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
          ShowMessageln ("Warning can t PERI_OBSTACLE_ROLL in time ");
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
          ShowMessageln ("Warning can t PERI_OBSTACLE_FORW in time ");
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
        //periFindDriveHeading = imu.ypr.yaw;
        setNextState(STATE_PERI_FIND, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t PERI_OBSTACLE_AVOID in time ");
        }
        //periFindDriveHeading = imu.ypr.yaw;
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
          ShowMessageln ("Warning can t reverse in time ");
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
          ShowMessageln ("Warning can t roll in time ");
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
          ShowMessageln ("Warning can t roll in time ");
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
          ShowMessageln ("Warning can t roll in time ");
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
          ShowMessageln ("Warning can t roll in time ");
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
          ShowMessageln ("Warning can t DRIVE1_TO_NEWAREA in time ");
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
          ShowMessageln ("Warning can t DRIVE2_TO_NEWAREA in time ");
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
          ShowMessageln ("Warning can t  stop ON BUMPER in time ");
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
          //  smoothPeriMag = perimeter.getSmoothMagnitude(0);
          ShowMessage("SmoothMagnitude =  ");
          ShowMessageln(smoothPeriMag);
          if ((perimeterInside) && (smoothPeriMag > 250)) //check if signal here and inside need a big value to be sure it is not only noise
          {
            if (areaToGo == 1) {
              statusCurr = BACK_TO_STATION; //if we are in the area1 it is to go to station
              // periFindDriveHeading = imu.ypr.yaw;
            }
            else
            {
              areaInMowing = areaToGo;
              statusCurr = TRACK_TO_START;
            }
            // if (RaspberryPIUse) MyRpi.SendStatusToPi();
            setNextState(STATE_PERI_FIND, rollDir);
            return;
          }
        }



      }
      if (millis() > (stateStartTime + 180000)) {  //wait the signal for 3 minutes
        ShowMessageln ("Warning can t find the signal for area2 ");
        setNextState(STATE_ERROR, rollDir);
      }

      break;


    case STATE_TEST_COMPASS:
      motorControlOdo();

      // YawActualDeg = (imu.ypr.yaw / PI * 180);
      /*
            if ((imu.distance180(YawActualDeg, yawToFind)) < 30) { //reduce speed to be sure stop
              PwmLeftSpeed = SpeedOdoMin / 2;
              PwmRightSpeed = -SpeedOdoMin / 2;
            }
            else {
              PwmLeftSpeed = SpeedOdoMin;
              PwmRightSpeed = -SpeedOdoMin;
            }
      */

      if ((YawActualDeg >= yawToFind - 1) && (YawActualDeg <= yawToFind + 1))  {
        ShowMessage(" OdometryLeft ");
        ShowMessage(odometryLeft);
        ShowMessage(" OdometryRight ");
        ShowMessage(odometryRight);
        ShowMessage(" Find YAW ****************************************  ");
        // ShowMessageln((imu.ypr.yaw / PI * 180));
        setNextState(STATE_OFF, rollDir);

      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        ShowMessageln ("Warning can t TestCompass in time ");
        setNextState(STATE_OFF, rollDir);
      }

      break;

    case STATE_CALIB_MOTOR_SPEED:
      motorControlOdo();
      if ((motorRightPWMCurr == 0 ) && (motorLeftPWMCurr == 0 )) {
        ShowMessageln("Calibration finish ");
        ShowMessage("Real State Duration : ");
        Tempovar = millis() - stateStartTime;
        ShowMessageln(Tempovar);
        ShowMessage("Compute Max State Duration : ");
        ShowMessageln(MaxOdoStateDuration);
        motorTickPerSecond = 1000 * stateEndOdometryRight / Tempovar;
        //bber400
        float motorRpmAvg;
        motorRpmAvg = 60000 * (stateEndOdometryRight / odometryTicksPerRevolution) / Tempovar;
        ShowMessage(" motorTickPerSecond : ");
        ShowMessageln(motorTickPerSecond);
        ShowMessage(" Average RPM : ");
        ShowMessageln(motorRpmAvg);
        setNextState(STATE_OFF, 0);
        motorSpeedMaxRpm = int(motorRpmAvg); //limit to 80% to have enought PWM
        saveUserSettings();
        return;
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        ShowMessageln ("Warning can t TestMotor in time please check your Odometry or speed setting ");
        setNextState(STATE_OFF, rollDir);
      }

      break;

    case STATE_TEST_MOTOR:
      motorControlOdo();
      if ((motorRightPWMCurr == 0 ) && (motorLeftPWMCurr == 0 )) {
        ShowMessageln("Test finish ");
        ShowMessage("Real State Duration : ");
        ShowMessageln(millis() - stateStartTime);
        ShowMessage("Compute Max State Duration : ");
        ShowMessageln(MaxOdoStateDuration);
        setNextState(STATE_OFF, 0);
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        ShowMessageln ("Warning can t TestMotor in time please check your Odometry or speed setting ");
        setNextState(STATE_OFF, rollDir);
      }
      break;




    case STATE_ROLL_TO_FIND_YAW:
      boolean finish_4rev;
      finish_4rev = false;
      motorControlOdo();

      //bber400
      if ((imuUse) && (millis() >= nextTimeImuLoop)) {
        nextTimeImuLoop = millis() + 50;
        //imu.run();
      }
      //it's ok

      //      if (CompassUse) {
      //        if ((yawToFind - 2 < (imu.comYaw / PI * 180)) && (yawToFind + 2 > (imu.comYaw / PI * 180)))  { //at +-2 degres
      //          findedYaw = (imu.comYaw / PI * 180);
      //          setNextState(STATE_STOP_CALIBRATE, rollDir);
      //          return;
      //        }
      //      }
      //      else //without compass
      //      {
      //        if ((yawToFind - 2 < (imu.ypr.yaw / PI * 180)) && (yawToFind + 2 > (imu.ypr.yaw / PI * 180)))  { //at +-2 degres
      //          findedYaw = (imu.ypr.yaw / PI * 180);
      //          setNextState(STATE_STOP_CALIBRATE, rollDir);
      //          return;
      //        }
      //      }


      //it's not ok
      if ((actualRollDirToCalibrate == RIGHT) && ((odometryRight <= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft))) finish_4rev = true;
      if ((actualRollDirToCalibrate == LEFT) && ((odometryRight >= stateEndOdometryRight) || (odometryLeft <= stateEndOdometryLeft))) finish_4rev = true;
      if (millis() > (stateStartTime + MaxOdoStateDuration + 6000)) finish_4rev = true;
      if (finish_4rev == true) {
        if (developerActive) {
          ShowMessageln ("Warning can t roll to find yaw The Compass is certainly not calibrate correctly ");
          ShowMessageln ("Continue to mow in random mode without compass ");
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
      ShowMessageln(odometryRight);

      if ((odometryRight <= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft))  setNextState(STATE_PERI_ROLL, rollDir);
      motorControlOdo();


      break;

    case STATE_PERI_FIND:
      // find perimeter
      if (!perimeterInside) {
        ShowMessageln("Not inside so start to track the wire");
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
      if (statusCurr == BACK_TO_STATION) {
        checkStuckOnIsland();
      }

      if (ActualSpeedPeriPWM != MaxSpeedperiPwm) {
        if (totalDistDrive > whereToResetSpeed) {
          ShowMessage("Distance OK, time to reset the initial Speed : ");
          ShowMessageln(ActualSpeedPeriPWM);
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
          ShowMessage("Distance OK, time to start mowing into new area ");
          ShowMessageln(areaInMowing);
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
          ShowMessageln("We are in station but ChargeVoltage is lost ??? ");
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
            ShowMessageln("End of charge by batfullcurrent Time to Restart PI and Due");
            autoReboot();
          }
          setNextState(STATE_STATION, 0);
          return;
        }
        if (millis() - stateStartTime > chargingTimeout)
        {
          ShowMessageln("End of charging duration check the batfullCurrent to try to stop before");
          if (autoResetActive) {
            ShowMessageln("Time to Restart PI and Due");
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
          ShowMessageln ("Warning can t  stop ON BUMPER in time ");
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
          ShowMessageln ("Warning can t peri out stop in time ");
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
          ShowMessageln ("Warning can t sonar trig in time ");
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
            //  findedYaw = (imu.ypr.yaw / PI * 180);
            setNextState(STATE_STOP_CALIBRATE, rollDir);
          }
        }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t peri out stop in time ");
        }
        if (laneUseNr == 1) yawToFind = yawSet1 ;
        if (laneUseNr == 2) yawToFind = yawSet2 ;
        if (laneUseNr == 3) yawToFind = yawSet3 ;
        if (CompassUse) {
          setNextState(STATE_ROLL_TO_FIND_YAW, rollDir);//if the motor can't rech the odocible in slope
        }
        else
        {
          //   findedYaw = (imu.ypr.yaw / PI * 180);
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
          ShowMessageln ("Warning can t stop to track in time ");
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
          ShowMessageln ("Warning can t stop to track in time ");
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
          ShowMessageln ("Warning can t stop  in time ");
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
          ShowMessageln ("Warning can t stop to track in time ");
        }
        setNextState(STATE_PERI_OUT_ROLL_TOTRACK, rollDir);//if the motor can't rech the odocible in slope
      }
      break;

    case STATE_AUTO_CALIBRATE:
      setBeeper(2000, 150, 150, 160, 50);
      if (millis() > nextTimeAddYawMedian) {  // compute a median of accelGyro and Compass  yaw
        //  compassYawMedian.add(imu.comYaw);
        // accelGyroYawMedian.add(imu.ypr.yaw);
        nextTimeAddYawMedian = millis() + 70;  // the value are read each 70ms
      }
      if (accelGyroYawMedian.getCount() > 56) { //we have the value of 4 secondes try to verify if the drift is less than x deg/sec
        ShowMessageln("4 sec of read value, verify if the drift is stop");
        if  (abs(accelGyroYawMedian.getHighest() - accelGyroYawMedian.getLowest()) < 4 * maxDriftPerSecond * PI / 180) { //drift is OK restart mowing
          if (CompassUse) {
            //  imu.CompassGyroOffset = distancePI( scalePI(accelGyroYawMedian.getMedian() -  imu.CompassGyroOffset), compassYawMedian.getMedian()); //change the Gyro offset according to Compass Yaw
          }
          else
          {
            //  imu.CompassGyroOffset = 0;
          }
          ShowMessageln("Drift is OK");
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
          ShowMessageln("Drift not Stop wait again 4 sec");
          compassYawMedian.clear();
          accelGyroYawMedian.clear();
        }

      }
      if (millis() > endTimeCalibration) { //we have wait enought and the result is not OK start to mow in random mode or make a total calibration
        mowPatternCurr = MOW_RANDOM;
        if (stopMotorDuringCalib) motorMowEnable = true;//stop the mow motor
        ShowMessageln("WAIT to stop Drift of GYRO : is not OK mowing Drift too important");
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
          ShowMessageln ("Warning can t  stop to calibrate in time ");
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
          ShowMessageln ("Warning cant stop before spirale in time");
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
          ShowMessageln ("Warning cant rotate right 360 in time ");
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
          ShowMessageln ("Warning can t  stop before next spire in time ");
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


      checkTimeout();

      //*************************************end of the spirale ***********************************************
      if ((spiraleNbTurn >= 8) || (!highGrassDetect)) {
        spiraleNbTurn = 0;
        highGrassDetect = false;
        setNextState(STATE_STOP_ON_BUMPER, RIGHT); //stop the spirale or setNextState(STATE_PERI_OUT_FORW, rollDir)
        return;
      }
      //********************************************************************************************
      if ((odometryRight >= stateEndOdometryRight) || (odometryLeft >= stateEndOdometryLeft) ) {
        if (!perimeterInside) {
          setNextState(STATE_STOP_ON_BUMPER, rollDir);
        }
        else
        {
          setNextState(STATE_NEXT_SPIRE, rollDir);
        }
        return;
      }


      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t MOW_SPIRALE in time ");
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
          ShowMessageln ("Warning can t peri out rev in time ");
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
          ShowMessageln ("Warning can t peri out roll in time ");
        }
        setNextState(STATE_PERI_OUT_FORW, rollDir);//if the motor can't rech the odocible in slope
      }


      break;

    case STATE_PERI_OUT_ROLL_TOINSIDE:
      checkBumpers();
      motorControlOdo();
      //bber17
      if (RollToInsideQty >= 10) {
        ShowMessageln("ERROR Mower is lost out the wire and can't find the signal. Roll to inside occur more than 10 Time");
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
          ShowMessageln ("Warning can t Roll to inside in time ");
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
          ShowMessageln ("Warning can t find perimeter Wire while PERI_OUT_ROLL_TOTRACK in time ");
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
          ShowMessageln ("Warning can t PERI_OUT_STOP_ROLL_TOTRACK in time ");
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
            if (!perimeterInside) {
              if (mowPatternCurr == MOW_LANES) {
                //bber601
                mowPatternDuration = mowPatternDurationMax - 3 ; //set the mow_random for the next 3 minutes
                ShowMessageln("We are in a corner mowPatternCurr change to Random for the next 3 minutes ");
                mowPatternCurr = MOW_RANDOM; //change the pattern each x minutes
                laneUseNr = laneUseNr + 1;
                if (laneUseNr > 3) laneUseNr = 1;
                findedYaw = 999;
                justChangeLaneDir = true;
                nextTimeToDmpAutoCalibration = millis(); // so the at the end of the next line a calibration occur

              }
              setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            }
            else setNextState(STATE_NEXT_LANE_FORW, rollDir);
          }
        }
      }
      else
      {
        if ((odometryRight >= stateEndOdometryRight) && (odometryLeft <= stateEndOdometryLeft))
        {
          if ((motorLeftPWMCurr == 0 ) && (motorRightPWMCurr == 0 )) { //wait until the left motor completly stop because rotation is inverted
            if (!perimeterInside) {
              if (mowPatternCurr == MOW_LANES) {
                //bber601

                mowPatternDuration = mowPatternDurationMax - 3 ; //set the mow_random for the next 3 minutes
                ShowMessageln("We are in a corner mowPatternCurr change to Random for the next 3 minutes ");
                mowPatternCurr = MOW_RANDOM; //change the pattern each x minutes
                laneUseNr = laneUseNr + 1;
                if (laneUseNr > 3) laneUseNr = 1;
                findedYaw = 999;
                justChangeLaneDir = true;
                nextTimeToDmpAutoCalibration = millis(); // so the at the end of the next line a calibration occur

              }


              setNextState(STATE_PERI_OUT_ROLL_TOINSIDE, rollDir);
            }
            else setNextState(STATE_NEXT_LANE_FORW, rollDir);
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {
        if (developerActive) {
          ShowMessageln ("Warning can t Roll1 by lane in time ");
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
          ShowMessageln ("Warning can t reach next lane in time ");
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
          ShowMessageln ("Warning can t make the roll2 in time ");
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
              ShowMessageln ("Charge Voltage detected ");
              setNextState(STATE_STATION, rollDir);// we are into the station
              return;
            }
            else {
              ShowMessageln ("No Voltage detected so certainly Obstacle ");
              setNextState(STATE_PERI_OBSTACLE_REV, rollDir);// not into the station so avoid obstacle
              return;
            }
          }
        }
      }
      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          ShowMessageln ("Warning can t make the station check in time ");
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
          ShowMessage ("Warning station rev not in time Max Compute duration in ms :");
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
          ShowMessageln ("Warning can t make the station roll in time ");
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
          // if (RaspberryPIUse) MyRpi.SendStatusToPi();
          setNextState(STATE_FORWARD_ODO, rollDir);
        }

      }

      if (millis() > (stateStartTime + MaxOdoStateDuration)) {//the motor have not enought power to reach the cible
        if (developerActive) {
          ShowMessageln ("Warning can t make the station forw in time ");
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
        ShowMessageln("Try to start at other location : We are not inside perimeter");
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
  //watchdogReset();
  //perimeter.speedTest();
  /*
    StartReadAt = millis();
    distance_find = sensor.readRangeSingleMillimeters();
    EndReadAt = millis();
    ReadDuration = EndReadAt - StartReadAt;
    ShowMessage("Dist :    ");
    ShowMessage(distance_find);
    ShowMessage("         Read Duration in ms ");
    ShowMessageln(ReadDuration);
  */
  delay(150);
}
