#include "mower.h"
#include "robot.h"
#include "RpiRemote.h"
#define N_FLOATS 4

uint16_t var_checksum;

String RpiCmd;


void RpiRemote::init() {
  RaspberryPIPort.begin(CONSOLE_BAUDRATE);

}
void RpiRemote::run() {
  readPi();
  if ((millis() >= nextTimeRaspberryPISendStat) && (millis() >= 15000)) { // start to send the stat only after 15 sec on power up to wait pi start
    nextTimeRaspberryPISendStat = millis() + 500;  //better to put 200
    RaspberryPISendStat();
  }
  if ((millis() >= nextTimeRaspberryPISendInfo) && (maxRepetInfoToPi > 0)) {
    maxRepetInfoToPi = maxRepetInfoToPi - 1;
    nextTimeRaspberryPISendInfo = millis() + delayInfoToPi;
    RaspberryPISendInfo();
  }
  if ((millis() >= nextTimeRaspberryPISendMot) && (maxRepetMotToPi > 0)) {
    maxRepetMotToPi = maxRepetMotToPi - 1;
    nextTimeRaspberryPISendMot = millis() + delayMotToPi;
    RaspberryPISendMot();
  }
  if ((millis() >= nextTimeRaspberryPISendMow) && (maxRepetMowToPi > 0)) {
    maxRepetMowToPi = maxRepetMowToPi - 1;
    nextTimeRaspberryPISendMow = millis() + delayMowToPi;
    RaspberryPISendMow();
  }
  if ((millis() >= nextTimeRaspberryPISendPeri) && (maxRepetPeriToPi > 0)) {
    maxRepetPeriToPi = maxRepetPeriToPi - 1;
    nextTimeRaspberryPISendPeri = millis() + delayPeriToPi;
    RaspberryPISendPeri();
  }
  if ((millis() >= nextTimeRaspberryPISendBat) && (maxRepetBatToPi > 0)) {
    maxRepetBatToPi = maxRepetBatToPi - 1;
    nextTimeRaspberryPISendBat = millis() + delayBatToPi;
    RaspberryPISendBat();
  }
  if ((millis() >= nextTimeRaspberryPISendByLane) && (maxRepetByLaneToPi > 0)) {
    maxRepetByLaneToPi = maxRepetByLaneToPi - 1;
    nextTimeRaspberryPISendByLane = millis() + delayByLaneToPi;
    RaspberryPISendByLane();
  }
  if ((millis() >= nextTimeRaspberryPISendImu) && (maxRepetImuToPi > 0)) {
    maxRepetImuToPi = maxRepetImuToPi - 1;
    nextTimeRaspberryPISendImu = millis() + delayImuToPi;
    RaspberryPISendImu();
  }
}

void RpiRemote::receivePiCommand (String ActuatorName, int value) {
  if (ActuatorName == "mowmotor") {
    if (value == 1) {
      robot->motorMowEnable = true;
    }
    else {
      robot->motorMowEnable = false;
    }
  }
}

void RpiRemote::sendCommandToPi(String stringLine) {
  String lineToSend;
  lineToSend = "RMCMD,";
  lineToSend = lineToSend + stringLine;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}


void RpiRemote::RaspberryPISendDebug (String data) {
  String lineToSend;
  lineToSend = "RMDEB,";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + data;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}




void RpiRemote::receivePiReqSetting (String Setting_page, int nb_page) {
  //send the timer setting to PI
  if (Setting_page == "Timer") {
    for (int i = 0; i <= 4; i++) {  // send the first 10 value of 5 Timers
      String lineToSend;
      lineToSend = "RMRET,";
      lineToSend = lineToSend + "Timer";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + i;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].active;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startTime.hour;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startTime.minute;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].stopTime.hour;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].stopTime.minute;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startDistance;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startMowPattern;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startNrLane;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startRollDir;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startLaneMaxlengh;
      lineToSend = lineToSend + ",";
      writePi(lineToSend);
    }

    //again a following for the area
    for (int i = 0; i <= 4; i++) {  // send the rest of 5 Timers
      String lineToSend;
      lineToSend = "RMRET,";
      lineToSend = lineToSend + "Timer1";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + i;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].startArea;;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + robot->timer[i].daysOfWeek;
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      lineToSend = lineToSend + "0";
      lineToSend = lineToSend + ",";
      writePi(lineToSend);
    }
  }
  //send only the date and time to the PI
  if (Setting_page == "Time") {
    String lineToSend;
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "Time";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "1";  //only one page
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->datetime.time.hour;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->datetime.time.minute;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->datetime.date.dayOfWeek;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->datetime.date.day;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->datetime.date.month;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->datetime.date.year;
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

  }

  //send only 2 pages motor setting but not use replace by all setting in 13 pages
  if (Setting_page == "Motor") {
    String lineToSend;
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "Motor";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "1";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorPowerMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSpeedMaxRpm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSpeedMaxPwm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorAccel;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorPowerIgnoreTime;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRollDegMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRollDegMin;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DistPeriOutRev;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DistPeriOutStop;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorLeftPID.Kp;
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    lineToSend = "RMRET,";
    lineToSend = lineToSend + "Motor";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "2";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorLeftPID.Ki;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorLeftPID.Kd;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorLeftSwapDir;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRightSwapDir;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRightOffsetFwd;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRightOffsetRev;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->SpeedOdoMin;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->SpeedOdoMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSenseLeftScale;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSenseRightScale;
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

  }
  //send 13 pages all setting
  if (Setting_page == "All") {
    String lineToSend;
    //page 1
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "1";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->developerActive;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorAccel;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSpeedMaxRpm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSpeedMaxPwm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorPowerMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSenseRightScale;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorSenseLeftScale;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRollDegMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRollDegMin;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DistPeriOutRev;
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    //page 2
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "2";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorPowerIgnoreTime;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorForwTimeMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorMowSpeedMaxPwm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorMowPowerMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorMowSpeedMinPwm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorMowSenseScale;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorLeftPID.Kp;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend +  robot->motorLeftPID.Ki;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend +  robot->motorLeftPID.Kd;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend +  robot->motorMowPID.Kp;
    lineToSend = lineToSend + ",";
    writePi(lineToSend);


    //page 3
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "3";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorMowPID.Ki; //0
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorMowPID.Kp;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorBiDirSpeedRatio1;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorBiDirSpeedRatio2;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorLeftSwapDir;  //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRightSwapDir;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->bumperUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->sonarUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->sonarCenterUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->sonarLeftUse;  //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);
    //page 4
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "4";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->sonarRightUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->sonarTriggerBelow;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeter.timedOutIfBelowSmag;  //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterTriggerMinSmag;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->trackingErrorTimeOut;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorTickPerSecond;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterOutRevTime;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterTrackRollTime;  //9
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterTrackRevTime; //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);


    //page 5
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "5";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterPID.Kp;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterPID.Ki;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterPID.Kd;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeter.signalCodeNo;  //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeter.swapCoilPolarityLeft;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeter.timeOutSecIfNotInside;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->trakBlockInnerWheel;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->lawnSensorUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->imuUse;  //9
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->stopMotorDuringCalib; //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    //page 6
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "6";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->imuDirPID.Kp;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->imuDirPID.Ki;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->imuDirPID.Kd;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->imuRollPID.Kp;  //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->imuRollPID.Ki;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->imuRollPID.Kd;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->remoteUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batMonitor;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batGoHomeIfBelow;  //9
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batSwitchOffIfBelow; //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    //page 7
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "7";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batSwitchOffIfIdle;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batFactor;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batChgFactor;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->chgSenseZero;  //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batSenseFactor;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->batFullCurrent;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->startChargingIfBelow;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->stationRevDist;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->stationRollAngle;  //9
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->stationForwDist; //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);


    //page 8
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "8";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->stationCheckDist;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->odometryUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->odometryTicksPerRevolution;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->odometryTicksPerCm;  //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->odometryWheelBaseCm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->autoResetActive;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->CompassUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->twoWayOdometrySensorUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->buttonUse;  //9
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->userSwitch1; //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    //page 9
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "9";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->userSwitch2;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->userSwitch3;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->timerUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->rainUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->gpsUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->stuckIfGpsSpeedBelow;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->gpsBaudrate;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->dropUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->statsOverride;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->bluetoothUse; //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);


    //page 10
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "10";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->esp8266Use;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->esp8266ConfigString;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->tiltUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->trackingPerimeterTransitionTimeOut; //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorMowForceOff;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->MaxSpeedperiPwm;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->RollTimeFor45Deg;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DistPeriObstacleAvoid;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->circleTimeForObstacle;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DistPeriOutRev; //10
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    //page 11
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "11";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRightOffsetFwd;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->motorRightOffsetRev;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeterMagMaxValue;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->SpeedOdoMin; //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->SpeedOdoMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawSet1;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawSet2;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawSet3;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawOppositeLane1RollRight;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawOppositeLane2RollRight;
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    //page 12
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "12";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawOppositeLane3RollRight;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawOppositeLane1RollLeft;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawOppositeLane2RollLeft;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->yawOppositeLane3RollLeft; //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DistBetweenLane;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->maxLenghtByLane;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeter.swapCoilPolarityRight;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->perimeter.read2Coil;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->maxDriftPerSecond; //9
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->delayBetweenTwoDmpAutocalib; //0
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

    //page 13
    lineToSend = "RMRET,";
    lineToSend = lineToSend + "All";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "13";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->maxDurationDmpAutocalib;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->mowPatternDurationMax;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DistPeriOutStop;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->DHT22Use; //4
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->RaspberryPIUse;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->sonarToFrontDist;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->UseBumperDock;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->dockingSpeed; //8
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "0";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "0";
    lineToSend = lineToSend + ",";
    writePi(lineToSend);

  }
}
void RpiRemote::RaspberryPISendMow () {
  String lineToSend;
  lineToSend = "RMMOW";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->motorMowPower;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->motorMowPWMCurr;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->batVoltage;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}
void RpiRemote::RaspberryPISendMot () {
  String lineToSend;
  lineToSend = "RMMOT";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->motorLeftPower;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->motorRightPower;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->motorLeftPWMCurr;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->motorRightPWMCurr;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->batVoltage;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}
void RpiRemote::RaspberryPISendPeri () {
  String lineToSend;
  lineToSend = "RMPER";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->perimeterMag;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->perimeterMagRight;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->areaInMowing;
  lineToSend = lineToSend + ",";

  writePi(lineToSend);
}
void RpiRemote::RaspberryPISendBat () {
  String lineToSend;
  lineToSend = "RMBAT";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->batVoltage;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->chgVoltage;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->chgCurrent;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}
void RpiRemote::RaspberryPISendByLane () {
  String lineToSend;
  lineToSend = "RMBYL";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->rollDir;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->laneUseNr;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "0";//robot->imu.ypr.yaw;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "0";//robot->imuDriveHeading;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}

void RpiRemote::RaspberryPISendImu () {
  String lineToSend;
  lineToSend = "RMIMU";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "0";//robot->imu.ypr.yaw * 180 / PI;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "0";//robot->imu.comYaw * 180 / PI;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}

void RpiRemote::RaspberryPISendInfo () {

  //Console.print(motorPowerMax);
  String lineToSend;
  lineToSend = "RMINF";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + VER;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->developerActive;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->statsOverride;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->statsMowTimeMinutesTrip;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->statsMowTimeHoursTotal;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->statsBatteryChargingCounterTotal;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->statsBatteryChargingCapacityTrip;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->statsBatteryChargingCapacityTotal / 1000;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}

void RpiRemote::SendStatusToPi () {
  //Console.print("New Status : ");
  //Console.println(robot->statusCurr);
  String lineToSend;
  lineToSend = "RMSTU,";
  lineToSend = lineToSend + robot->statusCurr;
  lineToSend = lineToSend + ",";
  if (robot->statusCurr == TRACK_TO_START) {
    lineToSend = lineToSend + robot->areaInMowing;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + robot->areaToGo;
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "3";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "4";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "5";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "6";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "7";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "8";
    lineToSend = lineToSend + ",";
  }
  else
  {
    lineToSend = lineToSend + "1";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "2";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "3";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "4";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "5";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "6";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "7";
    lineToSend = lineToSend + ",";
    lineToSend = lineToSend + "8";
    lineToSend = lineToSend + ",";
  }
  writePi(lineToSend);
}

void RpiRemote::SendRfidToPi () {
  //Console.print("New Status : ");
  //Console.println(robot->statusCurr);
  String lineToSend;
  lineToSend = "RMRFI,";
  lineToSend = lineToSend + robot->statusCurr;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->rfidTagFind;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "2";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "3";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "4";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "5";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "6";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "7";
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + "8";
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}

void RpiRemote::RaspberryPISendStat () {
  //Console.print(motorPowerMax);
  String lineToSend;
  lineToSend = "RMSTA,";
  lineToSend = lineToSend + millis();
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->stateCurr;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->odometryX;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->odometryY;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->prevYawCalcOdo;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->batVoltage;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->imu.ypr.yaw;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->imu.ypr.pitch;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->imu.ypr.roll;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->temperatureDht;
  lineToSend = lineToSend + ",";
  lineToSend = lineToSend + robot->loopsPerSec;
  lineToSend = lineToSend + ",";
  writePi(lineToSend);
}

void RpiRemote::setRobot(Robot *aRobot) {
  this->robot = aRobot;
}

void RpiRemote::writePi(String stringLine) {
  String lineToSend;
  uint8_t retour;
  retour = create_checksum(stringLine);
  lineToSend = "$" + stringLine + "*";
  if (retour < 16) {  //add the leading zero
    lineToSend = lineToSend + "0" + String(retour, HEX);
  }
  else {
    lineToSend = lineToSend + String(retour, HEX);
  }
  RaspberryPIPort.println(lineToSend);
  //watchdogReset();
  //Console.println(lineToSend);
}

void RpiRemote::readPi() {
  if (!RaspberryPIPort.available())
    return;
  int StartTrans = millis();
  while (RaspberryPIPort.available()) {
    //watchdogReset();
    char c = RaspberryPIPort.read();
    encode(c);
  }
}

RpiRemote::Tokeniser::Tokeniser(char* _str, char _token)
{
  str = _str;
  token = _token;
}

boolean RpiRemote::Tokeniser::next(char* out, int len)
{
  uint8_t count = 0;
  if (str[0] == 0)
    return false;
  while (true)
  {
    if (str[count] == '\0')
    {
      out[count] = '\0';
      str = &str[count];
      return true;
    }
    if (str[count] == token)
    {
      out[count] = '\0';
      count++;
      str = &str[count];
      return true;
    }
    if (count < len)
      out[count] = str[count];
    count++;
  }
  return false;
}

boolean RpiRemote::encode(char c)
{
  buf[pos] = c;
  pos++;
  if (c == '\n') //linefeed
  {
    //Console.println("..........FIND THE END LINE................");
    //Console.println(buf);
    boolean ret = process_buf();
    memset(buf, '\0', 120);
    pos = 0;
    return ret;
  }
  if (pos >= 120) //avoid a buffer overrun
  {
    Console.print (buf);
    Console.println("----------------- warning >120 char received  from PI------------");
    memset(buf, '\0', 120);
    pos = 0;
  }
  return false;
}

boolean RpiRemote::process_buf()
{
  if (!check_checksum()) //if checksum is bad
  {
    //Console.println("Error Checksum");
    //Console.println (buf);
    return false; //return
  }
  else {
    //Console.print("Checksum OK ");
    //Console.print (buf);
    //otherwise, what sort of message is it
    if (strncmp(buf, "$RMPFO", 6) == 0) read_pfo();
    if (strncmp(buf, "$RMSET", 6) == 0) readWrite_setting();
    if (strncmp(buf, "$RMVAR", 6) == 0) readWrite_var();
    if (strncmp(buf, "$RMCMD", 6) == 0) receive_command();
    if (strncmp(buf, "$RMREQ", 6) == 0) receive_request();

    return true;
  }
}


uint8_t RpiRemote::create_checksum(String lineOfString)
{
  char lineOfChar[lineOfString.length() + 1];
  lineOfString.toCharArray(lineOfChar, lineOfString.length() + 1); //need a char array to compute

  uint8_t XOR = 0;
  for (uint8_t posit = 0; posit < strlen(lineOfChar); posit++) {
    XOR = XOR ^ lineOfChar[posit];
  }

  return XOR;

}



boolean RpiRemote::check_checksum()
{

  if (buf[strlen(buf) - 5] == '*')
  {
    uint16_t sum = parse_hex(buf[strlen(buf) - 4]) * 16;
    sum += parse_hex(buf[strlen(buf) - 3]);
    //Serial.println(sum);
    var_checksum = sum;
    for (uint8_t i = 1; i < (strlen(buf) - 5); i++)
      sum ^= buf[i];
    if (sum != 0)
      return false;

    return true;
  }
  return false;
}


uint8_t RpiRemote::parse_hex(char c)
{
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A') + 10;
  return 0;
}

void RpiRemote::receive_request() {
  //$RMREQ,INF,2,0,3,0,0,0,*35

  // Console.print("Receive a request  --> ");
  // Console.println(buf);
  String messageType;
  int frequency ;
  int trigger ;
  int max_repetition ;
  int counter = 0;
  char token[12];
  Tokeniser tok(buf, ',');
  while (tok.next(token, 10))
  {
    switch (counter)
    {
      case 1:
        messageType = token;
        break;
      case 2:
        frequency = atoi(token);
        break;
      case 3:
        trigger = atoi(token);
        break;
      case 4:
        max_repetition = atoi(token);
        break;
    }
    counter++;
  }

  if (messageType == "INF") {
    delayInfoToPi = 1000 / frequency; // if freq=2 then read each 500ms
    maxRepetInfoToPi = max_repetition;
    nextTimeRaspberryPISendInfo = millis() + delayInfoToPi;
  }
  if (messageType == "MOT") {
    delayMotToPi = 1000 / frequency; // if freq=2 then read each 500ms
    maxRepetMotToPi = max_repetition;
    nextTimeRaspberryPISendMot = millis() + delayMotToPi;
  }
  if (messageType == "MOW") {
    delayMowToPi = 1000 / frequency; // if freq=2 then read each 500ms
    maxRepetMowToPi = max_repetition;
    nextTimeRaspberryPISendMow = millis() + delayMowToPi;
  }
  if (messageType == "PERI") {
    delayPeriToPi = 1000 / frequency; // if freq=2 then read each 500ms
    maxRepetPeriToPi = max_repetition;
    nextTimeRaspberryPISendPeri = millis() + delayPeriToPi;
  }
  if (messageType == "BAT") {
    delayBatToPi = 60000 / frequency; // Here use the minute instead of second to manage battery for long time
    maxRepetBatToPi = max_repetition;
    nextTimeRaspberryPISendBat = millis() + delayBatToPi;
  }
  if (messageType == "BYL") {
    delayByLaneToPi = 1000 / frequency;
    maxRepetByLaneToPi = max_repetition;
    nextTimeRaspberryPISendByLane = millis() + delayByLaneToPi;
  }
  if (messageType == "IMU") {
    delayImuToPi = 1000 / frequency;
    maxRepetImuToPi = max_repetition;
    nextTimeRaspberryPISendImu = millis() + delayImuToPi;
  }
}

void RpiRemote::receive_command() {
  //$RMCMD,mowmotor,1,0,0,4*62

  //Console.print("Receive a Actuator command  --> ");
  //Console.println(buf);
  String ActuatorName;
  int value ;

  int counter = 0;
  char token[12];
  Tokeniser tok(buf, ',');
  while (tok.next(token, 10))
  {
    switch (counter)
    {
      case 1:
        ActuatorName = token;
        break;
      case 2:
        value = atoi(token);
        break;

    }
    counter++;
  }
  receivePiCommand(ActuatorName, value);
}

void RpiRemote::readWrite_var() {  //can be use to change the value of 4 variable in one sentence
  //Console.print("Receive Read Write variable --> ");
  //Console.println(buf);

  char readOrWrite; //flag r or w
  char variable_name[4][30];
  char received_value[4][30];
  int counter = 0;
  char token[30];
  Tokeniser tok(buf, ',');
  while (tok.next(token, 30))
  {
    switch (counter)
    {
      case 1:
        {
          if (token[0] == 'w') readOrWrite = 'w';
          else readOrWrite = 'r';
        }
        break;
      case 2:
        strcpy(variable_name[0], token);
        break;

      case 3:
        strcpy(received_value[0] , token);
        break;
      case 4:
        strcpy(variable_name[1], token);
        break;

      case 5:
        strcpy(received_value[1] , token);
        break;
      case 6:
        strcpy(variable_name[2], token);
        break;

      case 7:
        strcpy(received_value[2] , token);
        break;
      case 8:
        strcpy(variable_name[3], token);
        break;
      case 9:
        strcpy(received_value[3] , token);
        break;
    }
    counter++;
  }

  if (readOrWrite == 'w') {

    for (int i = 0; i <= 3; i++) {  // read the 4 values and check to adjust
      //here need to add all the variable that can send the raspberry and what to do
      if (strncmp(variable_name[i], "mowPatternCurr", 20) == 0)  robot->mowPatternCurr = atoi(received_value[i]);
      if (strncmp(variable_name[i], "laneUseNr", 20) == 0)  robot->laneUseNr = atoi(received_value[i]);
      if (strncmp(variable_name[i], "rollDir", 20) == 0)  robot->rollDir = atoi(received_value[i]);
      if (strncmp(variable_name[i], "whereToStart", 20) == 0)  robot->whereToStart = atoi(received_value[i]);
      if (strncmp(variable_name[i], "actualLenghtByLane", 20) == 0) robot->actualLenghtByLane = atoi(received_value[i]);
      if (strncmp(variable_name[i], "motorLeftSpeedRpmSet", 20) == 0)  robot->motorLeftSpeedRpmSet = atoi(received_value[i]);
      if (strncmp(variable_name[i], "motorLeftSpeedRpmSet", 20) == 0)  robot->motorLeftSpeedRpmSet = atoi(received_value[i]);
      if (strncmp(variable_name[i], "newtagRotAngle1", 20) == 0) robot->newtagRotAngle1 = atoi(received_value[i]);
      if (strncmp(variable_name[i], "newtagRotAngle2", 20) == 0) robot->newtagRotAngle2 = atoi(received_value[i]);
      if (strncmp(variable_name[i], "motorSpeedMaxPwm", 20) == 0)  robot->motorSpeedMaxPwm = atoi(received_value[i]);
      //bber50
      if (strncmp(variable_name[i], "ActualSpeedPeriPWM", 20) == 0) {
        robot->ActualSpeedPeriPWM = atoi(received_value[i]);
        Console.print("Set New perimeter tracking speed  ");
        Console.println(robot->ActualSpeedPeriPWM);

      }


      //if (strncmp(variable_name[i], "newtagDistance1", 20) == 0)  robot->newtagDistance1 = atoi(received_value[i]);
      if (strncmp(variable_name[i], "newtagDistance2", 20) == 0)  robot->newtagDistance2 = atoi(received_value[i]);
      if (strncmp(variable_name[i], "areaToGo", 20) == 0)  robot->areaToGo = atoi(received_value[i]);
      if (strncmp(variable_name[i], "stateCurr", 20) == 0)  robot->stateCurr = atoi(received_value[i]);
      if (strncmp(variable_name[i], "statusCurr", 20) == 0)  robot->statusCurr = atoi(received_value[i]);
      if (strncmp(variable_name[i], "nextTimeTimer", 20) == 0)  robot->nextTimeTimer = atoi(received_value[i]);

      //bber50
      if (strncmp(variable_name[i], "newtagDistance1", 20) == 0)
      {
        robot->newtagDistance1 = atoi(received_value[i]);
        robot->whereToResetSpeed =  robot->totalDistDrive + robot->newtagDistance1; // when a speed tag is read it's where the speed is back to maxpwm value
        Console.print("Change speed for ");
        Console.print(robot->newtagDistance1);
        Console.println(" centimeters");


      }


      if (strncmp(variable_name[i], "areaInMowing", 20) == 0) {
        robot->areaInMowing = atoi(received_value[i]);
        robot->perimeter.changeArea(robot->areaInMowing);
      }

      if (strncmp(variable_name[i], "motorLeftSwapDir", 20) == 0) {
        if (strncmp(received_value[i], "0", 1) == 0) robot->motorLeftSwapDir = false;
        else robot->motorLeftSwapDir = true;
      }
      if (strncmp(variable_name[i], "motorRightSwapDir", 20) == 0) {
        if (strncmp(received_value[i], "0", 1) == 0) robot->motorRightSwapDir = false;
        else robot->motorRightSwapDir = true;
      }
      if (strncmp(variable_name[i], "gpsUse", 20) == 0) {
        if (strncmp(received_value[i], "0", 1) == 0)
        {
          robot->gpsUse = false;
          robot->gpsReady=false;
          GpsPort.flush();
          GpsPort.end();
        }
        else
        {
          robot->gpsUse = true;
          robot->gps.init();
          
        }
      }
    }
  }
  if (readOrWrite == 'r') {

  }

}




void RpiRemote::readWrite_setting()
{
  //Console.print("Receive Read Write setting command  --> ");
  //Console.println(buf);
  String Setting_page;
  char readOrWrite; //flag r or w
  int nr_page ;
  float val[9]; // 10 values from 0 to 9
  int counter = 0;
  char token[12];
  Tokeniser tok(buf, ',');
  while (tok.next(token, 10))
  {
    switch (counter)
    {
      case 1:
        Setting_page = token;
        break;
      case 2:
        if (token[0] == 'w') readOrWrite = 'w';
        else readOrWrite = 'r';
        break;
      case 3:
        nr_page = atoi(token);
        break;
      case 4:
        val[0] = atof(token);
        break;
      case 5:
        val[1] = atof(token);
        break;
      case 6:
        val[2] = atof(token);
        break;
      case 7:
        val[3] = atof(token);
        break;
      case 8:
        val[4] = atof(token);
        break;
      case 9:
        val[5] = atof(token);
        break;
      case 10:
        val[6] = atof(token);
        break;
      case 11:
        val[7] = atof(token);
        break;
      case 12:
        val[8] = atof(token);
        break;
      case 13:
        val[9] = atof(token);
        break;



    }
    counter++;
  }

  if (readOrWrite == 'w') {

    if (Setting_page == "Timer") {
      for (int i = 0; i <= 4; i++) {  // read the 5 Timers
        if (nr_page == i) {
          robot->timer[i].active = val[0];
          robot->timer[i].startTime.hour = val[1];
          robot->timer[i].startTime.minute = val[2];
          robot->timer[i].stopTime.hour = val[3];
          robot->timer[i].stopTime.minute = val[4];
          robot->timer[i].startDistance = val[5];
          robot->timer[i].startMowPattern = val[6];
          robot->timer[i].startNrLane = val[7];
          robot->timer[i].startRollDir = val[8];
          robot->timer[i].startLaneMaxlengh = val[9];

        }
      }

    }

    if (Setting_page == "Timer1") {
      for (int i = 0; i <= 4; i++) {  // read the 5 Timers
        if (nr_page == i) {
          robot->timer[i].startArea = val[0];
          robot->timer[i].daysOfWeek  = byte(int(val[1]));
        }
      }

    }

    //********************************************************The Bylane set Setting**********************************************
    if (Setting_page == "ByLane") {
      if (nr_page == 1) {
        robot->yawSet1 = val[0];
        robot->yawSet2 = val[1];
        robot->yawSet3 = val[2];
        robot->yawOppositeLane1RollRight = val[3];
        robot->yawOppositeLane2RollRight = val[4];
        robot->yawOppositeLane3RollRight = val[5];
      }
      if (nr_page == 2) {
        robot-> yawOppositeLane1RollLeft = val[0];
        robot-> yawOppositeLane2RollLeft = val[1];
        robot-> yawOppositeLane3RollLeft = val[2];
        robot->DistBetweenLane = val[3];
        robot->maxLenghtByLane = val[4];
      }
    }

    if (Setting_page == "Time") {
      if (nr_page == 1) {
        robot->datetime.time.hour = val[0];
        robot->datetime.time.minute = val[1];
        robot->datetime.date.dayOfWeek = val[2];
        robot->datetime.date.day = val[3];
        robot->datetime.date.month = val[4];
        robot->datetime.date.year = val[5];
        robot->setActuator(ACT_RTC, 0);
      }
    }
    if (Setting_page == "All") {
      if (nr_page == 1) {
        robot->developerActive = (bool)val[0];
        robot->motorAccel = val[1];
        robot->motorSpeedMaxRpm = val[2];
        robot->motorSpeedMaxPwm = val[3];
        robot->motorPowerMax = val[4];
        robot->motorSenseRightScale = val[5];
        robot->motorSenseLeftScale = val[6];
        robot->motorRollDegMax = val[7];
        robot->motorRollDegMin = val[8];
        robot->DistPeriOutRev = val[9];
      }
      if (nr_page == 2) {
        robot->motorPowerIgnoreTime = val[0];
        robot->motorForwTimeMax = val[1];
        robot->motorMowSpeedMaxPwm = val[2];
        robot->motorMowPowerMax = val[3];
        robot->motorMowSpeedMinPwm = val[4];
        robot->motorMowSenseScale = val[5];
        robot->motorLeftPID.Kp = val[6];
        robot->motorLeftPID.Ki = val[7];
        robot->motorLeftPID.Kd = val[8];
        robot->motorMowPID.Kp = val[9];
      }
      if (nr_page == 3) {
        robot->motorMowPID.Ki = val[0];
        robot->motorMowPID.Kd = val[1];
        robot->motorBiDirSpeedRatio1 = val[2];
        robot->motorBiDirSpeedRatio2 = val[3];
        robot->motorLeftSwapDir = val[4];
        robot->motorRightSwapDir = val[5];
        robot->bumperUse = val[6];
        robot->sonarUse = val[7];
        robot->sonarCenterUse = val[8];
        robot->sonarLeftUse = val[9];
      }
      if (nr_page == 4) {
        robot->sonarRightUse = val[0];
        robot->sonarTriggerBelow = val[1];
        robot->perimeterUse = val[2];
        robot->perimeter.timedOutIfBelowSmag = val[3];
        robot->perimeterTriggerMinSmag = val[4];
        robot->trackingErrorTimeOut = val[5];
        robot->motorTickPerSecond = val[6];
        robot->perimeterOutRevTime = val[7];
        robot->perimeterTrackRollTime = val[8];
        robot->perimeterTrackRevTime = val[9];
      }
      if (nr_page == 5) {
        robot->perimeterPID.Kp = val[0];
        robot->perimeterPID.Ki = val[1];
        robot->perimeterPID.Kd = val[2];
        robot->perimeter.signalCodeNo = val[3];
        robot->perimeter.swapCoilPolarityLeft = val[4];
        robot->perimeter.timeOutSecIfNotInside = val[5];
        robot->trakBlockInnerWheel = val[6];
        robot->lawnSensorUse = val[7];
        robot->imuUse = val[8];
        robot->stopMotorDuringCalib = val[9];
      }
      if (nr_page == 6) {
        robot->imuDirPID.Kp = val[0];
        robot->imuDirPID.Ki = val[1];
        robot->imuDirPID.Kd = val[2];
        robot->imuRollPID.Kp = val[3];
        robot->imuRollPID.Ki = val[4];
        robot->imuRollPID.Kd = val[5];
        robot->remoteUse = val[6];
        robot->batMonitor = val[7];
        robot->batGoHomeIfBelow = val[8];
        robot->batSwitchOffIfBelow = val[9];
      }
      if (nr_page == 7) {
        robot->batSwitchOffIfIdle = val[0];
        robot->batFactor = val[1];
        robot->batChgFactor = val[2];
        robot->chgSenseZero = val[3];
        robot->batSenseFactor = val[4];
        robot->batFullCurrent = val[5];
        robot->startChargingIfBelow = val[6];
        robot->stationRevDist = val[7];
        robot->stationRollAngle = val[8];
        robot->stationForwDist = val[9];
      }
      if (nr_page == 8) {
        robot->stationCheckDist = val[0];
        robot->odometryUse = val[1];
        robot->odometryTicksPerRevolution = val[2];
        robot->odometryTicksPerCm = val[3];
        robot->odometryWheelBaseCm = val[4];
        robot->autoResetActive = val[5];
        robot->CompassUse = val[6];
        robot->twoWayOdometrySensorUse = val[7];
        robot->buttonUse = val[8];
        robot->userSwitch1 = val[9];
      }
      if (nr_page == 9) {
        robot->userSwitch2 = val[0];
        robot->userSwitch3 = val[1];
        robot->timerUse = val[2];
        robot->rainUse = val[3];
        robot->gpsUse = val[4];
        robot->stuckIfGpsSpeedBelow = val[5];
        robot->gpsBaudrate = val[6];
        robot->dropUse = val[7];
        robot->statsOverride = val[8];
        robot->bluetoothUse = val[9];
      }
      if (nr_page == 10) {
        robot->esp8266Use = val[0];
        robot->esp8266ConfigString = val[1];
        robot->tiltUse = val[2];
        robot->trackingPerimeterTransitionTimeOut = val[3];
        robot->motorMowForceOff = val[4];
        robot->MaxSpeedperiPwm = val[5];
        robot->RollTimeFor45Deg = val[6];
        robot->DistPeriObstacleAvoid = val[7];
        robot->circleTimeForObstacle = val[8];
        robot->DistPeriOutRev = val[9];
      }
      if (nr_page == 11) {
        robot->motorRightOffsetFwd = val[0];
        robot->motorRightOffsetRev = val[1];
        robot->perimeterMagMaxValue = val[2];
        robot->SpeedOdoMin = val[3];
        robot->SpeedOdoMax = val[4];
        robot->yawSet1 = val[5];
        robot->yawSet2 = val[6];
        robot->yawSet3 = val[7];
        robot->yawOppositeLane1RollRight = val[8];
        robot->yawOppositeLane2RollRight = val[9];
      }
      if (nr_page == 12) {
        robot->yawOppositeLane3RollRight = val[0];
        robot->yawOppositeLane1RollLeft = val[1];
        robot->yawOppositeLane2RollLeft = val[2];
        robot->yawOppositeLane3RollLeft = val[3];
        robot->DistBetweenLane = val[4];
        robot->maxLenghtByLane = val[5];
        robot->perimeter.swapCoilPolarityRight = val[6];
        robot->perimeter.read2Coil = val[7];
        robot->maxDriftPerSecond = val[8];
        robot->delayBetweenTwoDmpAutocalib = val[9];
      }
      if (nr_page == 13) {
        robot->maxDurationDmpAutocalib = val[0];
        robot->mowPatternDurationMax = val[1];
        robot->DistPeriOutStop = val[2];
        robot->DHT22Use = val[3];
        robot->RaspberryPIUse = val[4];
        robot->sonarToFrontDist = val[5];
        robot->UseBumperDock = val[6];
        robot->dockingSpeed = val[7];
      }
    }
  }
  if (readOrWrite == 'r') {
    receivePiReqSetting (Setting_page, nr_page);
  }
}



void RpiRemote::read_pfo()
{
  //Console.print("Receive Pfod command  --> ");
  //Console.println(buf);
  float val[2] ;
  //float value1 ;
  //float value2;
  //float value3;
  int counter = 0;
  char token[4];
  Tokeniser tok(buf, ',');
  while (tok.next(token, 4))
  {
    switch (counter)
    {
      case 1: //command as char
        RpiCmd = token;
        break;
      case 2:
        val[0] = atof(token);
        break;
      case 3:
        val[1] = atof(token);
        break;
      case 4:
        val[2] = atof(token);
        break;



      case 8: //sample  : one char
        {
          if (token[0] == 'S');
        }
        break;


    }
    counter++;
  }

  robot->receivePiPfodCommand (RpiCmd, val[0], val[1], val[2]);
}
