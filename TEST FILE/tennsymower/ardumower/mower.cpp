/*
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

*/

/* Ardumower Chassis Kit 1.0 - robot configuration (Ardumower electronics, Arduino Mega)
   http://wiki.ardumower.de/index.php?title=Ardumower_chassis

   Requires: Ardumower PCB v0.5  ( https://www.marotronics.de/Ardumower-Board-Prototyp )

*/

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// NOTE: Verify in config.h that you have enabled 'USE_MOWER' !
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
#include "mower.h"
//#include "NewPing.h"
#include <Arduino.h>
#include "drivers.h"
#include "i2c.h"
//#include "//PinMan.h"
//#include "buzzer.h"
#include "flashmem.h"



//#define USE_DEVELOPER_TEST     1      // uncomment for new perimeter signal test (developers)

Mower robot;


Mower::Mower() {
  name = "Ardumower";
  // ------- wheel motors -----------------------------
  motorAccel       = 1500;  // motor wheel acceleration - only functional when odometry is not in use (warning: do not set too low)
  //bb
  motorLeftChange = 500;
  motorRightChange = 500;
  motorOdoAccel = 1500; //Time for accel from 0 to 100% in ms
  motorSpeedMaxRpm       = 24;   // motor wheel max RPM (WARNING: do not set too high, so there's still speed control when battery is low!)
  motorSpeedMaxPwm    = 190;  // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
  motorPowerMax     = 23;    // motor wheel max power (Watt)
  motorSenseRightScale = 1.870; // normal is 1.536 motor right sense scale (mA=(ADC-zero)/scale)
  motorSenseLeftScale = 1.650; // normal is 1.536 motor left sense scale  (mA=(ADC-zero)/scale)
  motorPowerIgnoreTime = 2000; // time to ignore motor power when start to avoid read the peack on motor start (ms)
  motorZeroSettleTime   = 2000 ; // defaut 3000 how long (ms) to wait for motors to settle at zero speed
  motorRollDegMax    = 100;  // max. roll Deg
  motorRollDegMin    = 20; //min. roll Deg

  motorForwTimeMax   = 80000; // not use max. forward time (ms) / timeout
  motorBiDirSpeedRatio1 = 0.3;   // bidir mow pattern speed ratio 1
  motorBiDirSpeedRatio2 = 0.92;   // bidir mow pattern speed ratio 2
  motorLeftPID.Kp       = 1.0;    // motor wheel PID controller
  motorLeftPID.Ki       = 0.4;
  motorLeftPID.Kd       = 0.0;
  motorRightSwapDir     = 0;    // inverse right motor direction?
  motorLeftSwapDir      = 0;    // inverse left motor direction?

  motorRightOffsetFwd = 0;  //percent offset in PWM use for the 2 wheels motor have the same speed a the same PWM
  motorRightOffsetRev = 0;  //use the 1 ml ODO test to find good value the 2 wheels need to stop at the same time
  motorTickPerSecond = 200; // use to compute the maxodostate duration and computed on the calibration motor

  UseAccelLeft = 1;
  UseBrakeLeft = 1;
  UseAccelRight = 1;
  UseBrakeRight = 1;
  AngleRotate = 100;
  SpeedOdoMin = 50;
  SpeedOdoMax = 140;
  odoLeftRightCorrection     = true;       // left-right correction for straight lines used in manual mode
  autoAdjustSlopeSpeed = true;  //adjust the speed on slope to have same speed on uphill and downhill

  
  // ------ mower motor -------------------------------
  motorMowAccel       = 1000;  // motor mower acceleration (warning: do not set too low) 2000 seems to fit best considerating start time and power consumption
  motorMowSpeedMaxPwm   = 200;    // motor mower max PWM
  motorMowSpeedMinPwm = 100;   // motor mower minimum PWM (only for cutter modulation)
  motorMowPowerMax = 18.0;     // motor mower max power (Watt)

  motorMowSenseScale = 1.536; // motor mower sense scale (mA=(ADC-zero)/scale)
  motorMowPID.Kp = 0.005;    // motor mower RPM PID controller
  motorMowPID.Ki = 0.01;
  motorMowPID.Kd = 0.01;
  //  ------ bumper -----------------------------------
  bumperUse         = 0;      // has bumpers?
  //  ------ drop -----------------------------------
  dropUse          = 0;     // has drops?                                                                                              Dropsensor - Absturzsensor vorhanden ?
  dropcontact      = 0;     //contact 0-openers 1-closers                                                                              Dropsensor - Kontakt 0-Ã–ffner - 1-SchlieÃŸer betÃ¤tigt gegen GND
  // ------ rain ------------------------------------
  rainUse          = 0;      // use rain sensor?

  // ------ DHT22Use ------------------------------------
  DHT22Use          = 0;      // use DHT22 sensor?
  maxTemperature    = 55;     // max temp before switch off
  //bber35
  // ------ RFID ------------------------------------
  rfidUse          = 0;      // use rfid
  newtagRotAngle1 = -90;
  newtagRotAngle2 = 0;
  newtagDistance1 = 10;
  newtagDistance2 = 0;

  // ------ sonar ------------------------------------
  sonarUse                   = 0;          // use ultra sonic sensor? (WARNING: robot will slow down, if enabled but not connected!)
  sonarLeftUse               = 1;
  sonarRightUse              = 1;
  sonarCenterUse             = 0;
  sonarTriggerBelow          = 87;       // ultrasonic sensor trigger distance in cm (0=off)
  sonarToFrontDist           = 30;        // ultrasonic sensor distance to front mower in cm
  sonarLikeBumper            = false;      //ultrasonic reduce speed vs bumper like



  // ------ perimeter ---------------------------------
  perimeterUse       = 0;      // use perimeter?
  perimeterTriggerMinSmag = 200;      // perimeter minimum smag to use on big area
  //perimeterOutRollTimeMax  = 2000;   // free
  //perimeterOutRollTimeMin = 750;    // free
  perimeterOutRevTime   = 2200;   // free
  perimeterTrackRollTime = 1500; //roll time during perimeter tracking
  perimeterTrackRevTime = 2200;  // reverse time during perimeter tracking
  DistPeriOutRev = 40; // reverse distance when reach the perimeter in cm
  DistPeriObstacleRev = 30; // reverse distance when hit obstacle while tracking in cm
  DistPeriOutForw = 60; // distance to accell
  DistPeriOutStop = 15; //slowing distance after crossover the wire
  DistPeriObstacleForw = 25; //distance while arc circle in peri obstacle avoid
  perimeterPID.Kp    = 16.5;  // perimeter PID controller
  perimeterPID.Ki    = 8;
  perimeterPID.Kd    = 0;
  trackingPerimeterTransitionTimeOut = 1500;
  trackingErrorTimeOut = 10000;
  trakBlockInnerWheel = true;
  //bb
  MaxSpeedperiPwm = 180; // speed max in PWM while perimeter tracking
  ActualSpeedPeriPWM = MaxSpeedperiPwm; //speed in PWM while perimeter tracking
  //timeToResetSpeedPeri = 0; // if millis() > at this var the speed is set to max value
  RollTimeFor45Deg = 1000; //time while roll in peri obstacle avoid if no Odometry
  circleTimeForObstacle = 4000; //time while arc circle in peri obstacle avoid if no Odometry
  DistPeriObstacleAvoid = 100; //distance while arc circle in peri obstacle avoid
  perimeterMagMaxValue = 2000; // Maximum value return when near the perimeter wire (use for tracking and slowing when near wire
  perimeter.read2Coil = false;
  areaToGo = 1;//initialise the areatogo to the station area

  // ------ lawn sensor --------------------------------
  lawnSensorUse     = 0;       // use capacitive Sensor
  // ------  IMU (compass/accel/gyro) ----------------------
  imuUse            = 0;       // use IMU?
  CompassUse = 0;       // activate compass?
  stopMotorDuringCalib     = 0;       // correct direction by compass?
  imuDirPID.Kp      = 4.4;     // direction PID controller
  imuDirPID.Ki      = 3.3;
  imuDirPID.Kd      = 0.0;
  imuRollPID.Kp     = 0.8;   // roll PID controller
  imuRollPID.Ki     = 21;
  imuRollPID.Kd     = 0;
  //bb
  yawSet1 = 45;
  yawOppositeLane1RollRight = -125;
  yawOppositeLane1RollLeft = -135;
  yawSet2 = 90;
  yawOppositeLane2RollRight = -92;
  yawOppositeLane2RollLeft = -88;
  yawSet3 = 135;
  yawOppositeLane3RollRight = -47;
  yawOppositeLane3RollLeft = -42;
  laneUseNr = 2;
  maxDriftPerSecond = 0.05; //limit the stop time if small drift
  maxDurationDmpAutocalib = 60; //in sec
  delayBetweenTwoDmpAutocalib = 360; //in sec
  yawCiblePos = 90;
  yawCibleNeg = -90;
  DistBetweenLane = 38;
  maxLenghtByLane = 9;  // distance to run in bylane before simulate a wire detection
  justChangeLaneDir = true;
  mowPatternCurr = MOW_LANES;
  compassRollSpeedCoeff = 40; //speed used when the mower search the compass yaw it's percent of motorSpeedMaxRpm ,Avoid to roll to fast for a correct detection


  // ------ model R/C ------------------------------------
  remoteUse         = 0;       // use model remote control (R/C)?
  // ------ battery -------------------------------------
  batMonitor = false;              // monitor battery and charge voltage?
  batGoHomeIfBelow = 24.3;     // drive home voltage (Volt)
  batSwitchOffIfBelow = 23;  // switch off battery if below voltage (Volt)
  batSwitchOffIfIdle = 300;      // switch off battery if idle (minutes)
  batFactor       = 10.88;     //depend of the resistor divisor on board R12 and R13
  batChgFactor    = 10.89;     //depend of the resistor divisor on board R9 and R10
  batFull          = 29.4;     // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
  batChargingCurrentMax = 2; // maximum current your charger can devliver
  batFullCurrent  = 0.1;      // current flowing when battery is fully charged
  startChargingIfBelow = 25.0; // start charging if battery Voltage is below
  chargingTimeout = 25200000; // safety timer for charging (ms)  7 hrs
  chgSenseZero    = 511;        // charge current sense zero point
  batSenseFactor  = 1.11;         // charge current conversion factor   - Empfindlichkeit nimmt mit ca. 39/V Vcc ab
  chgSense        = 185.0;      // mV/A empfindlichkeit des Ladestromsensors in mV/A (FÃ¼r ACS712 5A = 185)
  chgChange       = 0;          // Messwertumkehr von - nach +         1 oder 0
  chgNull         = 2;          // Nullduchgang abziehen (1 oder 2)
  // ------  charging station ---------------------------
  stationRevDist     = 50;    // charge station reverse 50 cm
  stationRollAngle    = 45;    // charge station roll after reverse
  stationForwDist    = 30;    // charge station accel distance cm
  stationCheckDist   = 2;    // charge station  check distance to be sure voltage is OK cm
  UseBumperDock = true; //bumper is pressed when docking or not
  dockingSpeed   =  60;   //speed docking is (percent of maxspeed)
  autoResetActive  = 0;       // after charging reboot or not


  // ------ odometry ------------------------------------
  odometryUse       = 1;       // use odometry?
  odometryTicksPerRevolution = 1010;   // encoder ticks per one full resolution
  odometryTicksPerCm = 12.9;  // encoder ticks per cm
  odometryWheelBaseCm = 43;    // wheel-to-wheel distance (cm)



  // ----- GPS -------------------------------------------
  gpsUse            = 0;       // use GPS?
  stuckIfGpsSpeedBelow = 0.2; // if Gps speed is below given value the mower is stuck
  gpsBaudrate = 9600; // Gps baud rate setting (use 9600 for m6n and 38400 for m8n)

  // ----- other -----------------------------------------
  buttonUse         = 1;       // has digital ON/OFF button?
  RaspberryPIUse = false; // a raspberryPi is connected to USBNative port
  mowPatternDurationMax = 120; //in minutes

  // ----- user-defined switch ---------------------------
  userSwitch1       = 0;       // user-defined switch 1 (default value)
  userSwitch2       = 0;       // user-defined switch 2 (default value)
  userSwitch3       = 0;       // user-defined switch 3 (default value)
  // ----- timer -----------------------------------------
  timerUse          = 0;       // use RTC and timer?
  // ----- bluetooth -------------------------------------
  bluetoothUse      = 1;      // use Bluetooth module? It's Impossible to use Bluetooth and esp8266 at same time
  // ----- esp8266 ---------------------------------------
  esp8266Use        = 0;       // use ESP8266 Wifi module?
  esp8266ConfigString = "1234567321"; // always use 10 char to avoid eeprom corruption

  // ------ mower stats-------------------------------------------
  statsOverride = false; // if set to true mower stats are overwritten - be careful
  statsMowTimeMinutesTotal = 300;
  statsBatteryChargingCounterTotal = 10;  //11
  statsBatteryChargingCapacityTotal = 10000;  //30000
  // -----------configuration end-------------------------------------
}

/*
// remote control (RC) ppm signal change interrupt
ISR(PCINT0_vect) {
  unsigned long timeMicros = micros();
  boolean remoteSpeedState = digitalRead(pinRemoteSpeed);
  boolean remoteSteerState = digitalRead(pinRemoteSteer);
  boolean remoteMowState = digitalRead(pinRemoteMow);
  boolean remoteSwitchState = digitalRead(pinRemoteSwitch);
  robot.setRemotePPMState(timeMicros, remoteSpeedState, remoteSteerState, remoteMowState, remoteSwitchState);
}


volatile long oldOdoPins_A = 0;
volatile long oldOdoPins_B = 0;
ISR(PCINT2_vect)
{
  const long actPins_A = REG_PIOA_PDSR;             // read PIO A
  const long actPins_B = REG_PIOB_PDSR;                               // read PIO B
  const long setPins_A = (oldOdoPins_A ^ actPins_A); //bitwise xor return 1 if operator are different else return 0  1100^0101 egal 1001
  const long setPins_B = (oldOdoPins_B ^ actPins_B);
  if (setPins_A & 0b00000000000000000000000000000010) {   // pin left has changed
    if (robot.motorRightPWMCurr >= 0) robot.odometryRight++; else robot.odometryRight--;
    oldOdoPins_A = actPins_A;
  }
  if (setPins_B & 0b00000000000000001000000000000000)           // pin right has changed
  {
    if (robot.motorLeftPWMCurr >= 0) robot.odometryLeft++; else robot.odometryLeft--;
    oldOdoPins_B = actPins_B;
  }
}
*/
/*
NewPing NewSonarLeft(pinSonarLeftTrigger, pinSonarLeftEcho, 110);
NewPing NewSonarRight(pinSonarRightTrigger, pinSonarRightEcho, 110);
NewPing NewSonarCenter(pinSonarCenterTrigger, pinSonarCenterEcho, 110);

*/


void Mower::setup() {


  //PinMan.begin();
  // keep battery switched ON (keep this at system start!)
  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);

 // Buzzer.begin();
  Console.begin(CONSOLE_BAUDRATE);
  I2Creset();
  Wire.begin();
 // Wire1.begin();
 // Wire1.setClock(400000);
  // Flash.test();

  /* while (!checkAT24C32()){
     Console.println("PCB not powered ON or RTC module missing");
     delay(1000);
    }
  */
  Console.println("SETUP");


  // LED, buzzer, battery
  pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, 0);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeRelay, OUTPUT);
  setActuator(ACT_CHGRELAY, 0);

  // left wheel motor
  pinMode(pinMotorEnable, OUTPUT);
  digitalWrite(pinMotorEnable, HIGH);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);
  pinMode(pinMotorLeftSense, INPUT);
  pinMode(pinMotorLeftFault, INPUT);

  // right wheel motor
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);
  pinMode(pinMotorRightSense, INPUT);
  pinMode(pinMotorRightFault, INPUT);

  // mower motor
  pinMode(pinMotorMowDir, OUTPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  pinMode(pinMotorMowSense, INPUT);
  pinMode(pinMotorMowRpm, INPUT);
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, HIGH);
  pinMode(pinMotorMowFault, INPUT);

  // lawn sensor
  pinMode(pinLawnBackRecv, INPUT);
  pinMode(pinLawnBackSend, OUTPUT);
  pinMode(pinLawnFrontRecv, INPUT);
  pinMode(pinLawnFrontSend, OUTPUT);

  // perimeter
  pinMode(pinPerimeterRight, INPUT);
  pinMode(pinPerimeterLeft, INPUT);
  pinMode(pinPerimeterCenter, INPUT);


  // button
  //pinMode(pinButton, INPUT);
  pinMode(pinButton, INPUT_PULLUP);

  // bumpers
  //pinMode(pinBumperLeft, INPUT);
  pinMode(pinBumperLeft, INPUT_PULLUP); //it's contact
  //pinMode(pinBumperRight, INPUT);
  pinMode(pinBumperRight, INPUT_PULLUP);

  // drops
  // pinMode(pinDropLeft, INPUT);                                                                                                         // Dropsensor - Absturzsensor - Deklariert als Eingang
  pinMode(pinDropLeft, INPUT_PULLUP);                                                                                                  // Dropsensor - Absturzsensor - Intern Pullab Widerstand aktiviert (AuslÃ¶sung erfolgt gegen GND)
  // pinMode(pinDropRight, INPUT);                                                                                                        // Dropsensor - Absturzsensor - Deklariert als Eingang
  pinMode(pinDropRight, INPUT_PULLUP);                                                                                                 // Dropsensor - Absturzsensor - Intern Pullab Widerstand aktiviert (AuslÃ¶sung erfolgt gegen GND)

  // rain
  pinMode(pinRain, INPUT);

  // R/C
  pinMode(pinRemoteMow, INPUT);
  pinMode(pinRemoteSteer, INPUT);
  pinMode(pinRemoteSpeed, INPUT);
  pinMode(pinRemoteSwitch, INPUT);

  // odometry
  //not sure the pullupis necessary with PCB1.3
  pinMode(pinOdometryLeft, INPUT_PULLUP);
  pinMode(pinOdometryLeft2, INPUT_PULLUP);
  pinMode(pinOdometryRight, INPUT_PULLUP);
  pinMode(pinOdometryRight2, INPUT_PULLUP);

  // user switches
  pinMode(pinUserSwitch1, OUTPUT);
  pinMode(pinUserSwitch2, OUTPUT);
  pinMode(pinUserSwitch3, OUTPUT);

  // other
  pinMode(pinVoltageMeasurement, INPUT);

  // PWM frequency setup
  // For obstacle detection, motor torque should be detectable - torque can be computed by motor current.
  // To get consistent current values, PWM frequency should be 3.9 Khz
  // http://wiki.ardumower.de/index.php?title=Motor_driver
  // http://sobisource.com/arduino-mega-pwm-pin-and-frequency-timer-control/
  // http://www.atmel.com/images/doc2549.pdf

  //PinMan.analogWrite(pinMotorMowPWM, 0); // sets PWMEnabled=true in Arduino library
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(3900 * PWM_MAX_DUTY_CYCLE, 0, VARIANT_MCK);   // 3.9 Khz

  ADCMan.setupChannel(pinChargeCurrent, 1, false);
  ADCMan.setupChannel(pinMotorMowSense, 1, false);
  ADCMan.setupChannel(pinMotorLeftSense, 1, false);
  ADCMan.setupChannel(pinMotorRightSense, 1, false);


  ADCMan.setupChannel(pinBatteryVoltage, 1, false);
  ADCMan.setupChannel(pinChargeVoltage, 1, false);
  ADCMan.setupChannel(pinVoltageMeasurement, 1, false);
  // perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);

  //imu.init();

  //gps.init();
  // Console.print(millis());
  // Console.println(" --> ******************************************* Start of Robot Setup from mower.cpp *********************************");
  Robot::setup();
  // Console.print(millis());
  // Console.println(" --> ******************************************* Back to mower.cpp *********************************");

  if (esp8266Use) {
    Console.println(F("ESP8266 in used : Use Arduremote over WIFI"));
    ESP8266port.begin(ESP8266_BAUDRATE);
    ESP8266port.println(esp8266ConfigString);
    ESP8266port.flush();
    ESP8266port.end();
    rc.initSerial(&ESP8266port, ESP8266_BAUDRATE);
  } else if (bluetoothUse) {
    Console.println(F("BT in used : Use Arduremote over Bluetooth"));
    rc.initSerial(&Bluetooth, BLUETOOTH_BAUDRATE);
  }

  // enable interrupts
  //-----------------------------------------------------------------------------------------------------------------UweZ geÃ¤ndert Anfang---------------------------------
  // Due interrupts
  attachInterrupt(pinOdometryLeft, PCINT2_vect, CHANGE);
  attachInterrupt(pinOdometryLeft2, PCINT2_vect, CHANGE);
  attachInterrupt(pinOdometryRight, PCINT2_vect, CHANGE);
  attachInterrupt(pinOdometryRight2, PCINT2_vect, CHANGE);

  attachInterrupt(pinRemoteSpeed, PCINT0_vect, CHANGE);
  attachInterrupt(pinRemoteSteer, PCINT0_vect, CHANGE);
  attachInterrupt(pinRemoteMow, PCINT0_vect, CHANGE);
  attachInterrupt(pinRemoteSwitch, PCINT0_vect, CHANGE);

  //attachInterrupt(pinMotorMowRpm, rpm_interrupt, CHANGE);
  attachInterrupt(pinMotorMowRpm, PCINT2_vect, CHANGE);
  // Console.print(millis());
  //Console.println(" --> ******************************************* End of attach interrupt *********************************");


}

void checkMotorFault() {
  //bb to test without motor board uncomment return
  //return;
  if ((robot.stateCurr == STATE_OFF) || (robot.stateCurr == STATE_ERROR)  ) return;  //do not generate error if the state if OFF to avoid Buzzer when PI power the DUE via the USB native port
  if (digitalRead(pinMotorLeftFault) == LOW) {
    robot.addErrorCounter(ERR_MOTOR_LEFT);
    Console.println(F("Error: motor left fault"));
    robot.setNextState(STATE_ERROR, 0);
    return;

  }
  if  (digitalRead(pinMotorRightFault) == LOW) {
    robot.addErrorCounter(ERR_MOTOR_RIGHT);
    Console.println(F("Error: motor right fault"));
    robot.setNextState(STATE_ERROR, 0);
    return;

  }
  if (digitalRead(pinMotorMowFault) == LOW) {
    robot.addErrorCounter(ERR_MOTOR_MOW);
    Console.println(F("Error: motor mow fault"));
    robot.setNextState(STATE_ERROR, 0);
    return;

  }
}


int Mower::readSensor(char type) {

  // the azurit readsensor send an integer to robot.cpp so can't use getVoltage from adcman as it's float
  switch (type) {

    // motors------------------------------------------------------------------------------------------------
    case SEN_MOTOR_MOW: return ADCMan.getValue(pinMotorMowSense); break;
    case SEN_MOTOR_RIGHT: checkMotorFault(); return ADCMan.getValue(pinMotorRightSense); break;
    case SEN_MOTOR_LEFT: checkMotorFault(); return ADCMan.getValue(pinMotorLeftSense); break;



    //case SEN_MOTOR_MOW_RPM: break; // not used - rpm is upated via interrupt

    // perimeter----------------------------------------------------------------------------------------------
    case SEN_PERIM_LEFT: return perimeter.getMagnitude(0); break;
    case SEN_PERIM_RIGHT: return perimeter.getMagnitude(1); break;

    // battery------------------------------------------------------------------------------------------------
    case SEN_BAT_VOLTAGE: return ADCMan.getValue(pinBatteryVoltage) ; break;
    case SEN_CHG_VOLTAGE: return ADCMan.getValue(pinChargeVoltage)  ; break;
    case SEN_CHG_CURRENT: return ADCMan.getValue(pinChargeCurrent) ;  break;

    // buttons------------------------------------------------------------------------------------------------
    case SEN_BUTTON: return (digitalRead(pinButton)); break;

    //bumper----------------------------------------------------------------------------------------------------
    case SEN_BUMPER_RIGHT: return (digitalRead(pinBumperRight)); break;
    case SEN_BUMPER_LEFT: return (digitalRead(pinBumperLeft)); break;

    //drop----------------------------------------------------------------------------------------------------
    case SEN_DROP_RIGHT: return (digitalRead(pinDropRight)); break;                                                                                     // Dropsensor - Absturzsensor
    case SEN_DROP_LEFT: return (digitalRead(pinDropLeft)); break;                                                                                       // Dropsensor - Absturzsensor

    // sonar---------------------------------------------------------------------------------------------------

    case SEN_SONAR_CENTER: return (NewSonarCenter.ping_cm()); break;
    case SEN_SONAR_LEFT: return (NewSonarLeft.ping_cm()); break;

    case SEN_SONAR_RIGHT: return (NewSonarRight.ping_cm()); break;


    // case SEN_LAWN_FRONT: return(measureLawnCapacity(pinLawnFrontSend, pinLawnFrontRecv)); break;
    //case SEN_LAWN_BACK: return(measureLawnCapacity(pinLawnBackSend, pinLawnBackRecv)); break;


    // rtc--------------------------------------------------------------------------------------------------------
    case SEN_RTC:
      if (!readDS1307(datetime)) {
        Console.println("RTC data error!");
        addErrorCounter(ERR_RTC_DATA);
        setNextState(STATE_ERROR, 0);
      }
      break;


    // rain--------------------------------------------------------------------------------------------------------
    case SEN_RAIN: if (digitalRead(pinRain) == LOW) return 1; break;

  }
  return 0;
}

void Mower::setActuator(char type, int value) {


  switch (type) {


    case ACT_MOTOR_MOW: setMC33926(pinMotorMowDir, pinMotorMowPWM, value); break;// Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926
    //bb
    case ACT_MOTOR_LEFT: setMC33926(pinMotorLeftDir, pinMotorLeftPWM, value); break;//   Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926

    case ACT_MOTOR_RIGHT:
      if (value >= 0) setMC33926(pinMotorRightDir, pinMotorRightPWM, value * (1 + (double)motorRightOffsetFwd / 100));
      else setMC33926(pinMotorRightDir, pinMotorRightPWM, value * (1 - (double)motorRightOffsetRev / 100));
      break; //  Motortreiber einstellung - bei Bedarf Ã¤ndern z.B setL298N auf setMC33926

    case ACT_BUZZER: if (value == 0) Buzzer.noTone(); else Buzzer.tone(value); break;
    case ACT_LED: digitalWrite(pinLED, value); break;
    case ACT_USER_SW1: digitalWrite(pinUserSwitch1, value); break;
    case ACT_USER_SW2: digitalWrite(pinUserSwitch2, value); break;
    case ACT_USER_SW3: digitalWrite(pinUserSwitch3, value); break;
    case ACT_RTC:
      if (!setDS1307(datetime)) {
        Console.println("RTC comm error!");
        addErrorCounter(ERR_RTC_COMM);
        setNextState(STATE_ERROR, 0);
      }
      break;
    case ACT_CHGRELAY: digitalWrite(pinChargeRelay, value); break;
    //case ACT_CHGRELAY: digitalWrite(pinChargeRelay, !value); break;
    case ACT_BATTERY_SW: digitalWrite(pinBatterySwitch, value); break;
  }
}

void Mower::configureBluetooth(boolean quick) {
  BluetoothConfig bt;
  bt.setParams(name, BLUETOOTH_PIN, BLUETOOTH_BAUDRATE, quick);
}
