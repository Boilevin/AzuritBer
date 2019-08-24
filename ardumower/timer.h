// timer-based mowing

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
              areaToGo=timer[i].startArea;
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
