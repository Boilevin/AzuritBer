class robot:
    def __init__(self):
        self.stateNames = ["OFF", "REMOTE", "FORW", "ROLL", "REV", "CIRC", "ERR ", "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK", "STREV",
                            "STROL", "STFOR", "MANU", "ROLW", "POUTFOR", "POUTREV", "POUTROLL", "POBSREV", "POBSROLL", "POBSFRWD", "POBSCIRC", "NEXTLANE", "POUTSTOP", "LANEROL1", "LANEROL2",
                            "ROLLTOIN", "WAITREPEAT", "FRWODO", "TESTCOMPAS", "ROLLTOTRACK",
                            "STOPTOTRACK", "AUTOCALIB", "ROLLTOFINDYAW", "TESTMOTOR", "FINDYAWSTOP", "STOPONBUMPER",
                            "STOPCALIB", "SONARTRIG", "STOPSPIRAL", "MOWSPIRAL", "ROT360", "NEXTSPIRE", "ESCAPLANE",
                            "TRACKSTOP","ROLLTOTAG", "STOPTONEWAREA", "ROLL1TONEWAREA", "DRIVE1TONEWAREA", "ROLL2TONEWAREA", "DRIVE2TONEWAREA","WAITSIG2","STOPTONEWAREA",
                            "ROLLSTOPTOTRACK","STOPTOFASTSTART","CALIBMOTORSPEED","ACCELFRWRD","ENDLANESTOP","STARTSTATION","INC_04","INC_05","INC_06"]
        self.statusNames = ["WAIT", "NORMAL_MOWING", "SPIRALE_MOWING", "BACK_TO_STATION", 
                      "TRACK_TO_START", "MANUAL", "REMOTE", "IN_ERROR", "IN_STATION",
                      "TESTING","WAITSIG2","WIRE_MOWING","INC_01","INC_02","INC_03","INC_04","INC_05","INC_06"]
        #main
        self.developerActive=0
        
        #drop
        self.dropUse=0
        
        #info
        self.statsOverride=0
        
        #drive motor
        self.motorAccel=0
        self.motorSpeedMaxRpm=0
        self.motorSpeedMaxPwm=0
        self.motorPowerMax=0
        self.motorSenseRightScale=0
        self.motorSenseLeftScale=0
        self.motorRollDegMax=0
        self.motorRollDegMin=0
        self.DistPeriOutRev=0
        self.motorPowerIgnoreTime=0
        self.motorForwTimeMax=0
        self.motorLeftPID_Kp=0
        self.motorLeftPID_Ki=0
        self.motorLeftPID_Kd=0
        self.motorBiDirSpeedRatio1=0
        self.motorBiDirSpeedRatio2=0
        self.motorLeftSwapDir=0
        self.motorRightSwapDir=0
        self.motorRightOffsetFwd=0
        self.motorRightOffsetRev=0
        self.SpeedOdoMin=0
        self.SpeedOdoMax=0
        
        #mow motor
        self.motorMowForceOff=0
        self.motorMowSpeedMaxPwm=0
        self.motorMowPowerMax=0
        self.motorMowRPMSet=0
        self.motorMowSenseScale=0
        self.motorMowPID_Kp=0
        self.motorMowPID_Ki=0
        self.motorMowPID_Kd=0
        self.mowPatternDurationMax=0
        
        #bumper
        self.bumperUse=0
        
        #sonar
        self.sonarUse=0
        self.sonarCenterUse=0
        self.sonarLeftUse=0
        self.sonarRightUse=0
        self.sonarTriggerBelow=0
        self.sonarToFrontDist=0
        
        #perimeter
        self.trackingPerimeterTransitionTimeOut=0
        self.perimeterUse=0
        self.perimeter_timedOutIfBelowSmag=0
        self.perimeterPID_Kp=0
        self.perimeterPID_Ki=0
        self.perimeterPID_Kd=0
        self.perimeter_signalCodeNo=0
        self.perimeter_swapCoilPolarityLeft=0
        self.perimeter_timeOutSecIfNotInside=0
        self.perimeter_swapCoilPolarityRight=0
        self.perimeter_read2Coil=0
        self.perimeterTriggerMinSmag=0
        self.trackingErrorTimeOut=0
        self.perimeterOutRollTimeMin=0
        self.perimeterOutRevTime=0
        self.perimeterTrackRollTime =0
        self.perimeterTrackRevTime=0
        self.trakBlockInnerWheel=0
        self.perimeterMagMaxValue=0
        self.MaxSpeedperiPwm=0
        self.RollTimeFor45Deg=0
        self.DistPeriObstacleAvoid=0
        self.circleTimeForObstacle=0
        self.DistPeriOutRev=0
        self.DistPeriOutStop=0
        
        #lawn sensor
        self.lawnSensorUse=0
        
        #imu
        self.maxDriftPerSecond=0
        self.imuUse=0
        self.stopMotorDuringCalib=0
        self.imuDirPID_Kp=0
        self.imuDirPID_Ki=0
        self.imuDirPID_Kd=0
        self.delayBetweenTwoDmpAutocalib=0
        self.maxDurationDmpAutocalib=0
        self.CompassUse=0
        
        #remote
        self.remoteUse=0
        
        #battery
        self.batMonitor=0
        self.batGoHomeIfBelow=0
        self.batSwitchOffIfBelow=0
        self.batSwitchOffIfIdle=0
        self.batFactor=0
        self.batChgFactor=0
        self.chgSenseZero=0
        self.batSenseFactor=0
        self.batFullCurrent=0
        self.startChargingIfBelow=0
        
        #odometry
        self.odometryUse=0
        self.odometryTicksPerRevolution=0
        self.odometryTicksPerCm=0
        self.odometryWheelBaseCm=0
        self.twoWayOdometrySensorUse=0
        
        #button
        self.buttonUse=0
        
        #user switch
        self.userSwitch1=0
        self.userSwitch2=0
        self.userSwitch3=0
        
        #rain
        self.rainUse=0
        
        #gps
        self.gpsUse=0
        self.stuckIfGpsSpeedBelow=0
        self.gpsSpeedIgnoreTime=0
        
       
        
        #communication
        self.bluetoothUse=0
        self.esp8266Use=0
        self.esp8266ConfigString=0
        self.RaspberryPIUse=0
        
        #tilt
        self.tiltUse=0
        
        #Bylane
        self.yawSet1=0
        self.yawSet2=0
        self.yawSet3=0
        self.yawOppositeLane1RollRight=0
        self.yawOppositeLane2RollRight=0
        self.yawOppositeLane3RollRight=0
        self.yawOppositeLane1RollLeft=0
        self.yawOppositeLane2RollLeft=0
        self.yawOppositeLane3RollLeft=0
        self.DistBetweenLane=0
        self.maxLenghtByLane=0
        
        
        
        #temperature
        self.DHT22Use=0
        
        #timer
        self.timerUse=0
        self.Timeractive=[0]*5
        self.TimerstartTime_hour=[0]*5
        self.TimerstartTime_minute=[0]*5
        self.TimerstopTime_hour=[0]*5
        self.TimerstopTime_minute=[0]*5
        self.TimerdaysOfWeek=[0]*5
        self.TimerstartDistance=[0]*5
        self.TimerstartBeacon=[0]*5
        self.TimerstartMowPattern=[0]*5
        self.TimerstartNrLane=[0]*5
        self.TimerstartRollDir=[0]*5
        self.TimerstartLaneMaxlengh=[0]*5
        self.TimerstartArea=[0]*5
        
        #station
        self.stationRevDist=0
        self.stationRollAngle=0
        self.stationForwDist=0
        self.stationCheckDist=0
        self.autoResetActive=0
        self.UseBumperDock=0
        self.dockingSpeed=0
        
        

