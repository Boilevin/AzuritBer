#!/usr/bin/env python3

import sys
import serial
import pynmea2
import time
#import numpy as np
import subprocess
import pickle

import os
from tkinter.ttk import Notebook
from tkinter import messagebox
from tkinter import filedialog
import tkinter as tk
import math
"""      bb file     """
from robot import *

sys.path.insert(0, "/home/pi/Documents/PiArdumower") #add to avoid KST plot error on path

def ButtonFlashDue_click():
    messagebox.showinfo('Info',"Actual program use the USB Serial so it need to be closed. it will restart at the end of the Flashing")
    ser.close()
    time.sleep(3)
    subprocess.call('/home/pi/Documents/PiArdumower/DueFlash.py')
    time.sleep(3)
    ser.open()

class streamVideo_class(object):
    """class use to start and stop the video stream"""
    def __init__(self):
        self.streamVideo = None

    def start(self,resolution):
        self.stop()
        print(resolution)
        if resolution==0:
            self.streamVideo=subprocess.Popen(["/home/pi/Documents/PiArdumower/streamVideo320.py","shell=True","stdout=subprocess.PIPE"])
        if resolution==1:
            self.streamVideo=subprocess.Popen(["/home/pi/Documents/PiArdumower/streamVideo640.py","shell=True","stdout=subprocess.PIPE"])
        
    def stop(self):
        if self.streamVideo:
            self.streamVideo.kill()
            self.streamVideo.wait()
            self.streamVideo = None
            
myStreamVideo=streamVideo_class()

    
class PlotterKst_class(object):
    """class use to start and stop the kst plotting prog"""
    def __init__(self):
        self.PlotterKst = None

    def start(self,fileNameKst):
        self.stop()
        self.PlotterKst=subprocess.Popen(["kst2",fileNameKst,"shell=True","stdout=subprocess.PIPE"])
        
    def stop(self):
        if self.PlotterKst:
            self.PlotterKst.kill()
            self.PlotterKst.wait()
            self.PlotterKst = None

direction_list=['LEFT','RIGHT']
days_list=['SUNDAY','MONDAY','TUESDAY','WEDNESDAY','THURSDAY','FRIDAY','SATURDAY']

motPlotterKst = PlotterKst_class()
mowPlotterKst = PlotterKst_class()
periPlotterKst = PlotterKst_class()
batPlotterKst = PlotterKst_class()

firstplotMotx=0
firstplotMowx=0
firstplotBatx=0
firstplotPerx=0

actualRep=os.getcwd()
dateNow=time.strftime('%d/%m/%y %H:%M:%S',time.localtime())

fen1 =tk.Tk()
"""Variable for check button """
MotVar1 = tk.IntVar()
MotVar2 = tk.IntVar()
PeriVar1 = tk.IntVar()
PeriVar2 = tk.IntVar()
PeriVar3 = tk.IntVar()
PeriVar4 = tk.IntVar()
ImuVar1=tk.IntVar()
SonVar1=tk.IntVar()
SonVar2=tk.IntVar()
SonVar3=tk.IntVar()
BatVar1=tk.IntVar()
MowVar1=tk.IntVar()
PlotVar1=tk.IntVar()
CamVar1=tk.IntVar()

tk_date_hour=tk.IntVar()
tk_date_minute=tk.IntVar()
tk_date_dayOfWeek=tk.IntVar()
tk_date_day=tk.IntVar()
tk_date_month=tk.IntVar()
tk_date_year=tk.IntVar()
tk_mowingPattern=tk.IntVar()

"""variable use into Auto Menu"""
tk_batVoltage=tk.DoubleVar()
tk_ImuYaw=tk.DoubleVar()
tk_ImuPitch=tk.DoubleVar()
tk_ImuRoll=tk.DoubleVar()
tk_Dht22Temp=tk.DoubleVar()
tk_Dht22Humid=tk.DoubleVar()

"""variable use into refreh plot"""
tk_millis=tk.IntVar()
tk_motorLeftSenseCurrent=tk.DoubleVar()
tk_motorRightSenseCurrent=tk.DoubleVar()
tk_motorLeftPWMCurr=tk.IntVar()
tk_motorRightPWMCurr=tk.IntVar()
tk_motorMowSense=tk.DoubleVar()
tk_motorMowPWMCurr=tk.IntVar()
#tk_batVoltage=tk.IntVar()
tk_chgVoltage=tk.DoubleVar()
tk_chgSense=tk.DoubleVar()
tk_perimeterMag=tk.IntVar()
tk_perimeterMagRight=tk.IntVar()

MainperimeterUse= tk.IntVar()
MainimuUse= tk.IntVar()
MaingpsUse= tk.IntVar()
MainbluetoothUse= tk.IntVar()
MainbumperUse= tk.IntVar()
MainsonarUse= tk.IntVar()
MainDHT22Use= tk.IntVar()
MainlawnSensorUse= tk.IntVar()
MaintimerUse= tk.IntVar()

MainrainUse= tk.IntVar()
MaindropUse= tk.IntVar()
Mainesp8266Use= tk.IntVar()
MaintiltUse= tk.IntVar()

tk_rollDir=tk.StringVar()
tk_laneInUse= tk.IntVar()
tk_YawActual=tk.DoubleVar()
tk_YawCible=tk.DoubleVar()



fen1.title('ARDUMOWER')
fen1.geometry("800x480")

class  datetime:
    def __init__(self):       
        datetime.hour = 12;
        datetime.minute = 0;
        datetime.dayOfWeek = 0;
        datetime.day = 1;
        datetime.month = 1;
        datetime.year = 2017;

class mower:
    #char* mowPatternNames[] = {"RAND", "LANE",  "WIRE"};
    def __init__(self):
        mower.millis=0
        mower.state="OFF"
        mower.odox=0
        mower.odoy=0
        mower.prevYaw=0
        mower.batVoltage=0
        mower.yaw=0
        mower.pitch=0
        mower.roll=0
        mower.version='Unknow'
        mower.statsOverride=0
        mower.statsMowTimeMinutesTrip=0
        mower.statsMowTimeHoursTotal=0
        mower.statsBatteryChargingCounterTotal=0
        mower.statsBatteryChargingCapacityTrip=0
        mower.statsBatteryChargingCapacityTotal=0
        mower.statsBatteryChargingCapacityAverage=0
        mower.motorLeftSenseCurrent=0
        mower.motorRightSenseCurrent=0
        mower.motorLeftPWMCurr=0
        mower.motorRightPWMCurr=0
        mower.motorMowSense=0
        mower.motorMowPWMCurr=0
        mower.mowPatternCurr=0
        mower.Dht22Temp=0
        mower.Dht22Humid=0
        mower.rollDir=0
        mower.laneInUse= 0
        mower.YawActual=0
        mower.YawCible=0

        
    
mymower=mower()
myRobot=robot()
myDate=datetime()


def checkSerial():  #the main loop is that
    #KeyboardFocusManager.getCurrentKeyboardFocusManager().getActiveWindow()
    
    response1=ser.readline()
    
    if str(response1)!="b''":  
        
        response1=str(response1,'utf8')
        if response1[:1] != '$' : #it is console message because the first digit is not $
            txtConsoleRecu.insert('1.0', response1)
            #print(response1)
        else :  # here a nmea message 
            message = pynmea2.parse(response1)
            txtRecu.insert('1.0', str(message)+ '\n')


            if message.sentence_type =='CMD': #receive a command from the DUE (need to do something
                if message.actuatorname == 'PowerOffPi':
                    ConsolePage.tkraise()
                    text1.config(text="Start to save all Data")
                    txtConsoleRecu.insert('1.0', 'Start to save all Data')
                    ButtonSaveReceived_click()  #save the console txt
                    
                    text1.config(text="Start to shutdown")
                    txtConsoleRecu.insert('1.0', 'PI start Shutdown')
                    time.sleep(1)
                    print("Start subprocess")
                    subprocess.Popen('/home/pi/Documents/PiArdumower/PowerOffNow.py')
                    fen1.destroy()
                    time.sleep(1)
                    print("Fen1 is destroy")
                    sys.exit("PowerOFF ordered by Arduino Undervoltage")
                    
                    
                   
        

            if message.sentence_type =='BYL': #to refresh the ByLane setting page 
                mymower.millis=message.millis
                mymower.rollDir=message.rollDir
                mymower.laneInUse=message.laneInUse
                mymower.YawActual=message.YawActual
                mymower.YawCible=message.YawCible
               
                tk_millis.set(mymower.millis)
                tk_rollDir.set(direction_list[int(mymower.rollDir)])
                tk_laneInUse.set(mymower.laneInUse)
                tk_YawActual.set(format(float(mymower.YawActual)*180/math.pi,'.1f'))
                tk_YawCible.set(format(float(mymower.YawCible),'.1f'))

            
            if message.sentence_type =='INF': #to refresh the info page
                mymower.millis=message.millis
                mymower.developerActive=message.developerActive
                mymower.version=message.version
                Infoline1.set("Firmware Version : " + mymower.version)
                mymower.statsOverride=message.statsOverride
                Infoline2.set("Developer Active : " + mymower.developerActive +" / statsOverride : " + str(mymower.statsOverride))
                mymower.statsMowTimeMinutesTrip=message.statsMowTimeMinutesTrip
                mymower.statsMowTimeHoursTotal=message.statsMowTimeHoursTotal
                Infoline3.set("Mowing Duration Last Trip : " + str(mymower.statsMowTimeMinutesTrip) + " Minutes")
                Infoline4.set ("Mowing Duration Total : " + str(mymower.statsMowTimeHoursTotal) + " Hours")
             
                mymower.statsBatteryChargingCounterTotal=message.statsBatteryChargingCounterTotal
                mymower.statsBatteryChargingCapacityTrip=message.statsBatteryChargingCapacityTrip
                mymower.statsBatteryChargingCapacityTotal=message.statsBatteryChargingCapacityTotal
                mymower.statsBatteryChargingCapacityAverage=message.statsBatteryChargingCapacityAverage



            if message.sentence_type =='MOT': #to refresh the plot page of motor wheel
                global firstplotMotx
                
                mymower.millis=message.millis
                mymower.motorLeftSenseCurrent=message.motorLeftSenseCurrent
                mymower.motorRightSenseCurrent=message.motorRightSenseCurrent
                mymower.motorLeftPWMCurr=message.motorLeftPWMCurr
                mymower.motorRightPWMCurr=message.motorRightPWMCurr
                mymower.batVoltage=message.batVoltage
                if firstplotMotx==0:
                    firstplotMotx=int(mymower.millis)
                tk_millis.set(mymower.millis)
                tk_motorLeftSenseCurrent.set(mymower.motorLeftSenseCurrent)
                tk_motorRightSenseCurrent.set(mymower.motorRightSenseCurrent)
                tk_motorLeftPWMCurr.set(mymower.motorLeftPWMCurr)
                tk_motorRightPWMCurr.set(mymower.motorRightPWMCurr)

                f=open("/home/pi/Documents/PiArdumower/log/PlotMot.txt",'a+')
                f.write("{};{};{};{};{}\n".format((int(mymower.millis)-firstplotMotx)/1000,float(mymower.motorLeftSenseCurrent) , float(mymower.motorRightSenseCurrent),float(mymower.motorLeftPWMCurr) , float(mymower.motorRightPWMCurr)))
                f.close()
                
            if message.sentence_type =='MOW': #to refresh the plot page of motor mow
                global firstplotMowx
                
                mymower.millis=message.millis
                mymower.motorMowSense=message.motorMowSense
                mymower.motorMowPWMCurr=message.motorMowPWMCurr
                mymower.batVoltage=message.batVoltage
                
                if firstplotMowx==0:
                    firstplotMowx=int(mymower.millis)
                tk_millis.set(mymower.millis)
                tk_motorMowSense.set(mymower.motorMowSense)
                tk_motorMowPWMCurr.set(mymower.motorMowPWMCurr)
                tk_batVoltage.set(mymower.batVoltage)

                f=open("/home/pi/Documents/PiArdumower/log/PlotMow.txt",'a+')
                f.write("{};{};{}\n".format((int(mymower.millis)-firstplotMowx)/1000,float(mymower.motorMowSense) , float(mymower.motorMowPWMCurr)))
                f.close()
               
                
            if message.sentence_type =='BAT': #to refresh the plot page of battery
                global firstplotBatx
                
                mymower.millis=message.millis
                mymower.chgVoltage=message.chgVoltage
                mymower.chgSense=message.chgSense
                mymower.batVoltage=message.batVoltage
                
                if firstplotBatx==0:
                    firstplotBatx=int(mymower.millis)
                tk_millis.set(mymower.millis)
                tk_chgVoltage.set(mymower.chgVoltage)
                tk_chgSense.set(mymower.chgSense)
                tk_batVoltage.set(mymower.batVoltage)
                
                f=open("/home/pi/Documents/PiArdumower/log/PlotBat.txt",'a+')
                f.write("{};{};{};{}\n".format((int(mymower.millis)-firstplotBatx)/1000,float(mymower.chgVoltage) , float(mymower.chgSense), float(mymower.batVoltage)))
                f.close()


            if message.sentence_type =='PER': #to refresh the plot page of perimeter
                global firstplotPerx
                
                mymower.millis=message.millis
                mymower.perimeterMag=message.perimeterMag
                mymower.perimeterMagRight=message.perimeterMagRight
                                
                if firstplotPerx==0:
                    firstplotPerx=int(mymower.millis)
                
                tk_millis.set(mymower.millis)
                tk_perimeterMag.set(mymower.perimeterMag)
                tk_perimeterMagRight.set(mymower.perimeterMagRight)
                
                f=open("/home/pi/Documents/PiArdumower/log/PlotPeri.txt",'a+')
                f.write("{};{};{}\n".format((int(mymower.millis)-firstplotPerx)/1000,float(mymower.perimeterMag) , float(mymower.perimeterMagRight)))
                f.close()
         
            if message.sentence_type =='STA': #permanent message for status info
                text1.config(text='State is ' + myRobot.stateNames[int(message.state)])
                mymower.millis=message.millis
                mymower.state=int(message.state)
                mymower.odox=message.odox
                mymower.odoy=message.odoy
                mymower.prevYaw=message.prevYaw
                mymower.batVoltage=message.batVoltage
                mymower.yaw=message.yaw
                mymower.pitch=message.pitch
                mymower.roll=message.roll
                mymower.Dht22Temp=message.Dht22Temp


                tk_batVoltage.set(mymower.batVoltage)


                
                tk_ImuYaw.set(180*float(mymower.yaw)/math.pi)
                tk_ImuPitch.set(180*float(mymower.pitch)/math.pi)
                tk_ImuRoll.set(180*float(mymower.roll)/math.pi)
                tk_Dht22Temp.set(mymower.Dht22Temp)
                #tk_Dht22Humid=tk.DoubleVar()

                
                Status.set("Yaw : "+mymower.yaw+ " Pitch : "+mymower.pitch+" Roll : "+mymower.roll)
                Status1.set("Batterie Voltage : "+mymower.batVoltage)
                MainStatusLine.set("State is : " + myRobot.stateNames[mymower.state])
                


            if message.sentence_type =='RET': #to fill the setting page All or name of the needed page
               
                if message.setting_page =='Time':
                     
                     myDate.hour = message.val1;
                     myDate.minute = message.val2;
                     myDate.dayOfWeek = message.val3;
                     myDate.day = message.val4;
                     myDate.month = message.val5;
                     myDate.year = message.val6;
                     tk_date_hour.set(myDate.hour)
                     tk_date_minute.set(myDate.minute)
                     tk_date_dayOfWeek.set(myDate.dayOfWeek)
                     tk_date_day.set(myDate.day)
                     tk_date_month.set(myDate.month)
                     tk_date_year.set(myDate.year)
                     
                if message.setting_page =='Timer':
                    for i in range(4):
                        if message.pageNr == str(i):
                            myRobot.Timeractive[i]=int(message.val1)
                            myRobot.TimerstartTime_hour[i]=int(message.val2)
                            myRobot.TimerstartTime_minute[i]=int(message.val3)
                            myRobot.TimerstopTime_hour[i]=int(message.val4)
                            myRobot.TimerstopTime_minute[i]=int(message.val5)
                            myRobot.TimerstartDistance[i]=int(message.val6)
                            myRobot.TimerstartMowPattern[i]=int(message.val7)
                            myRobot.TimerstartNrLane[i]=int(message.val8)
                            myRobot.TimerstartRollDir[i]=int(message.val9)
                            myRobot.TimerstartLaneMaxlengh[i]=int(message.val10)
                            
                            tk_timerActive[i].set(myRobot.Timeractive[i])
                            tk_timerStartTimehour[i].set(myRobot.TimerstartTime_hour[i])
                            tk_timerStartTimeMinute[i].set(myRobot.TimerstartTime_minute[i])
                            tk_timerStopTimehour[i].set(myRobot.TimerstopTime_hour[i])
                            tk_timerStopTimeMinute[i].set(myRobot.TimerstopTime_minute[i])
                            tk_timerStartDistance[i].set(myRobot.TimerstartDistance[i])
                            tk_timerStartMowPattern[i].set(myRobot.TimerstartMowPattern[i])
                            tk_timerStartNrLane[i].set(myRobot.TimerstartNrLane[i])
                            tk_timerStartRollDir[i].set(myRobot.TimerstartRollDir[i])
                            tk_timerStartLaneMaxlengh[i].set(myRobot.TimerstartLaneMaxlengh[i])
                           
                            
                    if message.pageNr =='5':
                        #the day is store into a bcd int value (00000010=2=mardi 00000100 =4=mercredi etc)
                        myRobot.TimerdaysOfWeek[0]=int(message.val1)
                        myRobot.TimerdaysOfWeek[1]=int(message.val2)
                        myRobot.TimerdaysOfWeek[2]=int(message.val3)
                        myRobot.TimerdaysOfWeek[3]=int(message.val4)
                        myRobot.TimerdaysOfWeek[4]=int(message.val5)
                        
                        for i in range(5):
                            for j in range(7):
                                
                                result=[bool((myRobot.TimerdaysOfWeek[i]) & (1<<n)) for n in range(8)]
                                #print(result)
                                tk_timerDayVar[i][j].set(result[j])
                                
                            
                           
                  
      
                            












                        
                    
                  
                
                if message.setting_page =='All':
                    if message.pageNr =='1':
                        myRobot.developerActive=message.val1
                        myRobot.motorAccel=message.val2
                        myRobot.motorSpeedMaxRpm=message.val3
                        myRobot.motorSpeedMaxPwm=message.val4
                        myRobot.motorPowerMax=message.val5
                        myRobot.motorSenseRightScale=message.val6
                        myRobot.motorSenseLeftScale=message.val7
                        myRobot.motorRollDegMax=message.val8
                        myRobot.motorRollDegMin=message.val9
                        myRobot.DistPeriOutRev=message.val10
                

                    if message.pageNr =='2':   
                        myRobot.motorPowerIgnoreTime=message.val1
                        myRobot.motorForwTimeMax=message.val2
                        myRobot.motorMowSpeedMaxPwm=message.val3
                        myRobot.motorMowPowerMax=message.val4
                        myRobot.motorMowRPMSet=message.val5
                        myRobot.motorMowSenseScale=message.val6
                        myRobot.motorLeftPID_Kp=message.val7
                        myRobot.motorLeftPID_Ki=message.val8
                        myRobot.motorLeftPID_Kd=message.val9
                        myRobot.motorMowPID_Kp=message.val10
                    if message.pageNr =='3':  
                        myRobot.motorMowPID_Ki=message.val1
                        myRobot.motorMowPID_Kd=message.val2
                        myRobot.motorBiDirSpeedRatio1=message.val3
                        myRobot.motorBiDirSpeedRatio2=message.val4
                        myRobot.motorLeftSwapDir=message.val5
                        myRobot.motorRightSwapDir=message.val6
                        myRobot.bumperUse=message.val7
                        myRobot.sonarUse=message.val8
                        myRobot.sonarCenterUse=message.val9
                        myRobot.sonarLeftUse=message.val10
                    if message.pageNr =='4': 
                        myRobot.sonarRightUse=message.val1
                        myRobot.sonarTriggerBelow=message.val2
                        myRobot.perimeterUse=message.val3
                        myRobot.perimeter_timedOutIfBelowSmag=message.val4
                        myRobot.perimeterTriggerTimeout=message.val5
                        myRobot.perimeterOutRollTimeMax=message.val6
                        myRobot.perimeterOutRollTimeMin=message.val7
                        myRobot.perimeterOutRevTime=message.val8
                        myRobot.perimeterTrackRollTime =message.val9
                        myRobot.perimeterTrackRevTime=message.val10
                    if message.pageNr =='5':     
                        myRobot.perimeterPID_Kp=message.val1
                        myRobot.perimeterPID_Ki=message.val2
                        myRobot.perimeterPID_Kd=message.val3
                        myRobot.perimeter_signalCodeNo=message.val4
                        myRobot.perimeter_swapCoilPolarityLeft=message.val5
                        myRobot.perimeter_timeOutSecIfNotInside=message.val6
                        myRobot.trakBlockInnerWheel=message.val7
                        myRobot.lawnSensorUse=message.val8
                        myRobot.imuUse=message.val9
                        myRobot.stopMotorDuringCalib=message.val10
                    if message.pageNr =='6':     
                        myRobot.imuDirPID_Kp=message.val1
                        myRobot.imuDirPID_Ki=message.val2
                        myRobot.imuDirPID_Kd=message.val3
                        myRobot.imuRollPID_Kp=message.val4
                        myRobot.imuRollPID_Ki=message.val5
                        myRobot.imuRollPID_Kd=message.val6
                        myRobot.remoteUse=message.val7
                        myRobot.batMonitor=message.val8
                        myRobot.batGoHomeIfBelow=message.val9
                        myRobot.batSwitchOffIfBelow=message.val10
                    if message.pageNr =='7':                       
                        myRobot.batSwitchOffIfIdle=message.val1
                        myRobot.batFactor=message.val2
                        myRobot.batChgFactor=message.val3
                        myRobot.chgSenseZero=message.val4
                        myRobot.batSenseFactor=message.val5
                        myRobot.batFullCurrent=message.val6
                        myRobot.startChargingIfBelow=message.val7
                        myRobot.stationRevDist=message.val8
                        myRobot.stationRollAngle=message.val9
                        myRobot.stationForwDist=message.val10
                    if message.pageNr =='8':    
                        myRobot.stationCheckDist=message.val1
                        myRobot.odometryUse=message.val2
                        myRobot.odometryTicksPerRevolution=message.val3
                        myRobot.odometryTicksPerCm=message.val4
                        myRobot.odometryWheelBaseCm=message.val5
                        myRobot.odometryLeftSwapDir=message.val6
                        myRobot.odometryRightSwapDir=message.val7
                        myRobot.twoWayOdometrySensorUse=message.val8
                        myRobot.buttonUse=message.val9
                        myRobot.userSwitch1=message.val10
                    if message.pageNr =='9':    
                        myRobot.userSwitch2=message.val1
                        myRobot.userSwitch3=message.val2
                        myRobot.timerUse=message.val3
                        myRobot.rainUse=message.val4
                        myRobot.gpsUse=message.val5
                        myRobot.stuckIfGpsSpeedBelow=message.val6
                        myRobot.gpsSpeedIgnoreTime=message.val7
                        myRobot.dropUse=message.val8
                        myRobot.statsOverride=message.val9
                        myRobot.bluetoothUse=message.val10
                    if message.pageNr =='10':    
                        myRobot.esp8266Use=message.val1
                        myRobot.esp8266ConfigString=message.val2
                        myRobot.tiltUse=message.val3
                        myRobot.trackingPerimeterTransitionTimeOut=message.val4
                        myRobot.motorMowForceOff=message.val5
                        myRobot.MaxSpeedperiPwm=message.val6
                        myRobot.RollTimeFor45Deg=message.val7
                        myRobot.DistPeriObstacleAvoid=message.val8
                        myRobot.circleTimeForObstacle=message.val9
                        myRobot.DistPeriOutRev=message.val10
                    if message.pageNr =='11':
                        myRobot.motorRightOffsetFwd=message.val1
                        myRobot.motorRightOffsetRev=message.val2
                        myRobot.perimeterMagMaxValue=message.val3
                        myRobot.SpeedOdoMin=message.val4
                        myRobot.SpeedOdoMax=message.val5
                        myRobot.yawSet1=message.val6
                        myRobot.yawSet2=message.val7
                        myRobot.yawSet3=message.val8
                        myRobot.yawOppositeLane1RollRight=message.val9
                        myRobot.yawOppositeLane2RollRight=message.val10
                    if message.pageNr =='12':    
                        myRobot.yawOppositeLane3RollRight=message.val1
                        myRobot.yawOppositeLane1RollLeft=message.val2
                        myRobot.yawOppositeLane2RollLeft=message.val3
                        myRobot.yawOppositeLane3RollLeft=message.val4
                        myRobot.DistBetweenLane=message.val5
                        myRobot.maxLenghtByLane=message.val6
                        myRobot.perimeter_swapCoilPolarityRight=message.val7
                        myRobot.perimeter_read2Coil=message.val8
                        myRobot.maxDriftPerSecond=message.val9
                        myRobot.delayBetweenTwoDmpAutocalib=message.val10
                    if message.pageNr =='13':    
                        myRobot.maxDurationDmpAutocalib=message.val1
                        myRobot.mowPatternDurationMax=message.val2
                        myRobot.DistPeriOutStop=message.val3
                        myRobot.DHT22Use=message.val4
                        myRobot.RaspberryPIUse=message.val5
 

                refreshAllSettingPage()                            
                   
                       
                        
                if message.setting_page =='Motor':
                    if message.pageNr =='1':
                       
                        myRobot.motorPowerMax=message.val1
                        myRobot.motorSpeedMaxRpm=message.val2
                        myRobot.motorSpeedMaxPwm=message.val3
                        myRobot.motorAccel=message.val4
                        myRobot.motorPowerIgnoreTime=message.val5
                        myRobot.motorRollDegMax=message.val6
                        myRobot.motorRollDegMin=message.val7
                        myRobot.DistPeriOutRev=message.val8
                        myRobot.DistPeriOutStop=message.val9
                        myRobot.motorLeftPID_Kp=message.val10
                                                
                    if message.pageNr =='2':
                        
                        myRobot.motorLeftPID_Ki=message.val1
                        myRobot.motorLeftPID_Kd=message.val2
                        myRobot.motorLeftSwapDir=message.val3
                        myRobot.motorRightSwapDir=message.val4
                        myRobot.motorRightOffsetFwd=message.val5
                        myRobot.motorRightOffsetRev=message.val6
                        myRobot.SpeedOdoMin=message.val7
                        myRobot.SpeedOdoMax=message.val8
                        myRobot.motorSenseLeftScale=message.val9
                        myRobot.motorSenseRightScale=message.val10
                   
                    refreshMotorSettingPage()
           
                
            if message.sentence_type == 'CFG':
                text1.config(text= message.debug)
                txtConsoleRecu.insert('1.0', message.debug)               
                
            if message.sentence_type == 'DEB':
                text1.config(text= message.debug)
                txtConsoleRecu.insert('1.0', message.debug)
        
        #txtRecu.delete('20.0', END) #keep only 20 lines
        #fen1.after(10,checkSerial) #to be sure empty the buffer read again immediatly
    
    fen1.after(50,checkSerial)  # here is the main loop each 50ms

    
def refreshAllSettingPage():
    
    refreshMotorSettingPage()
    refreshPerimeterSettingPage()
    refreshMainSettingPage()
    refreshImuSettingPage()
    refreshSonarSettingPage()
    refreshBatterySettingPage()
    refreshOdometrySettingPage()
    refreshMowMotorSettingPage()
    refreshByLaneSettingPage()
    refreshTimerSettingPage()
    


def ButtonSetMowMotorApply_click():
    myRobot.motorMowSpeedMaxPwm=slidermotorMowSpeedMaxPwm.get()
    myRobot.motorMowRPMSet=slidermotorMowRPMSet.get()
    myRobot.motorMowPID_Kp=slidermotorMowPID_Kp.get()
    myRobot.motorMowPID_Ki=slidermotorMowPID_Ki.get()
    myRobot.motorMowPID_Kd=slidermotorMowPID_Kd.get()
    myRobot.motorMowSenseScale=slidermotorMowSenseScale.get()
    myRobot.motorMowPowerMax=slidermotorMowPowerMax.get()
    myRobot.mowPatternDurationMax=slidermowPatternDurationMax.get()   
    myRobot.motorMowForceOff='0'
    if MowVar1.get()==1:
        myRobot.motorMowForceOff='1'
    ButtonSendSettingToDue_click()
    

def ButtonSetMotApply_click():
    myRobot.motorPowerMax=sliderPowerMax.get()
    myRobot.motorSpeedMaxRpm=sliderSpeedRpmMax.get()
    myRobot.motorSpeedMaxPwm=sliderSpeedPwmMax.get()
    myRobot.motorAccel=sliderAccel.get()
    myRobot.motorPowerIgnoreTime=sliderPowerIgnoreTime.get()
    myRobot.motorRollDegMax=sliderRollDegMax.get()
    myRobot.motorRollDegMin=sliderRollDegMin.get()
    myRobot.DistPeriOutRev=sliderRevDist.get()
    myRobot.DistPeriOutStop=sliderStopDist.get()
    myRobot.motorLeftPID_Kp=sliderPidP.get()
    myRobot.motorLeftPID_Ki=sliderPidI.get()
    myRobot.motorLeftPID_Kd=sliderPidD.get()
    myRobot.motorLeftSwapDir='0'
    if MotVar1.get()==1:
        myRobot.motorLeftSwapDir='1'
    myRobot.motorRightSwapDir='0'
    if MotVar2.get()==1:
        myRobot.motorRightSwapDir='1'
    myRobot.motorRightOffsetFwd=sliderRightFwOffset.get()
    myRobot.motorRightOffsetRev=sliderRightRevOffset.get()
    myRobot.SpeedOdoMin=sliderSpeedOdoMin.get()
    myRobot.SpeedOdoMax=sliderSpeedOdoMax.get()
    myRobot.motorSenseLeftScale=sliderLeftSense.get()
    myRobot.motorSenseRightScale=sliderRightSense.get()
    ButtonSendSettingToDue_click()
    
    """PFOD VERSION not use because more slower than rewrite all variable                    
    send_pfo_message('a02',''+str(sliderPowerMax.get())+'','2','3','4','5','6',)
    send_pfo_message('a06',''+str(sliderSpeedRpmMax.get())+'','2','3','4','5','6',)
    send_pfo_message('a15',''+str(sliderSpeedPwmMax.get())+'','2','3','4','5','6',)
    send_pfo_message('a11',''+str(sliderAccel.get())+'','2','3','4','5','6',)
    send_pfo_message('a07',''+str(sliderRollDegMax.get())+'','2','3','4','5','6',)
    send_pfo_message('a19',''+str(sliderRollDegMin.get())+'','2','3','4','5','6',)
    send_pfo_message('a18',''+str(sliderPowerIgnoreTime.get())+'','2','3','4','5','6',)
    send_pfo_message('a08',''+str(sliderRevDist.get())+'','2','3','4','5','6',)
    send_pfo_message('a09',''+str(sliderStopDist.get())+'','2','3','4','5','6',)
    #send_pfo_message('a14',''+str(sliderPidP.get())+'',''+str(sliderPidI.get())+'',''+str(sliderPidD.get())+'','4','5','6',)
    send_pfo_message('a22',''+str(sliderRightFwOffset.get())+'','2','3','4','5','6',)
    send_pfo_message('a23',''+str(sliderRightRevOffset.get())+'','2','3','4','5','6',)
    send_pfo_message('a30',''+str(sliderSpeedOdoMin.get())+'','2','3','4','5','6',)
    send_pfo_message('a31',''+str(sliderSpeedOdoMax.get())+'','2','3','4','5','6',)
    send_pfo_message('a20',''+str(sliderLeftSense.get())+'','2','3','4','5','6',)
    send_pfo_message('a21',''+str(sliderRightSense.get())+'','2','3','4','5','6',)
    send_var_message('w','motorLeftSwapDir',''+str(MotVar1.get())+'','motorRightSwapDir',''+str(MotVar2.get())+'','0','0','0','0','0')
    
    messagebox.showinfo('Info','The Motor Setting are send Do not forget to save')
    """


def ButtonSetBatteryApply_click():
    myRobot.batGoHomeIfBelow=sliderbatGoHomeIfBelow.get()
    myRobot.batSwitchOffIfBelow=sliderbatSwitchOffIfBelow.get()
    myRobot.batSwitchOffIfIdle=sliderbatSwitchOffIfIdle.get()
    myRobot.startChargingIfBelow=sliderstartChargingIfBelow.get()
    myRobot.batFullCurrent=sliderbatFullCurrent.get()

    myRobot.batFactor=sliderbatFactor.get()
    myRobot.batChgFactor=sliderbatChgFactor.get()
    myRobot.batSenseFactor=sliderbatSenseFactor.get()

    myRobot.batMonitor='0'
    if BatVar1.get()==1:
        myRobot.batMonitor='1'
    ButtonSendSettingToDue_click()

    
def ButtonSetSonarApply_click(): 
    myRobot.sonarTriggerBelow=slidersonarTriggerBelow.get()     
    myRobot.sonarCenterUse='0'
    if SonVar1.get()==1:
        myRobot.sonarCenterUse='1'
    myRobot.sonarLeftUse='0'
    if SonVar2.get()==1:
        myRobot.sonarLeftUse='1'
    myRobot.sonarRightUse='0'
    if SonVar3.get()==1:
        myRobot.sonarRightUse='1'
    ButtonSendSettingToDue_click()   
    



def ButtonSetOdometryApply_click():
    myRobot.odometryTicksPerRevolution=sliderodometryTicksPerRevolution.get()
    myRobot.odometryTicksPerCm=sliderodometryTicksPerCm.get()
    myRobot.odometryWheelBaseCm=sliderodometryWheelBaseCm.get()
    
   
    ButtonSendSettingToDue_click()
   
def ButtonSetImuApply_click():
    myRobot.imuDirPID_Kp=sliderimuDirPID_Kp.get()
    myRobot.imuDirPID_Ki=sliderimuDirPID_Ki.get()
    myRobot.imuDirPID_Kd=sliderimuDirPID_Kd.get()
    
    myRobot.delayBetweenTwoDmpAutocalib=sliderdelayBetweenTwoDmpAutocalib.get()
    myRobot.maxDurationDmpAutocalib=slidermaxDurationDmpAutocalib.get()
    myRobot.maxDriftPerSecond=slidermaxDriftPerSecond.get()
    
    myRobot.stopMotorDuringCalib='0'
    if ImuVar1.get()==1:
        myRobot.stopMotorDuringCalib='1'
    ButtonSendSettingToDue_click()
    

def ButtonSetPerimeterApply_click():
    myRobot.perimeter_timedOutIfBelowSmag=sliderTimeBelowSmag.get()
    myRobot.perimeter_timeOutSecIfNotInside=sliderTimeNotInside.get()
    myRobot.perimeterTriggerTimeout=sliderTrigTimeout.get()
    myRobot.MaxSpeedperiPwm=sliderTrackingSpeed.get()
    myRobot.DistPeriObstacleAvoid=sliderCircleArcDistance.get()
    myRobot.perimeterMagMaxValue=sliderPeriMagMaxValue.get()
    myRobot.trackingPerimeterTransitionTimeOut=sliderTransitionTimeout.get()

    myRobot.trackingErrorTimeOut=sliderTrackErrTimeout.get()
    myRobot.perimeterPID_Kp=sliderTrackPid_P.get()
    myRobot.perimeterPID_Ki=sliderTrackPid_I.get()
    myRobot.perimeterPID_Kd=sliderTrackPid_D.get()

    myRobot.perimeter_swapCoilPolarityLeft='0'
    if PeriVar1.get()==1:
        myRobot.perimeter_swapCoilPolarityLeft='1'
    myRobot.perimeter_swapCoilPolarityRight='0'
    if PeriVar2.get()==1:
        myRobot.perimeter_swapCoilPolarityRight='1'
    myRobot.perimeter_read2Coil='0'
    if PeriVar3.get()==1:
        myRobot.perimeter_read2Coil='1'
    myRobot.trakBlockInnerWheel='0'
    if PeriVar4.get()==1:
        myRobot.trakBlockInnerWheel='1'
    ButtonSendSettingToDue_click()

def ButtonSetMainApply_click():
    myRobot.perimeterUse='0'
    if MainperimeterUse.get()==1:
        myRobot.perimeterUse='1'   
    myRobot.imuUse='0'
    if MainimuUse.get()==1:
        myRobot.imuUse='1'        
    myRobot.gpsUse='0'
    if MaingpsUse.get()==1:
        myRobot.gpsUse='1'        
    myRobot.bluetoothUse='0'
    if MainbluetoothUse.get()==1:
        myRobot.bluetoothUse='1'      
    myRobot.bumperUse='0'
    if MainbumperUse.get()==1:
        myRobot.bumperUse='1'        
    myRobot.sonarUse='0'
    if MainsonarUse.get()==1:
        myRobot.sonarUse='1'        
    myRobot.DHT22Use='0'
    if MainDHT22Use.get()==1:
        myRobot.DHT22Use='1'       
    myRobot.lawnSensorUse='0'
    if MainlawnSensorUse.get()==1:
        myRobot.lawnSensorUse='1'
    myRobot.timerUse='0'
    if MaintimerUse.get()==1:
        myRobot.timerUse='1'        
    myRobot.rainUse='0'
    if MainrainUse.get()==1:
        myRobot.rainUse='1'       
    myRobot.dropUse='0'
    if MaindropUse.get()==1:
        myRobot.dropUse='1'        
    myRobot.esp8266Use='0'
    if Mainesp8266Use.get()==1:
        myRobot.esp8266Use='1'
    myRobot.tiltUse='0'
    if MaintiltUse.get()==1:
        myRobot.tiltUse='1'
        
    ButtonSendSettingToDue_click()


def BtnMowPlotStartRec_click():    
    mowPlotterKst.start('/home/pi/Documents/PiArdumower/plotMow.kst')
    send_req_message('MOW',''+str(SldMainMowRefresh.get())+'','1','10000','0','0','0',) #arduino start sending data
    
def BtnMowPlotStopRec_click():
    global firstplotMowx
    firstplotMowx=0  #initialise the first time plot for next plot
    mowPlotterKst.stop() #close the kst prog
    
    send_req_message('MOW','1','0','0','0','0','0',) #arduino stop sending data
    
    filename="/home/pi/Documents/PiArdumower/log/Plotmow" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:  #avoid error if file not exit 
        os.rename("/home/pi/Documents/PiArdumower/log/PlotMow.txt",filename) #keep a copy of the plot and clear the last kst file
    except OSError:
        pass
    
    

    """recreate an empty txt file to have correct auto legend into the graph """
    f=open("/home/pi/Documents/PiArdumower/log/PlotMow.txt",'w')
    f.write("{};{};{}\n".format("Time","motorMowSense","motorMowPWMCurr"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    

def BtnPeriPlotStartRec_click():
    periPlotterKst.start('/home/pi/Documents/PiArdumower/plotPeri.kst')
    send_req_message('PERI',''+str(SldMainPeriRefresh.get())+'','1','10000','0','0','0',)
    
def BtnPeriPlotStopRec_click():
    global firstplotPerx
    firstplotPerx=0
    periPlotterKst.stop() #close the kst prog
    
    send_req_message('PERI','1','0','0','0','0','0',)
    
    filename="/home/pi/Documents/PiArdumower/log/PlotPeri" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename("/home/pi/Documents/PiArdumower/log/PlotPeri.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in log directory")
    except OSError:
        pass
    """recreate an empty txt file to have correct auto legend into the graph """
    f=open("/home/pi/Documents/PiArdumower/log/PlotPeri.txt",'w')
    f.write("{};{};{}\n".format("Time","perimeterMag","perimeterMagRight"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    
def BtnBatPlotStartRec_click():
   
    batPlotterKst.start('/home/pi/Documents/PiArdumower/plotBat.kst')
    send_req_message('BAT',''+str(SldMainBatRefresh.get())+'','1','10000','0','0','0',)    

def BtnBatPlotStopRec_click():
    global firstplotBatx
    firstplotBatx=0
    batPlotterKst.stop() #close the kst prog
    
    send_req_message('BAT','1','0','0','0','0','0',)
    
    filename="/home/pi/Documents/PiArdumower/log/PlotBattery" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename("/home/pi/Documents/PiArdumower/log/PlotBat.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in log directory")
    except OSError:
        pass
    """recreate an empty txt file to have correct auto legend into the graph """
    f=open("/home/pi/Documents/PiArdumower/log/PlotBat.txt",'w')
    f.write("{};{};{};{}\n".format("Time","chgVoltage","chgSense","batVoltage"))
    f.write("{};{};{};{}\n".format("0","0","0","0"))
    f.close()
   
def BtnBylaneStartRec_click():
    send_req_message('BYL','3','1','6000','0','0','0',)
    
def BtnBylaneStopRec_click():
    send_req_message('BYL','1','0','0','0','0','0',)
    
def BtnMotPlotStartRec_click():
    motPlotterKst.start('/home/pi/Documents/PiArdumower/plotMot.kst')
    send_req_message('MOT',''+str(SldMainWheelRefresh.get())+'','1','10000','0','0','0',)
    
def BtnMotPlotStopRec_click():
    global firstplotMotx
    firstplotMotx=0
    motPlotterKst.stop() #close the kst prog
    
    send_req_message('MOT','1','0','0','0','0','0',)
    
    filename="/home/pi/Documents/PiArdumower/log/PlotMotor" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename("/home/pi/Documents/PiArdumower/log/PlotMot.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in log directory")
    except OSError:
        pass
    """recreate an empty txt file to have correct auto legend into the graph """
    f=open("/home/pi/Documents/PiArdumower/log/PlotMot.txt",'w')
    f.write("{};{};{};{};{}\n".format("Time","motorLeftSenseCurrent","motorRightSenseCurrent","motorLeftPWMCurr","motorRightPWMCurr"))
    f.write("{};{};{};{};{}\n".format("0","0","0","0","0"))
    f.close()

def BtnMotPlotStopAll_click():
    BtnMotPlotStopRec_click()
    BtnBatPlotStopRec_click()
    BtnPeriPlotStopRec_click()
    BtnMowPlotStopRec_click()
    

def refreshOdometrySettingPage():

    sliderodometryTicksPerRevolution.set(myRobot.odometryTicksPerRevolution)
    sliderodometryTicksPerCm.set(myRobot.odometryTicksPerCm)
    sliderodometryWheelBaseCm.set(myRobot.odometryWheelBaseCm)
    
def refreshByLaneSettingPage():

    sliderDistBetweenLane.set(myRobot.DistBetweenLane)
    slidermaxLenghtByLane.set(myRobot.maxLenghtByLane)
    slideryawSet1.set(myRobot.yawSet1)
    slideryawOppositeLane1RollRight.set(myRobot.yawOppositeLane1RollRight)
    slideryawOppositeLane1RollLeft.set(myRobot.yawOppositeLane1RollLeft)
    slideryawSet2.set(myRobot.yawSet2)
    slideryawOppositeLane2RollRight.set(myRobot.yawOppositeLane2RollRight)
    slideryawOppositeLane2RollLeft.set(myRobot.yawOppositeLane2RollLeft)
    slideryawSet3.set(myRobot.yawSet3)
    slideryawOppositeLane3RollRight.set(myRobot.yawOppositeLane3RollRight)
    slideryawOppositeLane3RollLeft.set(myRobot.yawOppositeLane3RollLeft)


    

def refreshMowMotorSettingPage():
   
    slidermowPatternDurationMax.set(myRobot.mowPatternDurationMax)
    slidermotorMowSpeedMaxPwm.set(myRobot.motorMowSpeedMaxPwm)
    slidermotorMowPowerMax.set(myRobot.motorMowPowerMax)
    slidermotorMowRPMSet.set(myRobot.motorMowRPMSet)    
    slidermotorMowSenseScale.set(myRobot.motorMowSenseScale)
    slidermotorMowPID_Kp.set(myRobot.motorMowPID_Kp)
    slidermotorMowPID_Ki.set(myRobot.motorMowPID_Ki)
    slidermotorMowPID_Kd.set(myRobot.motorMowPID_Kd)
        
    ChkBtnmotorMowForceOff.deselect()
    if myRobot.motorMowForceOff=='1':
        ChkBtnmotorMowForceOff.select()   

def refreshBatterySettingPage():

    sliderbatGoHomeIfBelow.set(myRobot.batGoHomeIfBelow)
    sliderbatSwitchOffIfBelow.set(myRobot.batSwitchOffIfBelow)
    sliderbatSwitchOffIfIdle.set(myRobot.batSwitchOffIfIdle)
    sliderstartChargingIfBelow.set(myRobot.startChargingIfBelow)
    
    sliderbatFullCurrent.set(myRobot.batFullCurrent)
    sliderbatFactor.set(myRobot.batFactor)

    sliderbatChgFactor.set(myRobot.batChgFactor)
    sliderbatSenseFactor.set(myRobot.batSenseFactor)
        
    ChkBtnbatMonitor.deselect()
    if myRobot.batMonitor=='1':
        ChkBtnbatMonitor.select()
    
    
def refreshSonarSettingPage():
    
    slidersonarTriggerBelow.set(myRobot.sonarTriggerBelow)  
        
    ChkBtnsonarRightUse.deselect()
    if myRobot.sonarRightUse=='1':
        ChkBtnsonarRightUse.select()
    
    ChkBtnsonarLeftUse.deselect()
    if myRobot.sonarLeftUse=='1':
        ChkBtnsonarLeftUse.select()
    ChkBtnsonarCenterUse.deselect()
    if myRobot.sonarCenterUse=='1':
        ChkBtnsonarCenterUse.select()

def refreshImuSettingPage():
    sliderimuDirPID_Kp.set(myRobot.imuDirPID_Kp)
    sliderimuDirPID_Ki.set(myRobot.imuDirPID_Ki)
    sliderimuDirPID_Kd.set(myRobot.imuDirPID_Kd)
    
    sliderdelayBetweenTwoDmpAutocalib.set(myRobot.delayBetweenTwoDmpAutocalib)
    slidermaxDurationDmpAutocalib.set(myRobot.maxDurationDmpAutocalib)
    slidermaxDriftPerSecond.set(myRobot.maxDriftPerSecond)
    
    
    ChkBtnstopMotorDuringCalib.deselect()
    if myRobot.stopMotorDuringCalib=='1':
        ChkBtnstopMotorDuringCalib.select()

        
def refreshTimerSettingPage():
    for i in range(5):
        tk_timerActive[i].set(myRobot.Timeractive[i])
        tk_timerStartTimehour[i].set(myRobot.TimerstartTime_hour[i])
        tk_timerStartTimeMinute[i].set(myRobot.TimerstartTime_minute[i])
        tk_timerStopTimehour[i].set(myRobot.TimerstopTime_hour[i])
        tk_timerStopTimeMinute[i].set(myRobot.TimerstopTime_minute[i])
        tk_timerStartDistance[i].set(myRobot.TimerstartDistance[i])
        tk_timerStartMowPattern[i].set(myRobot.TimerstartMowPattern[i])
        tk_timerStartNrLane[i].set(myRobot.TimerstartNrLane[i])
        tk_timerStartRollDir[i].set(myRobot.TimerstartRollDir[i])
        tk_timerStartLaneMaxlengh[i].set(myRobot.TimerstartLaneMaxlengh[i])
                           
        for j in range(7):
            tk_timerDayVar[i][j].set(myRobot.TimerdaysOfWeek[i])
  

  
def refreshMotorSettingPage():
    sliderPowerMax.set(myRobot.motorPowerMax)
    sliderSpeedRpmMax.set(myRobot.motorSpeedMaxRpm)
    sliderSpeedPwmMax.set(myRobot.motorSpeedMaxPwm)
    sliderAccel.set(myRobot.motorAccel)
    sliderPowerIgnoreTime.set(myRobot.motorPowerIgnoreTime)
    sliderRollDegMax.set(myRobot.motorRollDegMax)
    sliderRollDegMin.set(myRobot.motorRollDegMin)
    sliderRevDist.set(myRobot.DistPeriOutRev)
    sliderStopDist.set(myRobot.DistPeriOutStop)
    sliderPidP.set(myRobot.motorLeftPID_Kp)
    sliderPidI.set(myRobot.motorLeftPID_Ki)
    sliderPidD.set(myRobot.motorLeftPID_Kd)
    ChkBtnMotorSwapLeftDir.deselect()
    ChkBtnMotorSwapRightDir.deselect()
    if myRobot.motorLeftSwapDir=='1':
        ChkBtnMotorSwapLeftDir.select()
    if myRobot.motorRightSwapDir=='1':
        ChkBtnMotorSwapRightDir.select()
    sliderRightFwOffset.set(myRobot.motorRightOffsetFwd)
    sliderRightRevOffset.set(myRobot.motorRightOffsetRev)
    sliderSpeedOdoMin.set(myRobot.SpeedOdoMin)
    sliderSpeedOdoMax.set(myRobot.SpeedOdoMax)
    sliderLeftSense.set(myRobot.motorSenseLeftScale)
    sliderRightSense.set(myRobot.motorSenseRightScale)
    #ButtonSendSettingToDue_click

def refreshMainSettingPage():
    ChkBtnperimeterUse.deselect()
    if myRobot.perimeterUse=='1':
        ChkBtnperimeterUse.select()
    ChkBtnimuUse.deselect()
    if myRobot.imuUse=='1':
        ChkBtnimuUse.select()
    ChkBtngpsUse.deselect()
    if myRobot.gpsUse=='1':
        ChkBtngpsUse.select()
    ChkBtnbluetoothUse.deselect()
    if myRobot.bluetoothUse=='1':
        ChkBtnbluetoothUse.select()
    ChkBtnbumperUse.deselect()
    if myRobot.bumperUse=='1':
        ChkBtnbumperUse.select()
    ChkBtnsonarUse.deselect()
    if myRobot.sonarUse=='1':
        ChkBtnsonarUse.select()
    ChkBtnDHT22Use.deselect()
    if myRobot.DHT22Use=='1':
        ChkBtnDHT22Use.select()
    ChkBtnlawnSensorUse.deselect()
    if myRobot.lawnSensorUse=='1':
        ChkBtnlawnSensorUse.select()  
    ChkBtntimerUse.deselect()
    if myRobot.timerUse=='1':
        ChkBtntimerUse.select()     
    ChkBtnrainUse.deselect()
    if myRobot.rainUse=='1':
        ChkBtnrainUse.select()      
    ChkBtndropUse.deselect()
    if myRobot.dropUse=='1':
        ChkBtndropUse.select()        
    ChkBtnesp8266Use.deselect()
    if myRobot.esp8266Use=='1':
        ChkBtnesp8266Use.select()
    ChkBtntiltUse.deselect()
    if myRobot.tiltUse=='1':
        ChkBtntiltUse.select()
    
def refreshPerimeterSettingPage():
    sliderTimeBelowSmag.set(myRobot.perimeter_timedOutIfBelowSmag)
    sliderTimeNotInside.set(myRobot.perimeter_timeOutSecIfNotInside)
    sliderTrigTimeout.set(myRobot.perimeterTriggerTimeout)
    sliderTrackingSpeed.set(myRobot.MaxSpeedperiPwm)
    sliderCircleArcDistance.set(myRobot.DistPeriObstacleAvoid)
    sliderPeriMagMaxValue.set(myRobot.perimeterMagMaxValue)
    sliderTransitionTimeout.set(myRobot.trackingPerimeterTransitionTimeOut )

    sliderTrackErrTimeout.set(myRobot.trackingErrorTimeOut)
    sliderTrackPid_P.set(myRobot.perimeterPID_Kp)
    sliderTrackPid_I.set(myRobot.perimeterPID_Ki)
    sliderTrackPid_D.set(myRobot.perimeterPID_Kd)

    ChkBtnPeriSwapLeftCoil.deselect()
    if myRobot.perimeter_swapCoilPolarityLeft=='1':
        ChkBtnPeriSwapLeftCoil.select()
    ChkBtnPeriSwapRightCoil.deselect()
    if myRobot.perimeter_swapCoilPolarityRight=='1':
        ChkBtnPeriSwapRightCoil.select()
    ChkBtnPeriRead2Coil.deselect()
    if myRobot.perimeter_read2Coil=='1':
        ChkBtnPeriRead2Coil.select()
    ChkBtnPeriBlockInnWheel.deselect()
    if myRobot.trakBlockInnerWheel=='1':
        ChkBtnPeriBlockInnWheel.select()
    
    ButtonSendSettingToDue_click
    
    
def ButtonSaveReceived_click():
    fileName="/home/pi/Documents/PiArdumower/log/" + time.strftime("%Y%m%d%H%M") + "_Received.txt" 
    with open(fileName,"w") as f:
        f.write(txtRecu.get('1.0','end'))
    fileName="/home/pi/Documents/PiArdumower/log/" + time.strftime("%Y%m%d%H%M") + "_Send.txt"
    with open(fileName,"w") as f:
        f.write(txtSend.get('1.0','end'))
    fileName="/home/pi/Documents/PiArdumower/log/" + time.strftime("%Y%m%d%H%M") + "_Console.txt"
    with open(fileName,"w") as f:
        f.write(txtConsoleRecu.get('1.0','end'))

    txtConsoleRecu.insert('1.0', 'All Console file are saved')
    






    
def ButtonForward_click():
    ButtonReverse.configure(state='disabled')
    send_var_message('w','motorSpeedMaxPwm',''+str(manualSpeedSlider.get())+'','0','0','0','0','0','0','0')
    send_pfo_message('nf','1','2','3','4','5','6',)
def ButtonRight_click():
    send_pfo_message('nr','1','2','3','4','5','6',)
def ButtonLeft_click():
    send_pfo_message('nl','1','2','3','4','5','6',)
def ButtonReverse_click():
    ButtonForward.configure(state='disabled')
    send_pfo_message('nb','1','2','3','4','5','6',)
def ButtonStop_click():
    ButtonForward.configure(state='normal')
    ButtonReverse.configure(state='normal')
    send_pfo_message('ns','1','2','3','4','5','6',)
     

def button_home_click():
    send_pfo_message('rh','1','2','3','4','5','6',)
    
def button_track_click():
    send_pfo_message('rk','1','2','3','4','5','6',)

def button_stop_all_click():
    send_pfo_message('ro','1','2','3','4','5','6',)

def ButtonInfo_click():
    send_req_message('INF','2','0','1','0','0','0',)
    InfoPage.tkraise()
    
def ButtonCamera_click():
    StreamVideoPage.tkraise()
def ButtonSchedule_click():
    TabTimer.tkraise()
    
    
def ButtonGyroCal_click():
    send_pfo_message('ro','1','2','3','4','5','6',)

def ButtonCompasCal_click():
    send_pfo_message('ro','1','2','3','4','5','6',)











    
def ButtonStartMow_click():
    send_var_message('w','mowPatternCurr',''+str(tk_mowingPattern.get())+'','0','0','0','0','0','0','0')
    send_pfo_message('ra','1','2','3','4','5','6',)
   
def buttonBlade_stop_click():
    send_cmd_message('mowmotor','0','0','0','0')
    
def buttonBlade_start_click():
    send_cmd_message('mowmotor','1','0','0','4')
    
def ButtonSendSettingToEeprom_click():
    send_pfo_message('sz','1','2','3','4','5','6',)

def ButtonManual_click():
    manualSpeedSlider.set(myRobot.motorSpeedMaxPwm)
    ManualPage.tkraise()
        
def ButtonPlot_click():
    TabPlot.tkraise()

def ButtonSetting_click():
    TabSetting.tkraise()
   
def ButtonBackToMain_click():
    MainPage.tkraise()
    
                    
def ButtonConsole_click():
    ConsolePage.tkraise()

def ButtonTest_click():
    TestPage.tkraise()
    
def ButtonAuto_click():
    AutoPage.tkraise()
    
   

def ButtonOdo1TurnFw_click():
    send_pfo_message('yt0','1','2','3','4','5','6',)
def ButtonOdo5TurnFw_click():
    send_pfo_message('yt1','1','2','3','4','5','6',)
def ButtonOdo1TurnRev_click():
    send_pfo_message('yt2','1','2','3','4','5','6',)
def ButtonOdo5TurnRev_click():
    send_pfo_message('yt3','1','2','3','4','5','6',)
def ButtonOdo3MlFw_click():
    send_pfo_message('yt4','1','2','3','4','5','6',) 
def ButtonOdoRot180_click():
    send_pfo_message('yt6','1','2','3','4','5','6',)
def ButtonOdoRot360_click():
    send_pfo_message('yt5','1','2','3','4','5','6',)
def ButtonOdoRotNonStop_click():
    send_pfo_message('yt7','1','2','3','4','5','6',)
    
   
    
def ButtonSaveSettingToFile_click():
    settingFileName = filedialog.asksaveasfilename(title="Save As :", initialdir=actualRep, initialfile='myrobotsetting.ini', filetypes = [("All", "*"),("File Setting","*.ini")])    
    if len(settingFileName) > 0:
        fileOnPi = open(settingFileName, 'wb')   # Overwrites any existing file.
        pickle.dump(myRobot, fileOnPi)
        
        

def ButtonReadSettingFromFile_click():
    global myRobot
    settingFileName = filedialog.askopenfilename(title="Open :", initialdir=actualRep,initialfile='myrobotsetting.ini', filetypes = [("All", "*"),("File Setting","*.ini")])    
    
    if len(settingFileName) > 0:
        fileOnPi = open(settingFileName,'rb') 
        myRobot = pickle.load(fileOnPi)
        refreshAllSettingPage()
        

def ButtonReadSettingFromDue_click():
    read_all_setting()


def ButtonSendSettingByLaneToDue_click():
       
    Send_reqSetting_message('ByLane','w','1',''+str(myRobot.yawSet1)+\
                            '',''+str(myRobot.yawSet2)+\
                            '',''+str(myRobot.yawSet3)+\
                            '',''+str(myRobot.yawOppositeLane1RollRight)+\
                            '',''+str(myRobot.yawOppositeLane2RollRight)+\
                            '',''+str(myRobot.yawOppositeLane3RollRight)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+'',)
    Send_reqSetting_message('ByLane','w','2',''+str(myRobot.yawOppositeLane1RollLeft)+\
                            '',''+str(myRobot.yawOppositeLane2RollLeft)+\
                            '',''+str(myRobot.yawOppositeLane3RollLeft)+\
                            '',''+str(myRobot.DistBetweenLane)+\
                            '',''+str(myRobot.maxLenghtByLane)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+'',)
    
    
def ButtonSendSettingDateTimeToDue_click():
    Send_reqSetting_message('Time','w','1',''+str(tk_date_hour.get())+\
                            '',''+str(tk_date_minute.get())+\
                            '',''+str(tk_date_dayOfWeek.get())+\
                            '',''+str(tk_date_day.get())+\
                            '',''+str(tk_date_month.get())+\
                            '',''+str(tk_date_year.get())+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+'',)
    
def ButtonSendSettingToDue_click():
    Send_reqSetting_message('All','w','1',''+str(myRobot.developerActive)+\
                            '',''+str(myRobot.motorAccel)+\
                            '',''+str(myRobot.motorSpeedMaxRpm)+\
                            '',''+str(myRobot.motorSpeedMaxPwm)+\
                            '',''+str(myRobot.motorPowerMax)+\
                            '',''+str(myRobot.motorSenseRightScale)+\
                            '',''+str(myRobot.motorSenseLeftScale)+\
                            '',''+str(myRobot.motorRollDegMax)+\
                            '',''+str(myRobot.motorRollDegMin)+\
                            '',''+str(myRobot.DistPeriOutRev)+'',)

    Send_reqSetting_message('All','w','2',''+str(myRobot.motorPowerIgnoreTime)+\
                            '',''+str(myRobot.motorForwTimeMax)+\
                            '',''+str(myRobot.motorMowSpeedMaxPwm)+\
                            '',''+str(myRobot.motorMowPowerMax)+\
                            '',''+str(myRobot.motorMowRPMSet)+\
                            '',''+str(myRobot.motorMowSenseScale)+\
                            '',''+str(myRobot.motorLeftPID_Kp)+\
                            '',''+str(myRobot.motorLeftPID_Ki)+\
                            '',''+str(myRobot.motorLeftPID_Kd)+\
                            '',''+str(myRobot.motorMowPID_Kp)+'',)

    Send_reqSetting_message('All','w','3',''+str(myRobot.motorMowPID_Ki)+\
                            '',''+str(myRobot.motorMowPID_Kd)+\
                            '',''+str(myRobot.motorBiDirSpeedRatio1)+\
                            '',''+str(myRobot.motorBiDirSpeedRatio2)+\
                            '',''+str(myRobot.motorLeftSwapDir)+\
                            '',''+str(myRobot.motorRightSwapDir)+\
                            '',''+str(myRobot.bumperUse)+\
                            '',''+str(myRobot.sonarUse)+\
                            '',''+str(myRobot.sonarCenterUse)+\
                            '',''+str(myRobot.sonarLeftUse)+'',)
                    
    Send_reqSetting_message('All','w','4',''+str(myRobot.sonarRightUse)+\
                            '',''+str(myRobot.sonarTriggerBelow)+\
                            '',''+str(myRobot.perimeterUse)+\
                            '',''+str(myRobot.perimeter_timedOutIfBelowSmag)+\
                            '',''+str(myRobot.perimeterTriggerTimeout)+\
                            '',''+str(myRobot.perimeterOutRollTimeMax)+\
                            '',''+str(myRobot.perimeterOutRollTimeMin)+\
                            '',''+str(myRobot.perimeterOutRevTime)+\
                            '',''+str(myRobot.perimeterTrackRollTime)+\
                            '',''+str(myRobot.perimeterTrackRevTime)+'',)

 
    Send_reqSetting_message('All','w','5',''+str(myRobot.perimeterPID_Kp)+\
                            '',''+str(myRobot.perimeterPID_Ki)+\
                            '',''+str(myRobot.perimeterPID_Kd)+\
                            '',''+str(myRobot.perimeter_signalCodeNo)+\
                            '',''+str(myRobot.perimeter_swapCoilPolarityLeft)+\
                            '',''+str(myRobot.perimeter_timeOutSecIfNotInside)+\
                            '',''+str(myRobot.trakBlockInnerWheel)+\
                            '',''+str(myRobot.lawnSensorUse)+\
                            '',''+str(myRobot.imuUse)+\
                            '',''+str(myRobot.stopMotorDuringCalib)+'',)

    Send_reqSetting_message('All','w','6',''+str(myRobot.imuDirPID_Kp)+\
                            '',''+str(myRobot.imuDirPID_Ki)+\
                            '',''+str(myRobot.imuDirPID_Kd)+\
                            '',''+str(myRobot.imuRollPID_Kp)+\
                            '',''+str(myRobot.imuRollPID_Ki)+\
                            '',''+str(myRobot.imuRollPID_Kd)+\
                            '',''+str(myRobot.remoteUse)+\
                            '',''+str(myRobot.batMonitor)+\
                            '',''+str(myRobot.batGoHomeIfBelow)+\
                            '',''+str(myRobot.batSwitchOffIfBelow)+'',)

    Send_reqSetting_message('All','w','7',''+str(myRobot.batSwitchOffIfIdle)+\
                            '',''+str(myRobot.batFactor)+\
                            '',''+str(myRobot.batChgFactor)+\
                            '',''+str(myRobot.chgSenseZero)+\
                            '',''+str(myRobot.batSenseFactor)+\
                            '',''+str(myRobot.batFullCurrent)+\
                            '',''+str(myRobot.startChargingIfBelow)+\
                            '',''+str(myRobot.stationRevDist)+\
                            '',''+str(myRobot.stationRollAngle)+\
                            '',''+str(myRobot.stationForwDist)+'',)


    Send_reqSetting_message('All','w','8',''+str(myRobot.stationCheckDist)+\
                            '',''+str(myRobot.odometryUse)+\
                            '',''+str(myRobot.odometryTicksPerRevolution)+\
                            '',''+str(myRobot.odometryTicksPerCm)+\
                            '',''+str(myRobot.odometryWheelBaseCm)+\
                            '',''+str(myRobot.odometryLeftSwapDir)+\
                            '',''+str(myRobot.odometryRightSwapDir)+\
                            '',''+str(myRobot.twoWayOdometrySensorUse)+\
                            '',''+str(myRobot.buttonUse)+\
                            '',''+str(myRobot.userSwitch1)+'',)
    
    Send_reqSetting_message('All','w','9',''+str(myRobot.userSwitch2)+\
                            '',''+str(myRobot.userSwitch3)+\
                            '',''+str(myRobot.timerUse)+\
                            '',''+str(myRobot.rainUse)+\
                            '',''+str(myRobot.gpsUse)+\
                            '',''+str(myRobot.stuckIfGpsSpeedBelow)+\
                            '',''+str(myRobot.gpsSpeedIgnoreTime)+\
                            '',''+str(myRobot.dropUse)+\
                            '',''+str(myRobot.statsOverride)+\
                            '',''+str(myRobot.bluetoothUse)+'',)

                    
    Send_reqSetting_message('All','w','10',''+str(myRobot.esp8266Use)+\
                            '',''+str(myRobot.esp8266ConfigString)+\
                            '',''+str(myRobot.tiltUse)+\
                            '',''+str(myRobot.trackingPerimeterTransitionTimeOut)+\
                            '',''+str(myRobot.motorMowForceOff)+\
                            '',''+str(myRobot.MaxSpeedperiPwm)+\
                            '',''+str(myRobot.RollTimeFor45Deg)+\
                            '',''+str(myRobot.DistPeriObstacleAvoid)+\
                            '',''+str(myRobot.circleTimeForObstacle)+\
                            '',''+str(myRobot.DistPeriOutRev)+'',)

    Send_reqSetting_message('All','w','11',''+str(myRobot.motorRightOffsetFwd)+\
                            '',''+str(myRobot.motorRightOffsetRev)+\
                            '',''+str(myRobot.perimeterMagMaxValue)+\
                            '',''+str(myRobot.SpeedOdoMin)+\
                            '',''+str(myRobot.SpeedOdoMax)+\
                            '',''+str(myRobot.yawSet1)+\
                            '',''+str(myRobot.yawSet2)+\
                            '',''+str(myRobot.yawSet3)+\
                            '',''+str(myRobot.yawOppositeLane1RollRight)+\
                            '',''+str(myRobot.yawOppositeLane2RollRight)+'',)
     

    Send_reqSetting_message('All','w','12',''+str(myRobot.yawOppositeLane3RollRight)+\
                            '',''+str(myRobot.yawOppositeLane1RollLeft)+\
                            '',''+str(myRobot.yawOppositeLane2RollLeft)+\
                            '',''+str(myRobot.yawOppositeLane3RollLeft)+\
                            '',''+str(myRobot.DistBetweenLane)+\
                            '',''+str(myRobot.maxLenghtByLane)+\
                            '',''+str(myRobot.perimeter_swapCoilPolarityRight)+\
                            '',''+str(myRobot.perimeter_read2Coil)+\
                            '',''+str(myRobot.maxDriftPerSecond)+\
                            '',''+str(myRobot.delayBetweenTwoDmpAutocalib)+'',)
    

    Send_reqSetting_message('All','w','13',''+str(myRobot.maxDurationDmpAutocalib)+\
                            '',''+str(myRobot.mowPatternDurationMax)+\
                            '',''+str(myRobot.DistPeriOutStop)+\
                            '',''+str(myRobot.DHT22Use)+\
                            '',''+str(myRobot.RaspberryPIUse)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+'',)
    

     
def read_all_setting():
    Send_reqSetting_message('All','r','0','0','0','0','0','0','0','0','0','0','0')
    
def read_time_setting():
    Send_reqSetting_message('Time','r','0','0','0','0','0','0','0','0','0','0','0')
    
       
def send_req_message(val1,val2,val3,val4,val5,val6,val7):
    message = pynmea2.VAR('RM', 'REQ', (val1,val2,val3,val4,val5,val6,val7))
    message=str(message)
    rpiCheckSum=(message[-2:])
    message=message + '\r' +'\n'
    send_serial_message(message)
   

def send_var_message(val1,val2,val3,val4,val5,val6,val7,val8,val9,val10):
    message = pynmea2.VAR('RM', 'VAR', (val1,val2,val3,val4,val5,val6,val7,val8,val9,val10))
    message=str(message)
    rpiCheckSum=(message[-2:])
    message=message + '\r' +'\n'
    send_serial_message(message)
    
def send_cmd_message(val1,val2,val3,val4,val5):
    message = pynmea2.CMD('RM', 'CMD', (val1,val2,val3,val4,val5))
    message=str(message)
    rpiCheckSum=(message[-2:])
    message=message + '\r' +'\n'
    send_serial_message(message)

    
    
def send_pfo_message(val1,val2,val3,val4,val5,val6,val7):
    message = pynmea2.PFO('RM', 'PFO', (val1,val2,val3,val4,val5,val6,val7))
    message=str(message)
    rpiCheckSum=(message[-2:])
    message=message + '\r' +'\n'
    send_serial_message(message)
    
def Send_reqSetting_message(val1,val2,val3,val4,val5,val6,val7,val8,val9,val10,val11,val12,val13):
    message = pynmea2.SET('RM', 'SET', (val1,val2,val3,val4,val5,val6,val7,val8,val9,val10,val11,val12,val13))
    message=str(message)
    rpiCheckSum=(message[-2:])
    message=message + '\r' +'\n'
    send_serial_message(message)   
               
def send_serial_message(message1):
    
    try:
            checkSerial()
            ser.flushOutput()
            ser.write(bytes(message1,'utf-8'))
            #print("Send Message :" , message1)
            txtSend.insert('1.0',  message1) 
            
    except :
            print("ERREUR while transfert")
            time.sleep(2)



try:
        ser = serial.Serial('/dev/ttyACM0',250000,timeout=0)
        
except:
        print("ERREUR DE CONNECTION")
        time.sleep(2)
        #sys.exit("Impossible de continuer")



""" THE SETTING PAGE ****************************************************"""

TabSetting=Notebook(fen1)

TabSetting.place(x=0,y=0)

#TabConsole=Notebook(fen1)
#img1=tk.PhotoImage(file="/pi/Ardumawer/img/setting1.png")
tabMain=tk.Frame(TabSetting,width=800,height=430)
tabWheelMotor=tk.Frame(TabSetting,width=800,height=430)
tabMowMotor=tk.Frame(TabSetting,width=800,height=430)
tabPerimeter=tk.Frame(TabSetting,width=800,height=430)
tabImu=tk.Frame(TabSetting,width=800,height=430)
tabSonar=tk.Frame(TabSetting,width=800,height=430)
tabBattery=tk.Frame(TabSetting,width=800,height=430)
tabOdometry=tk.Frame(TabSetting,width=800,height=430)
tabDateTime=tk.Frame(TabSetting,width=800,height=430)
tabByLane=tk.Frame(TabSetting,width=800,height=430)



TabSetting.add(tabMain,text="Main")
TabSetting.add(tabWheelMotor,text="Wheels Motor")
TabSetting.add(tabMowMotor,text="Mow Motor")
#TabSetting.add(tabWheelMotor, Image = img1, text = "Manual",Compound = "top")
TabSetting.add(tabPerimeter,text="Perimeter")
TabSetting.add(tabImu,text="Imu")
TabSetting.add(tabSonar,text="Sonar")
TabSetting.add(tabBattery,text="Battery")
TabSetting.add(tabOdometry,text="Odometry")
TabSetting.add(tabDateTime,text="Date Time")
TabSetting.add(tabByLane,text="ByLane")


print (TabSetting.index(TabSetting.select()))


"""************* Main setting *****************************"""

ButtonSaveSettingToFile= tk.Button(tabMain)
ButtonSaveSettingToFile.place(x=30,y=15, height=25, width=200)
ButtonSaveSettingToFile.configure(command = ButtonSaveSettingToFile_click)
ButtonSaveSettingToFile.configure(text="Save Setting To File")

ButtonReadSettingFromFile= tk.Button(tabMain)
ButtonReadSettingFromFile.place(x=30,y=65, height=25, width=200)
ButtonReadSettingFromFile.configure(command = ButtonReadSettingFromFile_click)
ButtonReadSettingFromFile.configure(text="Read Setting From File")

ButtonFlashDue= tk.Button(tabMain)
ButtonFlashDue.place(x=30,y=115, height=40, width=200)
ButtonFlashDue.configure(command = ButtonFlashDue_click)
ButtonFlashDue.configure(text="Update the DUE Firmware")



ChkBtnperimeterUse=tk.Checkbutton(tabMain, text="Use Perimeter",relief=tk.SOLID,variable=MainperimeterUse,anchor='nw')
ChkBtnperimeterUse.place(x=270,y=10,width=250, height=20)
ChkBtnimuUse=tk.Checkbutton(tabMain, text="Use IMU",relief=tk.SOLID,variable=MainimuUse,anchor='nw')
ChkBtnimuUse.place(x=270,y=40,width=250, height=20)
ChkBtngpsUse=tk.Checkbutton(tabMain, text="Use GPS",relief=tk.SOLID,variable=MaingpsUse,anchor='nw')
ChkBtngpsUse.place(x=270,y=70,width=250, height=20)
ChkBtnbluetoothUse=tk.Checkbutton(tabMain, text="Use Bluetooth",relief=tk.SOLID,variable=MainbluetoothUse,anchor='nw')
ChkBtnbluetoothUse.place(x=270,y=100,width=250, height=20)
ChkBtnbumperUse=tk.Checkbutton(tabMain, text="Use Bumper",relief=tk.SOLID,variable=MainbumperUse,anchor='nw')
ChkBtnbumperUse.place(x=270,y=130,width=250, height=20)
ChkBtnsonarUse=tk.Checkbutton(tabMain, text="Use Sonar",relief=tk.SOLID,variable=MainsonarUse,anchor='nw')
ChkBtnsonarUse.place(x=270,y=160,width=250, height=20)
ChkBtnDHT22Use=tk.Checkbutton(tabMain, text="Use DHT22",relief=tk.SOLID,variable=MainDHT22Use,anchor='nw')
ChkBtnDHT22Use.place(x=270,y=190,width=250, height=20)
ChkBtnlawnSensorUse=tk.Checkbutton(tabMain, text="Use Lawn Sensor",relief=tk.SOLID,variable=MainlawnSensorUse,anchor='nw')
ChkBtnlawnSensorUse.place(x=530,y=10,width=250, height=20)
ChkBtntimerUse=tk.Checkbutton(tabMain, text="Use Timer",relief=tk.SOLID,variable=MaintimerUse,anchor='nw')
ChkBtntimerUse.place(x=530,y=40,width=250, height=20)
ChkBtnrainUse=tk.Checkbutton(tabMain, text="Use Rain Sensor",relief=tk.SOLID,variable=MainrainUse,anchor='nw')
ChkBtnrainUse.place(x=530,y=70,width=250, height=20)
ChkBtndropUse=tk.Checkbutton(tabMain, text="Use Drop Sensor",relief=tk.SOLID,variable=MaindropUse,anchor='nw')
ChkBtndropUse.place(x=530,y=100,width=250, height=20)
ChkBtnesp8266Use=tk.Checkbutton(tabMain, text="Use Esp8266",relief=tk.SOLID,variable=Mainesp8266Use,anchor='nw')
ChkBtnesp8266Use.place(x=530,y=130,width=250, height=20)
ChkBtntiltUse=tk.Checkbutton(tabMain, text="Use Tilt Sensor",relief=tk.SOLID,variable=MaintiltUse,anchor='nw')
ChkBtntiltUse.place(x=530,y=160,width=250, height=20)

ButtonRequestMainSettingFomMower = tk.Button(tabMain)
ButtonRequestMainSettingFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestMainSettingFomMower.configure(command = read_all_setting)
ButtonRequestMainSettingFomMower.configure(text="Read All From Mower")

ButtonSetMainApply = tk.Button(tabMain)
ButtonSetMainApply.place(x=300,y=400, height=25, width=100)
ButtonSetMainApply.configure(command = ButtonSetMainApply_click,text="Send To Mower")

ButtonSendSettingToEeprom= tk.Button(tabMain)
ButtonSendSettingToEeprom.place(x=500,y=400, height=25, width=200)
ButtonSendSettingToEeprom.configure(command = ButtonSendSettingToEeprom_click)
ButtonSendSettingToEeprom.configure(text="Save setting into RTC Eeprom")




"""************* Mow motor setting *****************************"""

ChkBtnmotorMowForceOff=tk.Checkbutton(tabMowMotor, text="SAFETY Force Mowing OFF ",relief=tk.SOLID,variable=MowVar1,anchor='nw')
ChkBtnmotorMowForceOff.place(x=10,y=350,width=250, height=20)

slidermotorMowSpeedMaxPwm = tk.Scale(tabMowMotor, from_=0, to=255, label='Max PWM Speed',relief=tk.SOLID,orient='horizontal')
slidermotorMowSpeedMaxPwm.place(x=10,y=10,width=250, height=50)
slidermotorMowRPMSet = tk.Scale(tabMowMotor, from_=0, to=4500, label='Max RPM Speed ',relief=tk.SOLID,orient='horizontal')
slidermotorMowRPMSet.place(x=10,y=60,width=250, height=50)
slidermotorMowPID_Kp = tk.Scale(tabMowMotor, from_=0, to=1, label='Mow RPM Regulation Pid P',relief=tk.SOLID,orient='horizontal')
slidermotorMowPID_Kp.place(x=10,y=110,width=250, height=50)
slidermotorMowPID_Ki = tk.Scale(tabMowMotor, from_=0, to=1, label='Mow RPM Regulation Pid I',relief=tk.SOLID,orient='horizontal')
slidermotorMowPID_Ki.place(x=10,y=160,width=250, height=50)
slidermotorMowPID_Kd = tk.Scale(tabMowMotor, from_=0, to=1, label='Mow RPM Regulation Pid D',relief=tk.SOLID,orient='horizontal')
slidermotorMowPID_Kd.place(x=10,y=210,width=250, height=50)

slidermotorMowSenseScale = tk.Scale(tabMowMotor, from_=0, to=3,resolution=0.1, label='Motor Sense Factor',relief=tk.SOLID,orient='horizontal')
slidermotorMowSenseScale.place(x=270,y=10,width=250, height=50)
slidermotorMowPowerMax = tk.Scale(tabMowMotor, from_=0, to=100, label='Power Max in Watt',relief=tk.SOLID,orient='horizontal')
slidermotorMowPowerMax.place(x=270,y=60,width=250, height=50)

slidermowPatternDurationMax = tk.Scale(tabMowMotor, from_=0, to=360, label='Mow Pattern Max Duration (Minutes)',relief=tk.SOLID,orient='horizontal')
slidermowPatternDurationMax.place(x=270,y=160,width=250, height=50)


ButtonRequestMainSettingFomMower = tk.Button(tabMowMotor,command = read_all_setting,text="Read All From Mower")
ButtonRequestMainSettingFomMower.place(x=10,y=400, height=25, width=150)

ButtonSetMainApply = tk.Button(tabMowMotor,command = ButtonSetMowMotorApply_click,text="Send To Mower")
ButtonSetMainApply.place(x=300,y=400, height=25, width=100)


"""************* Odometry setting *****************************"""

sliderodometryTicksPerRevolution = tk.Scale(tabOdometry, from_=500, to=1500, label='Ticks for one full revolution',relief=tk.SOLID,orient='horizontal')
sliderodometryTicksPerRevolution.place(x=10,y=10,width=250, height=50)
sliderodometryTicksPerCm = tk.Scale(tabOdometry, from_=0, to=30,resolution=0.1, label='Ticks Odo per CM  ',relief=tk.SOLID,orient='horizontal')
sliderodometryTicksPerCm.place(x=10,y=60,width=250, height=50)
sliderodometryWheelBaseCm = tk.Scale(tabOdometry, from_=0, to=50, label='Distance between the 2 Wheels Motor (CM)',relief=tk.SOLID,orient='horizontal')
sliderodometryWheelBaseCm.place(x=10,y=110,width=250, height=50)

ButtonRequestMainSettingFomMower = tk.Button(tabOdometry,command = read_all_setting,text="Read All From Mower")
ButtonRequestMainSettingFomMower.place(x=10,y=400, height=25, width=150)

ButtonSetMainApply = tk.Button(tabOdometry,command = ButtonSetOdometryApply_click,text="Send To Mower")
ButtonSetMainApply.place(x=300,y=400, height=25, width=100)




"""************* Date Time setting *****************************"""


sliderdate_hour = tk.Scale(tabDateTime, from_=0, to=23,variable=tk_date_hour,label='Hour: ',relief=tk.SOLID,orient='horizontal')
sliderdate_hour.place(x=10,y=10,width=250, height=50)

sliderdate_minute = tk.Scale(tabDateTime, from_=0, to=59,variable=tk_date_minute, label='Minute ',relief=tk.SOLID,orient='horizontal')
sliderdate_minute.place(x=10,y=60,width=250, height=50)

sliderdate_dayOfWeek = tk.Scale(tabDateTime, from_=1, to=7, variable=tk_date_dayOfWeek, label='Day of the Week',relief=tk.SOLID,orient='horizontal')
sliderdate_dayOfWeek.place(x=270,y=10,width=250, height=50)

sliderdate_day = tk.Scale(tabDateTime, from_=1, to=31,variable=tk_date_day, label='Day',relief=tk.SOLID,orient='horizontal')
sliderdate_day.place(x=270,y=60,width=250, height=50)

sliderdate_month = tk.Scale(tabDateTime, from_=1, to=12,variable=tk_date_month, label='Month  ',relief=tk.SOLID,orient='horizontal')
sliderdate_month.place(x=270,y=110,width=250, height=50)

sliderdate_year = tk.Scale(tabDateTime, from_=2017, to=2040,variable=tk_date_year, label='Year  ',relief=tk.SOLID,orient='horizontal')
sliderdate_year.place(x=270,y=160,width=250, height=50)

ButtonRequestTimeSettingFomMower = tk.Button(tabDateTime,command = read_time_setting,text="Read Time From Mower")
ButtonRequestTimeSettingFomMower.place(x=10,y=400, height=25, width=150)

ButtonSetMainApply = tk.Button(tabDateTime,command = ButtonSendSettingDateTimeToDue_click,text="Send Time To Mower")
ButtonSetMainApply.place(x=300,y=400, height=25, width=150)





"""************* Battery setting *****************************"""

ChkBtnbatMonitor=tk.Checkbutton(tabBattery, text="Monitor low Battery",relief=tk.SOLID,variable=BatVar1,anchor='nw')
ChkBtnbatMonitor.place(x=10,y=350,width=250, height=20)

sliderbatGoHomeIfBelow = tk.Scale(tabBattery, from_=20, to=30,resolution=0.1, label='Go Home if Voltage Below ',relief=tk.SOLID,orient='horizontal')
sliderbatGoHomeIfBelow.place(x=10,y=10,width=250, height=50)
sliderbatSwitchOffIfBelow = tk.Scale(tabBattery, from_=20, to=30,resolution=0.1, label='All OFF if Voltage Below ',relief=tk.SOLID,orient='horizontal')
sliderbatSwitchOffIfBelow.place(x=10,y=60,width=250, height=50)
sliderbatSwitchOffIfIdle = tk.Scale(tabBattery, from_=1, to=300, label='All OFF After this Delay in Minute',relief=tk.SOLID,orient='horizontal')
sliderbatSwitchOffIfIdle.place(x=10,y=110,width=250, height=50)
sliderstartChargingIfBelow = tk.Scale(tabBattery, from_=20, to=30,resolution=0.1, label='Start Charging if Voltage Below ',relief=tk.SOLID,orient='horizontal')
sliderstartChargingIfBelow.place(x=10,y=160,width=250, height=50)
sliderbatFullCurrent = tk.Scale(tabBattery, from_=0, to=2,resolution=0.1, label='Charge Full if Current Below  ',relief=tk.SOLID,orient='horizontal')
sliderbatFullCurrent.place(x=10,y=210,width=250, height=50)

sliderbatFactor = tk.Scale(tabBattery, from_=9, to=12,resolution=0.1, label='Battery Voltage Factor',relief=tk.SOLID,orient='horizontal')
sliderbatFactor.place(x=270,y=10,width=250, height=50)
sliderbatChgFactor = tk.Scale(tabBattery, from_=9, to=12,resolution=0.1, label='Charger Voltage Factor',relief=tk.SOLID,orient='horizontal')
sliderbatChgFactor.place(x=270,y=60,width=250, height=50)
sliderbatSenseFactor = tk.Scale(tabBattery, from_=0, to=12,resolution=0.1, label='Battery Sense Factor',relief=tk.SOLID,orient='horizontal')
sliderbatSenseFactor.place(x=270,y=110,width=250, height=50)

ButtonRequestMainSettingFomMower = tk.Button(tabBattery,command = read_all_setting,text="Read All From Mower")
ButtonRequestMainSettingFomMower.place(x=10,y=400, height=25, width=150)

ButtonSetMainApply = tk.Button(tabBattery,command = ButtonSetBatteryApply_click,text="Send To Mower")
ButtonSetMainApply.place(x=300,y=400, height=25, width=100)


"""************* SONAR setting *****************************"""

ChkBtnsonarCenterUse=tk.Checkbutton(tabSonar, text="Use Sonar center",relief=tk.SOLID,variable=SonVar1,anchor='nw')
ChkBtnsonarCenterUse.place(x=10,y=10,width=250, height=20)
ChkBtnsonarLeftUse=tk.Checkbutton(tabSonar, text="Use Sonar Left",relief=tk.SOLID,variable=SonVar2,anchor='nw')
ChkBtnsonarLeftUse.place(x=10,y=40,width=250, height=20)
ChkBtnsonarRightUse=tk.Checkbutton(tabSonar, text="Use Sonar Right",relief=tk.SOLID,variable=SonVar3,anchor='nw')
ChkBtnsonarRightUse.place(x=10,y=70,width=250, height=20)

slidersonarTriggerBelow = tk.Scale(tabSonar,orient='horizontal',relief=tk.SOLID, from_=20, to=150, label='Reverse Below in CM (20 to 150)')
slidersonarTriggerBelow.place(x=10,y=130,width=250, height=50)

ButtonRequestMainSettingFomMower = tk.Button(tabSonar)
ButtonRequestMainSettingFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestMainSettingFomMower.configure(command = read_all_setting)
ButtonRequestMainSettingFomMower.configure(text="Read All From Mower")

ButtonSetMainApply = tk.Button(tabSonar)
ButtonSetMainApply.place(x=300,y=400, height=25, width=100)
ButtonSetMainApply.configure(command = ButtonSetSonarApply_click,text="Send To Mower")



"""************* IMU setting *****************************"""
sliderimuDirPID_Kp = tk.Scale(tabImu,orient='horizontal',relief=tk.SOLID, from_=0, to=20,resolution=0.1, label='imuDirPID_Kp (0 to 20)')
sliderimuDirPID_Kp.place(x=5,y=10,width=250, height=50)
sliderimuDirPID_Ki = tk.Scale(tabImu,orient='horizontal',relief=tk.SOLID, from_=0, to=20,resolution=0.1, label='imuDirPID_Ki (0 to 20)')
sliderimuDirPID_Ki.place(x=5,y=60,width=250, height=50)
sliderimuDirPID_Kd = tk.Scale(tabImu,orient='horizontal',relief=tk.SOLID, from_=0, to=20,resolution=0.1, label='imuDirPID_Kd (0 to 20)')
sliderimuDirPID_Kd.place(x=5,y=110,width=250, height=50)
sliderdelayBetweenTwoDmpAutocalib = tk.Scale(tabImu,orient='horizontal',relief=tk.SOLID, from_=60, to=600, label='Delay Between 2 Dmp Autocalib (60 to 600)')
sliderdelayBetweenTwoDmpAutocalib.place(x=5,y=160,width=250, height=50)
slidermaxDurationDmpAutocalib = tk.Scale(tabImu,orient='horizontal',relief=tk.SOLID, from_=10, to=100, label='Calib Duration (10 to 100)')
slidermaxDurationDmpAutocalib.place(x=5,y=210,width=250, height=50)
slidermaxDriftPerSecond = tk.Scale(tabImu,orient='horizontal',relief=tk.SOLID, from_=0, to=20,resolution=0.1, label='Maxi Drift Per Second (0 to 20)')
slidermaxDriftPerSecond.place(x=5,y=260,width=250, height=50)

ChkBtnstopMotorDuringCalib=tk.Checkbutton(tabImu, text="Stop Mow Motor During Calib",relief=tk.SOLID,variable=ImuVar1,anchor='nw')
ChkBtnstopMotorDuringCalib.place(x=5,y=330,width=250, height=20)
ButtonGyroCal= tk.Button(tabImu)
ButtonGyroCal.place(x=330,y=10, height=40, width=200)
ButtonGyroCal.configure(command = ButtonGyroCal_click,text="Start GYRO Calibration")
ButtonCompasCal= tk.Button(tabImu)
ButtonCompasCal.place(x=330,y=60, height=40, width=200)
ButtonCompasCal.configure(command = ButtonCompasCal_click,text="Start COMPAS Calibration")

ButtonRequestMainSettingFomMower = tk.Button(tabImu)
ButtonRequestMainSettingFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestMainSettingFomMower.configure(command = read_all_setting)
ButtonRequestMainSettingFomMower.configure(text="Read All From Mower")

ButtonSetMainApply = tk.Button(tabImu)
ButtonSetMainApply.place(x=300,y=400, height=25, width=100)
ButtonSetMainApply.configure(command = ButtonSetImuApply_click,text="Send To Mower")

"""************* Motor setting *****************************"""

sliderPowerMax = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=100,resolution=0.1, label='Power Max in Watt (0 to 100)')
sliderPowerMax.place(x=5,y=10,width=250, height=50)
sliderSpeedRpmMax = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=100, label='Speed Max in Rpm (0 to 100)')
sliderSpeedRpmMax.place(x=5,y=60,width=250, height=50)
sliderSpeedPwmMax = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=255, label='Speed Max in Pwm (0 to 255)')
sliderSpeedPwmMax.place(x=5,y=110,width=250, height=50)
sliderAccel = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=500, to=2000, label='Starting Accel (500 to 2000)')
sliderAccel.place(x=5,y=160,width=250, height=50)
sliderPowerIgnoreTime = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=8000, label='Power Ignore Time To Start')
sliderPowerIgnoreTime.place(x=5,y=210,width=250, height=50)
sliderRollDegMax = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=360, label='Roll Degrees Max (0 to 360)')
sliderRollDegMax.place(x=5,y=260,width=250, height=50)
sliderRollDegMin = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=180, label='Roll Degrees Min (0 to 180)')
sliderRollDegMin.place(x=5,y=310,width=250, height=50)

sliderRevDist = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=100, label='Reverse Dist from Perimeter in CM')
sliderRevDist.place(x=270,y=10,width=250, height=50)
sliderStopDist = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=30, label='Stop Distance on Perimeter in CM')
sliderStopDist.place(x=270,y=60,width=250, height=50)
sliderPidP = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=3, resolution=0.01,label='Speed PID -- P (0 to 3)')
sliderPidP.place(x=270,y=110,width=250, height=50)
sliderPidI = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=3, resolution=0.01,label='Speed PID -- I (0 to 3)')
sliderPidI.place(x=270,y=160,width=250, height=50)
sliderPidD = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=3, resolution=0.01,label='Speed PID -- D (0 to 3)')
sliderPidD.place(x=270,y=210,width=250, height=50)
sliderRightFwOffset = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=-50, to=50, label='PWM Right Forward Offset in %')
sliderRightFwOffset.place(x=270,y=260,width=250, height=50)
sliderRightRevOffset = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=-50, to=50, label='PWM Right Reverse Offset in %')
sliderRightRevOffset.place(x=270,y=310,width=250, height=50)

sliderSpeedOdoMin = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=70, label='PWM Speed Odo Min (0 to 70)')
sliderSpeedOdoMin.place(x=535,y=10,width=250, height=50)
sliderSpeedOdoMax = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=100, to=255, label='PWM Speed Odo Max (100 to 255)')
sliderSpeedOdoMax.place(x=535,y=60,width=250, height=50)

sliderLeftSense = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=3,resolution=0.01, label='Motor Sense Left Scale (0 to 3)')
sliderLeftSense.place(x=535,y=110,width=250, height=50)
sliderRightSense = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=3,resolution=0.01, label='Motor Sense Right Scale (0 to 3)')
sliderRightSense.place(x=535,y=160,width=250, height=50)



ChkBtnMotorSwapLeftDir=tk.Checkbutton(tabWheelMotor, text="Swap left Direction",relief=tk.SOLID,variable=MotVar1,anchor='nw')
ChkBtnMotorSwapLeftDir.place(x=535,y=310,width=250, height=20)

ChkBtnMotorSwapRightDir=tk.Checkbutton(tabWheelMotor, text="Swap right Direction",relief=tk.SOLID,variable=MotVar2,anchor='nw')
ChkBtnMotorSwapRightDir.place(x=535,y=340,width=250, height=20)


ButtonRequestMotSettingFomMower = tk.Button(tabWheelMotor)
ButtonRequestMotSettingFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestMotSettingFomMower.configure(command = read_all_setting)
ButtonRequestMotSettingFomMower.configure(text="Read From Mower")

ButtonSetMotApply = tk.Button(tabWheelMotor)
ButtonSetMotApply.place(x=300,y=400, height=25, width=100)
ButtonSetMotApply.configure(command = ButtonSetMotApply_click,text="Send To Mower")



"""************* Perimeter setting *****************************"""

sliderTimeBelowSmag = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=500, label='Error if Smag below (0 to 500)')
sliderTimeBelowSmag.place(x=5,y=10,width=250, height=50)
sliderTimeNotInside = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=20, label='Timeout if not inside (0 to 20) in Sec')
sliderTimeNotInside.place(x=5,y=60,width=250, height=50)
sliderTrigTimeout = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=1000, label='Trigger timeout (0 to 1000)')
sliderTrigTimeout.place(x=5,y=110,width=250, height=50)
sliderTrackingSpeed = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=255, label='Tracking PWM Max Speed (0 to 255)')
sliderTrackingSpeed.place(x=5,y=160,width=250, height=50)
sliderCircleArcDistance = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=1, to=250, label='Circle Arc Distance (cm)')
sliderCircleArcDistance.place(x=5,y=210,width=250, height=50)
sliderPeriMagMaxValue = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=500, to=2500, label='Perimeter MAG Max Value (500 to 2500)')
sliderPeriMagMaxValue.place(x=5,y=260,width=250, height=50)
sliderTransitionTimeout = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=5000, label='Transition Timeout (0 to 5000) in msec')
sliderTransitionTimeout.place(x=5,y=310,width=250, height=50)

sliderTrackErrTimeout = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=10000, label='Track error Timeout (0 to 10000)')
sliderTrackErrTimeout.place(x=270,y=10,width=250, height=50)
sliderTrackPid_P = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=50,resolution=0.1, label='Tracking PID value P (0 to 50)')
sliderTrackPid_P.place(x=270,y=60,width=250, height=50)
sliderTrackPid_I = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=50, resolution=0.1,label='Tracking PID value I (0 to 50)')
sliderTrackPid_I.place(x=270,y=110,width=250, height=50)
sliderTrackPid_D = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=50, resolution=0.1,label='Tracking PID value D (0 to 50)')
sliderTrackPid_D.place(x=270,y=160,width=250, height=50)


ChkBtnPeriSwapLeftCoil=tk.Checkbutton(tabPerimeter, text="Swap Left Coil polarity",relief=tk.SOLID,variable=PeriVar1,anchor='nw')
ChkBtnPeriSwapLeftCoil.place(x=535,y=40,width=250, height=20)

ChkBtnPeriSwapRightCoil=tk.Checkbutton(tabPerimeter, text="Swap Right Coil polarity",relief=tk.SOLID,variable=PeriVar2,anchor='nw')
ChkBtnPeriSwapRightCoil.place(x=535,y=70,width=250, height=20)
ChkBtnPeriRead2Coil=tk.Checkbutton(tabPerimeter, text="Read The 2 Coil",relief=tk.SOLID,variable=PeriVar3,anchor='nw')
ChkBtnPeriRead2Coil.place(x=535,y=100,width=250, height=20)
ChkBtnPeriBlockInnWheel=tk.Checkbutton(tabPerimeter, text="Block Inner Wheel",relief=tk.SOLID,variable=PeriVar4,anchor='nw')
ChkBtnPeriBlockInnWheel.place(x=535,y=130,width=250, height=20)


ButtonRequestSettingFomMower = tk.Button(tabPerimeter)
ButtonRequestSettingFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestSettingFomMower.configure(command = read_all_setting)
ButtonRequestSettingFomMower.configure(text="Read From Mower")

ButtonSetPerimeterApply = tk.Button(tabPerimeter)
ButtonSetPerimeterApply.place(x=300,y=400, height=25, width=100)
ButtonSetPerimeterApply.configure(command = ButtonSetPerimeterApply_click,text="Send To Mower")


"""************* Bylane setting *****************************"""

slideryawSet1 = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=20, to=60, label='Lane 1 positive Yaw Heading')
slideryawSet1.place(x=5,y=10,width=250, height=50)
slideryawOppositeLane1RollRight = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=-110, to=-165, label='Lane 1 return Yaw after Roll Right')
slideryawOppositeLane1RollRight.place(x=5,y=60,width=250, height=50)
slideryawOppositeLane1RollLeft = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=-110, to=-165, label='Lane 1 return Yaw after Roll Left')
slideryawOppositeLane1RollLeft.place(x=5,y=110,width=250, height=50)

slideryawSet2 = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=60, to=110, label='Lane 2 positive Yaw Heading')
slideryawSet2.place(x=270,y=10,width=250, height=50)
slideryawOppositeLane2RollRight = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=-60, to=-110, label='Lane 2 return Yaw after Roll Right')
slideryawOppositeLane2RollRight.place(x=270,y=60,width=250, height=50)
slideryawOppositeLane2RollLeft = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=-60, to=-110, label='Lane 2 return Yaw after Roll Left')
slideryawOppositeLane2RollLeft.place(x=270,y=110,width=250, height=50)

slideryawSet3 = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=110, to=165, label='Lane 3 positive Yaw Heading')
slideryawSet3.place(x=535,y=10,width=250, height=50)
slideryawOppositeLane3RollRight = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=-15, to=-80, label='Lane 3 return Yaw after Roll Right')
slideryawOppositeLane3RollRight.place(x=535,y=60,width=250, height=50)
slideryawOppositeLane3RollLeft = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=-15, to=-80, label='Lane 3 return Yaw after Roll Left')
slideryawOppositeLane3RollLeft.place(x=535,y=110,width=250, height=50)


sliderDistBetweenLane = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=10, to=100, label='Distance Between Lane (CM)')
sliderDistBetweenLane.place(x=140,y=200,width=250, height=50)

slidermaxLenghtByLane = tk.Scale(tabByLane,orient='horizontal',relief=tk.SOLID, from_=3, to=50, label='Maximum Drive Distance (M)')
slidermaxLenghtByLane.place(x=410,y=200,width=250, height=50)

def ButtonLaneUseNr_click():
    send_pfo_message('w01','1','2','3','4','5','6',)
ButtonLaneUseNr = tk.Button(tabByLane,command = ButtonLaneUseNr_click,text="Next Lane")
ButtonLaneUseNr.place(x=10,y=200,width=100, height=20)

def ButtonRollDir_click():
    send_pfo_message('w20','1','2','3','4','5','6',) 
ButtonRollDir = tk.Button(tabByLane,command = ButtonRollDir_click,text="Next Roll Dir")
ButtonRollDir.place(x=10,y=230,width=100, height=20)


Frame1= tk.Frame(tabByLane)
Frame1.place(x=10, y=260, height=120, width=650)
Frame1.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
#tk.Label(Frame1,text="Last Roll Dir",fg='green').pack(side='top',anchor='n')
#tk.Label(Frame1,textvariable=tk_batVoltage, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')


tk.Label(Frame1,text="tk_laneInUse",fg='green').place(x=100,y=10,width=100, height=20)
tk.Label(Frame1,textvariable=tk_laneInUse, fg='red',font=("Arial", 20)).place(x=100,y=40,width=100, height=20)
tk.Label(Frame1,text="Last Roll Dir",fg='green').place(x=220,y=10,width=100, height=20)
tk.Label(Frame1,textvariable=tk_rollDir, fg='red',font=("Arial", 20)).place(x=220,y=40,width=100, height=20)
tk.Label(Frame1,text="tk_YawActual",fg='green').place(x=330,y=10,width=100, height=20)
tk.Label(Frame1,textvariable=tk_YawActual, fg='red',font=("Arial", 20)).place(x=330,y=40,width=100, height=20)
tk.Label(Frame1,text="tk_YawCible",fg='green').place(x=440,y=10,width=100, height=20)
tk.Label(Frame1,textvariable=tk_YawCible, fg='red',font=("Arial", 20)).place(x=440,y=40,width=100, height=20)
BtnBylaneStartRec= tk.Button(Frame1,command = BtnBylaneStartRec_click,text="Start")
BtnBylaneStartRec.place(x=10,y=10, height=25, width=60)
BtnBylaneStopRec= tk.Button(Frame1,command = BtnBylaneStopRec_click,text="Stop")
BtnBylaneStopRec.place(x=10,y=40, height=25, width=60)

ButtonRequestSettingFomMower = tk.Button(tabByLane)
ButtonRequestSettingFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestSettingFomMower.configure(command = read_all_setting)
ButtonRequestSettingFomMower.configure(text="Read All From Mower")


ButtonSetByLaneApply = tk.Button(tabByLane)
ButtonSetByLaneApply.place(x=300,y=400, height=25, width=150)
ButtonSetByLaneApply.configure(command = ButtonSendSettingByLaneToDue_click,text="Send By Lane To Mower")










""" THE AUTO PAGE ***************************************************"""
#todo temp humid bat 
imgHome=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/home.png")
imgTrack=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/track.png")
imgStopAll=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/stop all.png")
imgstartMow=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/startmow.png")

AutoPage = tk.Frame(fen1)
AutoPage.place(x=0, y=0, height=430, width=800)

Frame2= tk.Frame(AutoPage)
Frame2.place(x=10, y=180, height=60, width=100)
Frame2.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")

tk.Label(Frame2,text="BATTERY",fg='green').pack(side='top',anchor='n')
tk.Label(Frame2,textvariable=tk_batVoltage, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

Frame3= tk.Frame(AutoPage)
Frame3.place(x=130, y=180, height=60, width=100)
Frame3.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")

tk.Label(Frame3,text="TEMPERATURE",fg='green').pack(side='top',anchor='n')
tk.Label(Frame3,textvariable=tk_Dht22Temp, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

Frame1 = tk.Frame(AutoPage)
Frame1.place(x=10, y=20, height=150, width=130)
tk.Label(Frame1,text="MOW PATTERN",fg='green').pack(side='top',anchor='w')
RdBtn_Random=tk.Radiobutton(Frame1, text="Random", variable=tk_mowingPattern, value=0).pack(side='top',anchor='w')
RdBtn_ByLane=tk.Radiobutton(Frame1, text="By Lane", variable=tk_mowingPattern, value=1).pack(side='top',anchor='w')
RdBtn_Perimeter=tk.Radiobutton(Frame1, text="Perimeter", variable=tk_mowingPattern, value=2).pack(side='top',anchor='w')
ButtonStartMow = tk.Button(AutoPage, image=imgstartMow, command = ButtonStartMow_click)
ButtonStartMow.place(x=130,y=0,width=100, height=130)
Buttonhome = tk.Button(AutoPage, image=imgHome, command = button_home_click)
Buttonhome.place(x=250,y=0,width=100, height=130)
Buttontrack = tk.Button(AutoPage, image=imgTrack, command = button_track_click)
Buttontrack.place(x=380,y=0,width=100, height=130)
ButtonStopAllAuto = tk.Button(AutoPage, image=imgStopAll, command = button_stop_all_click)
ButtonStopAllAuto.place(x=500,y=0,width=100, height=130)


"""**************************THE MANUAL PAGE  **********************************************"""
ManualPage = tk.Frame(fen1)
ManualPage.place(x=0, y=0, height=430, width=800)
Frame1 = tk.Frame(ManualPage)
Frame1.place(x=0, y=0, height=300, width=300)

imgBladeStop = tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/bladeoff.png")
imgBladeStart = tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/bladeon.png")
imgForward=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/forward.png")
imgReverse=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/reverse.png")
imgLeft=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/left.png")
imgRight=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/right.png")
imgStop=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/stop.png")



ButtonForward = tk.Button(Frame1,image=imgForward, command = ButtonForward_click)
ButtonForward.place(x=100, y=0, height=100, width=100)

ButtonStop = tk.Button(Frame1,image=imgStop, command = ButtonStop_click)
ButtonStop.place(x=100, y=100, height=100, width=100)

ButtonRight = tk.Button(Frame1,image=imgRight, command = ButtonRight_click)
ButtonRight.place(x=200, y=100, height=100, width=100)

ButtonLeft = tk.Button(Frame1,image=imgLeft, command = ButtonLeft_click)
ButtonLeft.place(x=0, y=100, height=100, width=100)

ButtonReverse = tk.Button(Frame1,image=imgReverse,command = ButtonReverse_click)
ButtonReverse.place(x=100,y=200, height=100, width=100)

ButtonBladeStart = tk.Button(ManualPage, image=imgBladeStart, command = buttonBlade_start_click)
ButtonBladeStart.place(x=500,y=80,width=100, height=50)
ButtonBladeStop = tk.Button(ManualPage, image=imgBladeStop, command = buttonBlade_stop_click)
ButtonBladeStop.place(x=500,y=130,width=100, height=80)



ButtonStopAllManual = tk.Button(ManualPage, image=imgStopAll, command = button_stop_all_click)
ButtonStopAllManual.place(x=500,y=250,width=100, height=130)


manualSpeedSlider = tk.Scale(ManualPage,orient='horizontal', from_=0, to=255)
manualSpeedSlider.place(x=0,y=300,width=300, height=80)
tk.Label(ManualPage, text='SPEED',fg='green').place(x=0,y=300)

"""
slider1 = tk.Scale(orient='horizontal', from_=0, to=350)
slider1.place(x=50,y=50,anchor='nw',width=300, height=50)
slider2 = tk.Scale(orient='horizontal', from_=0, to=100)
slider2.place(x=50,y=100,anchor='nw',width=300, height=50)
"""

""" The Console page  ************************************"""

ConsolePage = tk.Frame(fen1)
ConsolePage.place(x=0, y=0, height=430, width=800)
Frame2 = tk.Frame(ConsolePage)
Frame2.place(x=100, y=0, height=80, width=380)
Frame2.configure(relief=tk.GROOVE)
Frame2.configure(borderwidth="3")
Frame2.configure(relief=tk.GROOVE)
Frame2.configure(background="#d9d9d9")
Frame2.configure(highlightbackground="#d9d9d9")
Frame2.configure(highlightcolor="black")

txtRecu = tk.Text(ConsolePage)
ScrollTxtRecu = tk.Scrollbar(txtRecu)
ScrollTxtRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtRecu.config(command=txtRecu.yview)
txtRecu.config(yscrollcommand=ScrollTxtRecu.set)
txtRecu.place(x=0,y=350,anchor='nw',width=400, height=100)

txtSend = tk.Text(ConsolePage)
ScrollTxtSend = tk.Scrollbar(txtSend)
ScrollTxtSend.pack(side=tk.RIGHT, fill=tk.Y)
txtSend.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtSend.config(command=txtSend.yview)
txtSend.config(yscrollcommand=ScrollTxtSend.set)
txtSend.place(x=420,y=350,anchor='nw',width=380, height=100)


txtConsoleRecu = tk.Text(ConsolePage)
ScrolltxtConsoleRecu = tk.Scrollbar(txtConsoleRecu)
ScrolltxtConsoleRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtConsoleRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrolltxtConsoleRecu.config(command=txtConsoleRecu.yview)
txtConsoleRecu.config(yscrollcommand=ScrolltxtConsoleRecu.set)
txtConsoleRecu.place(x=0,y=100,anchor='nw',width=800, height=250)

text1 = tk.Label(Frame2, text='Status bar', fg='red')
text1.pack()
Status=tk.StringVar()
tk.Label(Frame2,textvariable=(Status)).pack()
Status1=tk.StringVar()
tk.Label(Frame2,textvariable=(Status1)).pack()

ButtonSaveReceived = tk.Button(ConsolePage)
ButtonSaveReceived .place(x=600,y=5, height=25, width=130)
ButtonSaveReceived .configure(command = ButtonSaveReceived_click,text="Save To File")



""" THE PLOT PAGE ***************************************************"""

TabPlot=Notebook(fen1)
tabPlotMain=tk.Frame(TabPlot,width=800,height=430)
tabPlotWheelMotor=tk.Frame(TabPlot,width=800,height=200)
tabPlotMowMotor=tk.Frame(TabPlot,width=800,height=200)
tabPlotPerimeter=tk.Frame(TabPlot,width=800,height=200)
tabPlotBattery=tk.Frame(TabPlot,width=800,height=200)

TabPlot.add(tabPlotMain,text="Main")
TabPlot.add(tabPlotWheelMotor,text="Wheels Motor")
TabPlot.add(tabPlotMowMotor,text="Mow Motor")
TabPlot.add(tabPlotPerimeter,text="Perimeter")
TabPlot.add(tabPlotBattery,text="Battery")

TabPlot.place(x=0, y=0, height=430, width=800)

#'Main
Frame11= tk.Frame(tabPlotMain,relief=tk.GROOVE,borderwidth="3")
Frame11.place(x=10, y=20, height=80, width=680)

BtnMotPlotStopAll= tk.Button(Frame11,command = BtnMotPlotStopAll_click,text="Stop ALL the Data send by DUE")
BtnMotPlotStopAll.place(x=10,y=10, height=25, width=250)

#'Wheel Motor
tk.Label(tabPlotWheelMotor, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotWheelMotor, textvariable=tk_millis).place(x=400,y=0)
Frame12= tk.Frame(tabPlotWheelMotor,relief=tk.GROOVE,borderwidth="3")
Frame12.place(x=10, y=20, height=80, width=680)
BtnMotPlotStartRec= tk.Button(Frame12,command = BtnMotPlotStartRec_click,text="Start")
BtnMotPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnMotPlotStopRec= tk.Button(Frame12,command = BtnMotPlotStopRec_click,text="Stop")
BtnMotPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainWheelRefresh = tk.Scale(Frame12, from_=1, to=10, label='Refresh Rate per second',relief=tk.SOLID,orient='horizontal')
SldMainWheelRefresh.place(x=70,y=0,width=250, height=50)


tk.Label(Frame12, text='Sence',fg='green').place(x=400,y=0)
tk.Label(Frame12, text='Left',fg='green').place(x=350,y=15)
tk.Label(Frame12, textvariable=tk_motorLeftSenseCurrent).place(x=400,y=15)
tk.Label(Frame12, text='Right',fg='green').place(x=350,y=35)
tk.Label(Frame12, textvariable=tk_motorRightSenseCurrent).place(x=400,y=35)
tk.Label(Frame12, text='PWM',fg='green').place(x=550,y=0)
tk.Label(Frame12, textvariable=tk_motorLeftPWMCurr).place(x=550,y=15)
tk.Label(Frame12, textvariable=tk_motorRightPWMCurr).place(x=550,y=35)

#'Mow Motor
tk.Label(tabPlotMowMotor, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotMowMotor, textvariable=tk_millis).place(x=400,y=0)
Frame13= tk.Frame(tabPlotMowMotor,relief=tk.GROOVE,borderwidth="3")
Frame13.place(x=10, y=20, height=80, width=680)
BtnMowPlotStartRec= tk.Button(Frame13,command = BtnMowPlotStartRec_click,text="Start")
BtnMowPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnMowPlotStopRec= tk.Button(Frame13,command = BtnMowPlotStopRec_click,text="Stop")
BtnMowPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainMowRefresh = tk.Scale(Frame13, from_=1, to=10, label='Refresh Rate per second',relief=tk.SOLID,orient='horizontal')
SldMainMowRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame13, text='Sense',fg='green').place(x=400,y=0)
tk.Label(Frame13, textvariable=tk_motorMowSense).place(x=400,y=15)
tk.Label(Frame13, text='PWM',fg='green').place(x=550,y=0)
tk.Label(Frame13, textvariable=tk_motorMowPWMCurr).place(x=550,y=15)

#'Battery'
tk.Label(tabPlotBattery, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotBattery, textvariable=tk_millis).place(x=400,y=0)
Frame14= tk.Frame(tabPlotBattery,relief=tk.GROOVE,borderwidth="3")
Frame14.place(x=10, y=20, height=80, width=680)
BtnBatPlotStartRec= tk.Button(Frame14,command = BtnBatPlotStartRec_click,text="Start")
BtnBatPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnBatPlotStopRec= tk.Button(Frame14,command = BtnBatPlotStopRec_click,text="Stop")
BtnBatPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainBatRefresh = tk.Scale(Frame14, from_=1, to=100, label='Refresh Rate per minute',relief=tk.SOLID,orient='horizontal')
SldMainBatRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame14, text='Charge',fg='green').place(x=350,y=15)
tk.Label(Frame14, text='Battery',fg='green').place(x=350,y=35)
tk.Label(Frame14, text='Sense',fg='green').place(x=400,y=0)
tk.Label(Frame14, textvariable=tk_chgSense).place(x=400,y=15)
tk.Label(Frame14, text='Voltage',fg='green').place(x=550,y=0)
tk.Label(Frame14, textvariable=tk_chgVoltage).place(x=550,y=15)
tk.Label(Frame14, textvariable=tk_batVoltage).place(x=550,y=35)


#'Perimeter'
tk.Label(tabPlotPerimeter, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotPerimeter, textvariable=tk_millis).place(x=400,y=0)
Frame15= tk.Frame(tabPlotPerimeter,relief=tk.GROOVE,borderwidth="3")
Frame15.place(x=10, y=20, height=80, width=680)
BtnPeriPlotStartRec= tk.Button(Frame15,command = BtnPeriPlotStartRec_click,text="Start")
BtnPeriPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnPeriPlotStopRec= tk.Button(Frame15,command = BtnPeriPlotStopRec_click,text="Stop")
BtnPeriPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainPeriRefresh = tk.Scale(Frame15, from_=1, to=10, label='Refresh Rate per second',relief=tk.SOLID,orient='horizontal')
SldMainPeriRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame15, text='Mag',fg='green').place(x=400,y=0)
tk.Label(Frame15, text='Left',fg='green').place(x=350,y=15)
tk.Label(Frame15, textvariable=tk_perimeterMag).place(x=400,y=15)
tk.Label(Frame15, text='Right',fg='green').place(x=350,y=35)
tk.Label(Frame15, textvariable=tk_perimeterMagRight).place(x=400,y=35)



"""
 THE INFO PAGE ***************************************************
"""
InfoPage = tk.Frame(fen1)
InfoPage.place(x=0, y=0, height=430, width=800)
Infoline1=tk.StringVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline1)
LabInfoline.place(x=10,y=10, height=25, width=300)
Infoline2=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline2)
LabInfoline.place(x=10,y=40, height=25, width=300)
Infoline3=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline3)
LabInfoline.place(x=10,y=70, height=25, width=300)
Infoline4=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline4)
LabInfoline.place(x=10,y=100, height=25, width=300)
Infoline5=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline5)
LabInfoline.place(x=10,y=130, height=25, width=300)

#myWindows.loadPageInfo()


""" THE CAMERA PAGE ***************************************************"""
def BtnStreamVideoStart_click():
    if CamVar1.get()==1:
        myStreamVideo.start(1)
        #webbrowser.open("http://localhost:8000/index.html")

    else:
        myStreamVideo.start(0)
        
def BtnStreamVideoStop_click():
    myStreamVideo.stop()
        

      


StreamVideoPage =tk.Frame(fen1)
StreamVideoPage.place(x=0, y=0, height=430, width=800)
FrameStreamVideo = tk.Frame(StreamVideoPage,borderwidth="1",relief=tk.SOLID)
FrameStreamVideo.place(x=20, y=30, width=600, height=300)
OptBtnStreamVideo1=tk.Radiobutton(FrameStreamVideo, text="320*240",relief=tk.SOLID,variable=CamVar1,value=0,anchor='nw').place(x=10,y=10,width=250, height=20)
OptBtnStreamVideo2=tk.Radiobutton(FrameStreamVideo, text="640*480",relief=tk.SOLID,variable=CamVar1,value=1,anchor='nw').place(x=10,y=30,width=250, height=20)
tk.Label(FrameStreamVideo, text='To view the vidéo stream use a browser http://(Your PI IP Adress and):8000/index.html').place(x=10,y=180)
BtnStreamVideoStart= tk.Button(FrameStreamVideo,command = BtnStreamVideoStart_click,text="Start")
BtnStreamVideoStart.place(x=180,y=250, height=25, width=60)
BtnStreamVideoStop= tk.Button(FrameStreamVideo,command = BtnStreamVideoStop_click,text="Stop")
BtnStreamVideoStop.place(x=10,y=250, height=25, width=60)


        
        
""" THE TEST PAGE ***************************************************"""

TestPage =tk.Frame(fen1)
TestPage.place(x=0, y=0, height=430, width=800)

ButtonOdo1TurnFw = tk.Button(TestPage)
ButtonOdo1TurnFw.place(x=30,y=15, height=25, width=200)
ButtonOdo1TurnFw.configure(command = ButtonOdo1TurnFw_click)
ButtonOdo1TurnFw.configure(text="Forward 1 Turn")

ButtonOdo5TurnFw= tk.Button(TestPage)
ButtonOdo5TurnFw.place(x=30,y=65, height=25, width=200)
ButtonOdo5TurnFw.configure(command = ButtonOdo5TurnFw_click)
ButtonOdo5TurnFw.configure(text="Forward 5 Turn")

ButtonOdo1TurnRev = tk.Button(TestPage)
ButtonOdo1TurnRev.place(x=30,y=115, height=25, width=200)
ButtonOdo1TurnRev.configure(command = ButtonOdo1TurnRev_click)
ButtonOdo1TurnRev.configure(text="Reverse 1 Turn")

ButtonOdo5TurnRev= tk.Button(TestPage)
ButtonOdo5TurnRev.place(x=30,y=165, height=25, width=200)
ButtonOdo5TurnRev.configure(command = ButtonOdo5TurnRev_click)
ButtonOdo5TurnRev.configure(text="Reverse 5 Turn")

ButtonOdo3MlFw = tk.Button(TestPage)
ButtonOdo3MlFw.place(x=300,y=15, height=25, width=200)
ButtonOdo3MlFw.configure(command = ButtonOdo3MlFw_click)
ButtonOdo3MlFw.configure(text="3 Meters Forward")

ButtonOdoRot180= tk.Button(TestPage)
ButtonOdoRot180.place(x=300,y=65, height=25, width=200)
ButtonOdoRot180.configure(command = ButtonOdoRot180_click)
ButtonOdoRot180.configure(text="Rotate 180 Degree")

ButtonOdoRot360= tk.Button(TestPage)
ButtonOdoRot360.place(x=300,y=115, height=25, width=200)
ButtonOdoRot360.configure(command = ButtonOdoRot360_click)
ButtonOdoRot360.configure(text="Rotate 360 Degree")

ButtonOdoRotNonStop= tk.Button(TestPage)
ButtonOdoRotNonStop.place(x=300,y=165, height=25, width=200)
ButtonOdoRotNonStop.configure(command = ButtonOdoRotNonStop_click)
ButtonOdoRotNonStop.configure(text="Rotate Non Stop 100 Turns")

"""
  serialPort->print(F("|yt4~3 meter Forward")); //to verify and adjust the TicksPerCM
  serialPort->print(F("|yt6~Rotate 180Deg"));  //to verify and adjust the odometryWheelBaseCm
  serialPort->print(F("|yt5~Rotate 360Deg"));  //to verify and adjust the odometryWheelBaseCm
  serialPort->println(F("|yt7~Rotate Non Stop"))
"""



""" THE TIMER PAGE ***************************************************"""

TabTimer=Notebook(fen1)
SheetTimer= [None]*5
for i in range(5):
    SheetTimer[i]=tk.Frame(TabTimer,width=800,height=380)
    TabTimer.add(SheetTimer[i],text="Timer "+str(i))

    
TabTimer.place(x=0, y=0, height=430, width=800)

tk_timerActive = []
tk_timerdaysOfWeek = []
tk_timerStartTimehour = []
tk_timerStopTimehour = []
tk_timerStartTimeMinute = []
tk_timerStopTimeMinute = []

tk_timerStartNrLane = []
tk_timerStartRollDir = []
tk_timerStartMowPattern = []
tk_timerStartLaneMaxlengh = []
tk_timerStartDistance = []
tk_Random= []
tk_ByLane= []
tk_Perimeter= []
dayvalue=[0]*5
    
for i in range(5):
    tk_timerActive.append(tk.IntVar())
    tk_timerdaysOfWeek.append(tk.IntVar())
    tk_timerStartTimehour.append(tk.IntVar())
    tk_timerStopTimehour.append(tk.IntVar())
    tk_timerStartTimeMinute.append(tk.IntVar())
    tk_timerStopTimeMinute.append(tk.IntVar())
    tk_timerStartNrLane.append(tk.IntVar())
    tk_timerStartRollDir.append(tk.IntVar())
    tk_timerStartMowPattern.append(tk.IntVar())
    tk_timerStartLaneMaxlengh.append(tk.IntVar())
    tk_timerStartDistance.append(tk.IntVar())
    


tk_timerDayVar=[[None] * 7 for i in range(5)]

ChkBtnDayGroup = [[None] * 7 for i in range(5)]
ChkBtnEnableGroup=[None]*5
FrameStartGroup=[None]*5
FrameStopGroup=[None]*5
SliderHourStartGroup=[None]*5
SliderMinuteStartGroup=[None]*5
SliderHourStopGroup=[None]*5
SliderMinuteStopGroup=[None]*5
SliderStartNrLaneGroup=[None]*5
SliderStartMowPatternGroup=[None]*5
SliderStartLaneMaxlenghGroup=[None]*5
SliderStartDistanceGroup=[None]*5
FrameRollDir=[None]*5
FrameMowPattern=[None]*5
FrameLaneParameter=[None]*5

RdBtn_Random=[None]*5
RdBtn_ByLane=[None]*5
RdBtn_Perimeter=[None]*5



for i in range(5):
    ChkBtnEnableGroup[i] = tk.Checkbutton(SheetTimer[i],text="Enable this Timer",font=("Arial", 14), fg='red',variable=tk_timerActive[i],anchor = 'w')
    ChkBtnEnableGroup[i].place(x=20, y=0, height=25, width=380)
    
    FrameStartGroup[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    FrameStartGroup[i].place(x=20, y=30, height=115, width=350)
    startText="Mower START at " + str(tk_timerStartTimehour[i].get()) +":" +str(tk_timerStartTimeMinute[i].get())
    tk.Label(FrameStartGroup[i], text=startText,font=("Arial", 12), fg='green').place(x=0,y=10, height=15, width=300)
    SliderHourStartGroup[i] = tk.Scale(FrameStartGroup[i],from_=0, to=23,variable=tk_timerStartTimehour[i],relief=tk.SOLID,orient='horizontal')
    SliderHourStartGroup[i].place(x=80, y=30, height=40, width=265)
    SliderMinuteStartGroup[i] = tk.Scale(FrameStartGroup[i],from_=0, to=59,variable=tk_timerStartTimeMinute[i],relief=tk.SOLID,orient='horizontal')
    SliderMinuteStartGroup[i].place(x=80, y=70, height=40, width=265)
    tk.Label(FrameStartGroup[i], text='Hour :',font=("Arial", 14), fg='green').place(x=10,y=30, height=40, width=70)
    tk.Label(FrameStartGroup[i], text='Minute :',font=("Arial", 14), fg='green').place(x=10,y=70, height=40, width=70)
    

    FrameStopGroup[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    FrameStopGroup[i].place(x=380, y=30, height=115, width=350)
    stopText="Mower STOP at " + str(tk_timerStopTimehour[i].get()) +":" +str(tk_timerStopTimeMinute[i].get())
    tk.Label(FrameStopGroup[i], text=stopText,font=("Arial", 12), fg='green').place(x=0,y=10, height=15, width=300)
    SliderHourStopGroup[i] = tk.Scale(FrameStopGroup[i],from_=0, to=23,variable=tk_timerStopTimehour[i],relief=tk.SOLID,orient='horizontal')
    SliderHourStopGroup[i].place(x=80, y=30, height=40, width=265)
    SliderMinuteStopGroup[i] = tk.Scale(FrameStopGroup[i],from_=0, to=59,variable=tk_timerStopTimeMinute[i],relief=tk.SOLID,orient='horizontal')
    SliderMinuteStopGroup[i].place(x=80, y=70, height=40, width=265)
    tk.Label(FrameStopGroup[i], text='Hour :',font=("Arial", 14), fg='green').place(x=10,y=30, height=40, width=70)
    tk.Label(FrameStopGroup[i], text='Minute :',font=("Arial", 14), fg='green').place(x=10,y=70, height=40, width=70)

    tk.Label(SheetTimer[i],text="Where to start on the Perimeter :",fg='green').place(x=10,y=180, height=20, width=780)
    SliderStartDistanceGroup[i]= tk.Scale(SheetTimer[i],from_=0, to=400,font=("Arial", 8),variable=tk_timerStartDistance[i],relief=tk.SOLID,orient='horizontal')
    SliderStartDistanceGroup[i].place(x=10,y=200, height=35, width=780)


   
    FrameMowPattern[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    FrameMowPattern[i].place(x=410, y=240, height=120, width=180)
    tk.Label(FrameMowPattern[i],text="MOW PATTERN :",fg='green').pack(side='top',anchor='w')
    RdBtn_Random[i]=tk.Radiobutton(FrameMowPattern[i], text="Random", variable=tk_timerStartMowPattern[i], value=0).pack(side='top',anchor='w')
    RdBtn_ByLane[i]=tk.Radiobutton(FrameMowPattern[i], text="By Lane", variable=tk_timerStartMowPattern[i], value=1).pack(side='top',anchor='w')
    RdBtn_Perimeter[i]=tk.Radiobutton(FrameMowPattern[i], text="Perimeter", variable=tk_timerStartMowPattern[i], value=2).pack(side='top',anchor='w')
    
    FrameLaneParameter[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    FrameLaneParameter[i].place(x=20, y=240, height=120, width=380)
    tk.Label(FrameLaneParameter[i],text="Maximum Lane Lenght :",fg='green').place(x=10,y=0, height=20, width=360)
    SliderStartLaneMaxlenghGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=30,font=("Arial", 8),variable=tk_timerStartLaneMaxlengh[i],relief=tk.SOLID,orient='horizontal')
    SliderStartLaneMaxlenghGroup[i].place(x=10,y=20, height=35, width=360)
    
    tk.Label(FrameLaneParameter[i], text='Roll Dir',font=("Arial", 12), fg='green').place(x=10,y=60, height=15, width=80)
    RdBtn_Right=tk.Radiobutton(FrameLaneParameter[i], text="Right",variable=tk_timerStartRollDir[i], value=0).place(x=10,y=75, height=20, width=80)
    RdBtn_Left=tk.Radiobutton(FrameLaneParameter[i], text="Left ",variable=tk_timerStartRollDir[i], value=1).place(x=10,y=95, height=20, width=80)


    tk.Label(FrameLaneParameter[i],text="START Lane",font=("Arial", 12), fg='green').place(x=220,y=60, height=15, width=90)
    SliderStartNrLaneGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=3,variable=tk_timerStartNrLane[i],relief=tk.SOLID,orient='horizontal').place(x=230,y=75, height=40, width=70)





    for j in range(7):
        tk_timerDayVar[i][j]=tk.BooleanVar()
        ChkBtnDayGroup[i][j] = tk.Checkbutton(SheetTimer[i],text=days_list[j],variable=tk_timerDayVar[i][j],relief=tk.GROOVE,borderwidth="1",anchor = 'w')
        ChkBtnDayGroup[i][j].place(x=110*j+10, y=150, height=25, width=120) 


     


def ButtonSendTimerToDue_click():
    
    for i in range(5):
        myRobot.Timeractive[i]=tk_timerActive[i].get()
        myRobot.TimerstartTime_hour[i]=tk_timerStartTimehour[i].get()
        myRobot.TimerstartTime_minute[i]=tk_timerStartTimeMinute[i].get()
        myRobot.TimerstopTime_hour[i]=tk_timerStopTimehour[i].get()
        myRobot.TimerstopTime_minute[i]=tk_timerStopTimeMinute[i].get()
        myRobot.TimerstartDistance[i]=tk_timerStartDistance[i].get()
        myRobot.TimerstartMowPattern[i]=tk_timerStartMowPattern[i].get()
        myRobot.TimerstartNrLane[i]=tk_timerStartNrLane[i].get()
        myRobot.TimerstartRollDir[i]=tk_timerStartRollDir[i].get()
        myRobot.TimerstartLaneMaxlengh[i]=tk_timerStartLaneMaxlengh[i].get()
        
        myRobot.TimerdaysOfWeek[i]=1*int(tk_timerDayVar[i][0].get())+2*int(tk_timerDayVar[i][1].get())+4*int(tk_timerDayVar[i][2].get())+\
                  8*int(tk_timerDayVar[i][3].get())+16*int(tk_timerDayVar[i][4].get())+32*int(tk_timerDayVar[i][5].get())+\
                  64*int(tk_timerDayVar[i][6].get())
        
    
        Send_reqSetting_message('Timer','w',''+str(i)+'',''+str(myRobot.Timeractive[i])+\
                                '',''+str(myRobot.TimerstartTime_hour[i])+\
                                '',''+str(myRobot.TimerstartTime_minute[i])+\
                                '',''+str(myRobot.TimerstopTime_hour[i])+\
                                '',''+str(myRobot.TimerstopTime_minute[i])+\
                                '',''+str(myRobot.TimerstartDistance[i])+\
                                '',''+str(myRobot.TimerstartMowPattern[i])+\
                                '',''+str(myRobot.TimerstartNrLane[i])+\
                                '',''+str(myRobot.TimerstartRollDir[i])+\
                                '',''+str(myRobot.TimerstartLaneMaxlengh[i])+'',)
    
    Send_reqSetting_message('Timer','w','5',''+str(myRobot.TimerdaysOfWeek[0])+\
                                '',''+str(myRobot.TimerdaysOfWeek[1])+\
                                '',''+str(myRobot.TimerdaysOfWeek[2])+\
                                '',''+str(myRobot.TimerdaysOfWeek[3])+\
                                '',''+str(myRobot.TimerdaysOfWeek[4])+\
                                '','0','0','0','0','0',)
    
    
    
def ButtonReadTimerFromDue_click():
    Send_reqSetting_message('Timer','r','0','0','0','0','0','0','0','0','0','0','0')

    
ButtonRequestTimerFomMower = tk.Button(TabTimer)
ButtonRequestTimerFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestTimerFomMower.configure(command = ButtonReadTimerFromDue_click)
ButtonRequestTimerFomMower.configure(text="Read Timer From Mower")


ButtonSetTimerApply = tk.Button(TabTimer)
ButtonSetTimerApply.place(x=300,y=400, height=25, width=150)
ButtonSetTimerApply.configure(command = ButtonSendTimerToDue_click,text="Send Timer To Mower")








 
""" THE MAIN PAGE ***************************************************"""


def ButtonPowerOff_click():
    returnval=messagebox.askyesno('Info',"Are you sure you want to shutdown all the PCB ?")
    if returnval :
        send_pfo_message('rt','1','2','3','4','5','6',)
        
imgArdumower = tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/ardumower.png")
imgManual=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/manual.png")
imgAuto=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/auto.png")
imgTest=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/test.png")
imgConsole=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/console.png")
imgSetting=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/setting.png")
imgPowerOff=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/off.png")
imgPlot=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/plot.png")
imgSchedule=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/schedule.png")
imgCamera=tk.PhotoImage(file="/home/pi/Documents/PiArdumower/icons/camera.png")


MainPage = tk.Frame(fen1)
MainPage.place(x=0, y=0, height=480, width=800)

ButtonAuto = tk.Button(MainPage,image=imgAuto,command = ButtonAuto_click)
ButtonAuto.place(x=10,y=10, height=130, width=100)

ButtonManual = tk.Button(MainPage,image=imgManual)
ButtonManual.place(x=145,y=10, height=130, width=100)
ButtonManual.configure(command = ButtonManual_click)

ButtonSetting = tk.Button(MainPage,image=imgSetting)
ButtonSetting.place(x=280,y=10, height=130, width=100)
ButtonSetting.configure(command = ButtonSetting_click)


ButtonConsole = tk.Button(MainPage,image=imgConsole)
ButtonConsole.place(x=415,y=10, height=130, width=100)
ButtonConsole.configure(command = ButtonConsole_click)


ButtonTest = tk.Button(MainPage,image=imgTest)
ButtonTest.place(x=550,y=10, height=130, width=100)
ButtonTest.configure(command = ButtonTest_click)

ButtonPlot = tk.Button(MainPage, image=imgPlot, command = ButtonPlot_click)
ButtonPlot.place(x=10,y=145,width=100, height=130)

ButtonSchedule = tk.Button(MainPage, image=imgSchedule, command = ButtonSchedule_click)
ButtonSchedule.place(x=145,y=145,width=100, height=130)

ButtonCamera = tk.Button(MainPage, image=imgCamera, command = ButtonCamera_click)
ButtonCamera.place(x=280,y=145,width=100, height=130)

ButtonPowerOff = tk.Button(MainPage, image=imgPowerOff, command = ButtonPowerOff_click)
ButtonPowerOff.place(x=685,y=300,width=100, height=130)

Buttonimgardu=tk.Button(MainPage,image=imgArdumower,command = ButtonInfo_click)
Buttonimgardu.place(x=10,y=280,height=150,width=650)

MainStatusLine=tk.StringVar()

Maintext = tk.Label(MainPage, text='',textvariable=MainStatusLine,font=("Arial", 20), fg='red')
Maintext.place(x=10,y=450, height=25, width=300)



ButtonBack1 = tk.Button(MainPage)
ButtonBack1.place(x=500,y=455, height=20, width=100)
ButtonBack1.configure(command = ButtonBackToMain_click)
ButtonBack1.configure(text="Back To Main")



checkSerial()
read_all_setting()
read_time_setting()

fen1.mainloop()




            
            
        
