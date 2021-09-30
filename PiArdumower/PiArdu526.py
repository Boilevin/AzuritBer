#!/usr/bin/env python3
PiVersion="525"
import traceback
import sys
import serial
import pynmea2
import time
#import numpy as np
import subprocess 
import pickle

import os
from tkinter import ttk

from tkinter import messagebox
from tkinter import filedialog
import tkinter as tk
import math

from gpiozero import CPUTemperature

from config import cwd
from config import myOS
from config import GpsConnectedOnPi
from config import NanoConnectedOnPi
from config import DueConnectedOnPi
from config import GpsIsM6n
from config import AutoRecordBatCharging
from config import useDebugConsole
from config import useMqtt

from config import Mqtt_Broker_IP
from config import Mqtt_Port
from config import Mqtt_IdleFreqency
from config import Mqtt_MowerName
from config import Mqtt_User
from config import Mqtt_Password
from config import Mqtt_ShowDebug

from config import streamVideoOnPower
from config import Sender1AdressIP
from config import Sender2AdressIP
from config import Sender3AdressIP

from config import useVision
from config import visionDetectMinScore

#Show video in tkinter canvas but very slow
if(useVision):
    from PIL import Image,ImageTk

import threading


    
   

if(useMqtt):   
    import paho.mqtt.client as mqtt_client
    KEEP_ALIVE  = 60
    Mqqt_client = mqtt_client.Client( client_id = Mqtt_MowerName)
    Mqqt_client.connected_flag=False # create flag in class
       
    def Mqqt_on_log( Mqqt_client, userdata, level, buf ):
        #consoleInsertText("log: ",buf)
        print( "log: ",str(buf))

    def Mqqt_on_connect( Mqqt_client, userdata, flags, rc ):
        if rc==0:           
            Mqqt_client.connected_flag=True #set flag
            print("MQTT connected "+ '\n')
            consoleInsertText("MQTT connected "+ '\n')
            #initialize all for next loop update MQTT data
            mymower.state=255
            mymower.batVoltage=0
            mymower.lastMqttBatteryValue=0
            mymower.Dht22Temp=0
            
            sendMqtt(Mqtt_MowerName + "/Status",str(myRobot.statusNames[mymower.status]))
            
                
            
            
            
        else:
            Mqqt_client.connected_flag=False
            print("MQTT Bad connection Returned Code:" + str(rc) +'\n')
            consoleInsertText("MQTT Bad connection Returned Code:" + str(rc) +'\n')
            
            
            
    def Mqqt_on_disconnect( Mqqt_client, userdata, rc ):
        Mqqt_client.connected_flag=False
        Mqqt_client.loop_stop()    #Stop loop
        mymower.timeToReconnectMqtt=time.time()+20
        print("MQTT Disconnected Code:" + str(rc) + '\n')
        consoleInsertText("MQTT Disconnected Code:" + str(rc) + '\n')
                    
    def Mqqt_on_publish( Mqqt_client, userdata, result ):       
        if (Mqqt_client.connected_flag) :
            mymower.callback_id=int(result)
            #consoleInsertText("MQTT Callback message  " + str(mymower.callback_id) + '\n')
            print("MQTT Callback message  " + str(mymower.callback_id) + '\n')
        else:
            #consoleInsertText("MQTT Callback error last message id  " + mymower.mqtt_message_id + " return " + str(mymower.callback_id) + '\n')
            print("MQTT Callback error last message id  " + str(mymower.mqtt_message_id) + " return " + str(mymower.callback_id) + '\n')
            mymower.callback_id=0
            #receive a callback from the last message send before disconnect
         
    def Mqqt_on_message( Mqqt_client, userdata, message ):
        consoleInsertText( "Reception message MQTT..." + '\n')
        consoleInsertText( "Topic : %s" % message.topic + " Data  : %s" % message.payload + '\n')
        message_txt=str((message.payload),'utf8')
        responsetable=message_txt.split(";")
        #Here the main option to do from COMMAND topic
        if(str(message.topic)==Mqtt_MowerName + "/COMMAND/VIDEO/" or str(message.topic)==Mqtt_MowerName + "/COMMAND/VIDEO"):
            if(message_txt=="ON"):
                consoleInsertText("Start Video streaming" + '\n')
                myStreamVideo.start(0)
            if(message_txt=='OFF'):
                consoleInsertText("Stop Video streaming" + '\n')
                myStreamVideo.stop()    
                
        if(str(message.topic)==Mqtt_MowerName + "/COMMAND/" or str(message.topic)==Mqtt_MowerName + "/COMMAND"):
            if(str(responsetable[0]) == "HOME"):
                button_home_click()
            if(str(responsetable[0]) == "STOP"):
                button_stop_all_click()
            if(str(responsetable[0]) == "START"):
                buttonStartMow_click()
            if(str(responsetable[0]) == "MOWPATTERN"):
                #Maybe need to stop and restart to mow between change ???
                tk_mowingPattern.set(int(responsetable[1]))
                send_var_message('w','mowPatternCurr',''+str(tk_mowingPattern.get())+'','0','0','0','0','0','0','0')
            if(str(responsetable[0]) == "PAUSE"):
                tempVar=mymower.millis + (3600000*int(responsetable[1]))
                send_var_message('w','nextTimeTimer',''+str(tempVar)+'','0','0','0','0','0','0','0')

            if(str(responsetable[0]) == "STARTTIMER"):
                send_var_message('w','mowPatternCurr',''+str(responsetable[1])+'','laneUseNr',''+str(responsetable[2])+'','rollDir',''+str(responsetable[3])+'','0','0','0')
                send_var_message('w','whereToStart',''+str(responsetable[4])+'','areaToGo',''+str(responsetable[5])+'','actualLenghtByLane',''+str(responsetable[6])+'','0','0','0')
                send_pfo_message('rv','1','2','3','4','5','6',)


    #the on_log and on_publish show debug message in terminal
    if (Mqtt_ShowDebug):
        Mqqt_client.on_log = Mqqt_on_log
        Mqqt_client.on_publish = Mqqt_on_publish
    Mqqt_client.on_message = Mqqt_on_message
    Mqqt_client.on_connect = Mqqt_on_connect
    Mqqt_client.on_disconnect = Mqqt_on_disconnect   
    

    def Mqqt_DisConnection():
        Mqqt_client.loop_stop()    #Stop loop 
        Mqqt_client.disconnect() # disconnect
        
    def Mqqt_Connection():
        consoleInsertText("PING Broker" + '\n')
        testNet = os.system("ping -c 1 -W 2000 " + Mqtt_Broker_IP)
        if (testNet == 0):
            consoleInsertText("Broker OK" + '\n')
            try:
                Mqqt_client.username_pw_set( username=Mqtt_User, password=Mqtt_Password )
                Mqqt_client.connect( host=Mqtt_Broker_IP, port=Mqtt_Port, keepalive=KEEP_ALIVE )
                Mqqt_client.subscribe( Mqtt_MowerName + "/COMMAND/#" )
                #Mqqt_client.loop_start()
                print("MQTT Connecting Please Wait " + '\n')
                consoleInsertText("MQTT Connecting Please Wait " + '\n')
                mymower.callback_id=0
                mymower.mqtt_message_id=0
        
            except:
                Mqqt_client.connected_flag=False
                print("MQTT connection failed" + '\n')
                consoleInsertText("MQTT connection failed" + '\n')
                #Mqqt_client.loop_stop()    #Stop loop
                mymower.callback_id=0
                mymower.mqtt_message_id=0
            
        else:
            
            consoleInsertText("BROKER NOT CONNECTED" + '\n')
            

    def sendMqtt(var_topic,var_payload) :        
        if (Mqqt_client.connected_flag):
            r=Mqqt_client.publish(topic=var_topic,payload=var_payload,qos=0, retain=False)
            #mymower.mqtt_message_id=int(r[1])
            if (Mqtt_ShowDebug):
                consoleInsertText("MQTT send message " + var_topic + " " + var_payload + '\n')   
                         

        else:
            if (Mqtt_ShowDebug):
                consoleInsertText("MQTT not connected" + '\n')
            pass
            #consoleInsertText("MQTT not Connected fail to send " + str(mymower.mqtt_message_id) + " " + var_topic + " " + var_payload + '\n')   

#END ADDon For MQTT
            





if myOS == "Linux":
    from Ps4remote import PS4Controller


"""      bb file     """
from robot import *

sys.path.insert(0, "/home/pi/Documents/PiArdumower") #add to avoid KST plot error on path





def ButtonFlashDue_click():
    
    global DueConnectedOnPi
    DueConnectedOnPi=False
    Due_Serial.close()
    ButtonClearConsole_click()
    ConsolePage.tkraise()
    consoleInsertText("Choose a file and click on Flash buitton" + '\n')
    consoleInsertText("Do NOT FORGET to put JP8 in Alawys on mode" + '\n')
    fen1.update()
    print ("Close serial and start Flash subprocess")
    
    subprocess.call('/home/pi/Documents/PiArdumower/DueFlash.py', stdout = subprocess.PIPE)
    consoleInsertText("Flash Finish" + '\n')
    consoleInsertText("Do NOT FORGET to put JP8 in Auto mode" + '\n')
    fen1.update()

    
    time.sleep(3)
    DueConnectedOnPi=True
    Due_Serial.open()


#################################### CAMERA MANAGEMENT ###############################################
class streamVideo_class(object):
    """class use to start and stop the video stream"""
    def __init__(self):
        self.streamVideo = None

    def start(self,resolution):
        self.stop()
        
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


#################################### GPS MANAGEMENT ###############################################

class gpsRecord_class(object):
    """class use to start and stop the gps Record use only if the GPS is connected to the Raspberry"""
    def __init__(self):
        self.gpsRecord = None

    def start(self):
        self.stop()
        self.gpsRecord=subprocess.Popen(["/home/pi/Documents/PiArdumower/gpsrecord.py","shell=True","stdout=subprocess.PIPE"])
        
    def stop(self):
        if self.gpsRecord:
            self.gpsRecord.kill()
            self.gpsRecord.wait()
            self.gpsRecord = None
            
mygpsRecord=gpsRecord_class()

#################################### KST PLOT MANAGEMENT ###############################################
    
class PlotterKst_class(object):
    """class use to start and stop the kst plotting prog"""
    def __init__(self):
        self.PlotterKst = None

    def start(self,fileNameKst):
        self.stop()
        self.PlotterKst=subprocess.Popen(["kst2",fileNameKst,"show=maximize","shell=True","stdout=subprocess.PIPE"])
        
    def stop(self):
        if self.PlotterKst:
            self.PlotterKst.kill()
            self.PlotterKst.wait()
            self.PlotterKst = None

#################################### RFID MANAGEMENT ###############################################    
def find_rfid_tag():
    #mymower.lastRfidFind
    #mymower.lastRfidFind=0

    search_code=mymower.lastRfidFind
    search_status=myRobot.statusNames[mymower.status]
    
    mymower.newtagToDo="Null"
    for i in range(0,len(rfid_list)):
        #if str("b'"+rfid_list[i][0]+"'")== str(search_code):
        if (str(rfid_list[i][0])== str(search_code)) & (str(rfid_list[i][1])== str(search_status)):
            mymower.newtagToDo=rfid_list[i][2]
            mymower.newtagSpeed=rfid_list[i][3]
            mymower.newtagRotAngle1=rfid_list[i][4]
            mymower.newtagDistance1=rfid_list[i][5]
            mymower.newtagRotAngle2=rfid_list[i][6]
            mymower.newtagDistance2=rfid_list[i][7]
            
    if (mymower.newtagToDo=="Null"):
        consoleInsertText('  RFID Tag : %s' % (search_code)+ '\n')
        consoleInsertText('  Status : %s' % (search_status)+ '\n')
        consoleInsertText('  Are not present in the database \n')
        
        
    else:
        consoleInsertText('RFID Tag find ToDO is: %s' % (mymower.newtagToDo)+ '\n')
        consoleInsertText('Area to Mow ---> '+ str(mymower.areaToGo) + '\n')
        if((mymower.newtagToDo=="RTS")): #return to station from station area
            consoleInsertText('RFID Find faster return' + '\n')
            send_var_message('w','newtagRotAngle1',''+str(mymower.newtagRotAngle1)+'','motorSpeedMaxPwm',''+str(mymower.newtagSpeed)+'','0','0','0','0','0')
            send_pfo_message('rz','1','2','3','4','5','6',)
            
        if((mymower.newtagToDo=="FAST_START")): #find a faster way to start
            consoleInsertText('RFID Find faster start' + '\n')
            send_var_message('w','newtagRotAngle1',''+str(mymower.newtagRotAngle1)+'','motorSpeedMaxPwm',''+str(mymower.newtagSpeed)+'','0','0','0','0','0')
            send_pfo_message('ru','1','2','3','4','5','6',)
            
        if(mymower.newtagToDo=="SPEED"): #find the station for example
            
            consoleInsertText('RFID change tracking speed'+'\n')
            send_var_message('w','newtagDistance1',''+str(mymower.newtagDistance1)+'','0','0','0','0','0','0','0')
            send_var_message('w','ActualSpeedPeriPWM',''+str(mymower.newtagSpeed)+'','0','0','0','0','0','0','0')

        if((mymower.newtagToDo=="AREA2") & (mymower.areaToGo==2)):
            consoleInsertText('Go to area ---> '+ str(mymower.areaToGo) + '\n')
            send_var_message('w','motorSpeedMaxPwm',''+str(mymower.newtagSpeed)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagRotAngle1',''+str(mymower.newtagRotAngle1)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagRotAngle2',''+str(mymower.newtagRotAngle2)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagDistance1',''+str(mymower.newtagDistance1)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagDistance2',''+str(mymower.newtagDistance2)+'','0','0','0','0','0','0','0')
            send_pfo_message('ry','1','2','3','4','5','6',)
            ButtonStopArea1_click()
            ButtonStartArea2_click()
            
        
        if((mymower.newtagToDo=="AREA3") & (mymower.areaToGo==3)):
            consoleInsertText('Go to area ---> '+ str(mymower.areaToGo) + '\n')
            send_var_message('w','motorSpeedMaxPwm',''+str(mymower.newtagSpeed)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagRotAngle1',''+str(mymower.newtagRotAngle1)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagRotAngle2',''+str(mymower.newtagRotAngle2)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagDistance1',''+str(mymower.newtagDistance1)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagDistance2',''+str(mymower.newtagDistance2)+'','0','0','0','0','0','0','0')
            send_pfo_message('ry','1','2','3','4','5','6',)    
            ButtonStopArea1_click()
            ButtonStartArea3_click()
        
        if((mymower.newtagToDo=="AREA1")):
            mymower.areaToGo=1
            consoleInsertText('Return to Station area' + '\n')
            send_var_message('w','motorSpeedMaxPwm',''+str(mymower.newtagSpeed)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagRotAngle1',''+str(mymower.newtagRotAngle1)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagRotAngle2',''+str(mymower.newtagRotAngle2)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagDistance1',''+str(mymower.newtagDistance1)+'','0','0','0','0','0','0','0')
            send_var_message('w','newtagDistance2',''+str(mymower.newtagDistance2)+'','0','0','0','0','0','0','0')
            send_var_message('w','areaToGo','1','0','0','0','0','0','0','0')
            send_pfo_message('ry','1','2','3','4','5','6',)
            #stopsender can freeze the Pi so better to put it after the remote
            if (mymower.areaInMowing==2):
                ButtonStopArea2_click()
                ButtonStartArea1_click()
            if (mymower.areaInMowing==3):
                ButtonStopArea3_click()
                ButtonStartArea1_click()
                
            

#################################### VARIABLE INITIALISATION ###############################################
   




direction_list=['LEFT','RIGHT']
days_list=['SUNDAY','MONDAY','TUESDAY','WEDNESDAY','THURSDAY','FRIDAY','SATURDAY']
page_list=['MAIN','AUTO','MANUAL','SETTING','CONSOLE','TEST','PLOT','TIMER','VIDEO','GPS','RFID']


motPlotterKst = PlotterKst_class()
mowPlotterKst = PlotterKst_class()
periPlotterKst = PlotterKst_class()
batPlotterKst = PlotterKst_class()
ImuPlotterKst = PlotterKst_class()


firstplotMotx=0
firstplotMowx=0
firstplotBatx=0
firstplotPerx=0
firstplotImux=0

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
SonVar4=tk.IntVar()

BatVar1=tk.IntVar()
MowVar1=tk.IntVar()
PlotVar1=tk.IntVar()
CamVar1=tk.IntVar()


tk_date_Now=tk.StringVar()
tk_time_Now=tk.StringVar()
tk_MainStatusLine=tk.StringVar()
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
tk_loopsPerSecond=tk.DoubleVar()
tk_lastRfidFind=tk.DoubleVar()
tk_areaToGo=tk.IntVar()
tk_areaToGo.set(1)


"""variable use into refreh plot"""
tk_millis=tk.IntVar()
tk_motorLeftPower=tk.DoubleVar()
tk_motorRightPower=tk.DoubleVar()
tk_motorLeftPWMCurr=tk.IntVar()
tk_motorRightPWMCurr=tk.IntVar()
tk_motorMowPower=tk.DoubleVar()
tk_motorMowPWMCurr=tk.IntVar()
#tk_batVoltage=tk.IntVar()
tk_chgVoltage=tk.DoubleVar()
tk_chgSense=tk.DoubleVar()
tk_perimeterMag=tk.IntVar()
tk_perimeterMagRight=tk.IntVar()
tk_gyroYaw=tk.DoubleVar()
tk_compassYaw=tk.DoubleVar()


ManualKeyboardUse=tk.IntVar()
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

firstFixFlag=False
firstFixDate=0


fen1.title('ARDUMOWER')
fen1.geometry("800x480+0+0")

class  datetime:
    def __init__(self):       
        datetime.hour = 12
        datetime.minute = 0
        datetime.dayOfWeek = 0
        datetime.day = 1
        datetime.month = 1
        datetime.year = 2017

class mower:
    #char* mowPatternNames[] = {"RAND", "LANE",  "WIRE"};
    def __init__(self):
        mower.millis=0
        mower.status=0
        mower.state=0
        mower.odox=0
        mower.odoy=0
        mower.prevYaw=0
        mower.batVoltage=0.00
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
        #mower.motorLeftSenseCurrent=0
        #mower.motorRightSenseCurrent=0
        mower.motorLeftPower=0
        mower.motorRightPower=0
        mower.motorLeftPWMCurr=0
        mower.motorRightPWMCurr=0
        mower.motorMowPower=0
        mower.motorMowPWMCurr=0
        mower.mowPatternCurr=0
        mower.Dht22Temp=0
        mower.Dht22Humid=0
        mower.rollDir=0
        mower.laneInUse= 0
        mower.YawActual=0
        mower.YawCible=0
        mower.loopsPerSecond=0
        mower.useJoystick=False
        mower.lastRfidFind=0
        mower.newtagToDo='FR0'
        mower.newtagRotAngle1=90
        mower.newtagRotAngle2=90
        mower.newtagSpeed=120
        mower.newtagDistance1=0
        mower.newtagDistance2=0
        mower.laserSensor1=9000
        mower.laserSensor2=9000
        mower.laserSensor3=9000
        mower.laserSensor4=9000
        mower.laserSensor5=9000
        mower.laserSensor6=9000
        mower.rainDetect=False
        mower.areaInMowing=1
        mower.areaToGo=1
        
        mower.sigAreaOff=True
        
        mower.timeToStartAreaSignal=0
        mower.focusOnPage=0
        mower.dueSerialReceived=''
        mower.autoRecordBatChargeOn=False

        mower.mqtt_message_id=0
        mower.callback_id=0
        mower.timeToSendMqttIdle=time.time()+20
        mower.timeToReconnectMqtt=time.time()+40
        mower.oneMinuteTrigger=time.time()+10
        mower.cpuFan=False
        mower.lastMqttBatteryValue=0
        
        mower.visionDetect=False
        mower.visionDetectAt=time.time()-3
        mower.visionRollRight=1
        mower.visionRunning=False
        mower.surfaceDetected=0
        mower.objectDetectedID=0
        mower.userOut2=False
        mower.userOut3=False
        
        mower.visionScore=0
        
        
        
    
mymower=mower()
myRobot=robot()
myDate=datetime()
cpu = CPUTemperature()

if myOS == "Linux":
    myps4 = PS4Controller()

def consoleInsertText(texte):
    
        txtConsoleRecu.insert('1.0',' ' + texte)
        txtConsoleRecu.insert('1.0', time.strftime('%H:%M:%S',time.localtime()))   
        #txtConsoleRecu.insert('1.0',' ' + texte + '\n')
    
    
    

#################################### MAIN LOOP ###############################################

def checkSerial():  #the main loop is that
    
    
    if NanoConnectedOnPi :
        response2=Nano_Serial.readline()
        if str(response2)!="b''":
            response2=str(response2,'utf8')
            if response2[:1] != '$' : #it is console message because the first digit is not $
                consoleInsertText(response2)
                
                
            else :  # here a nmea message

                #message = pynmea2.parse(response2)
                #decode_message(message)
            
                try:
                    message = pynmea2.parse(response2)
                    decode_message(message)
                except :
                    print("PARSE ERROR FROM NANO MESSAGE")
                    consoleInsertText("PARSE ERROR FROM NANO MESSAGE  RECU ????????"+ '\n')
                    consoleInsertText(response2)

    try:
        global DueConnectedOnPi

        if (DueConnectedOnPi) :
        #if (DueConnectedOnPi and Due_Serial.inWaiting() != 0) :
        
            
            mymower.dueSerialReceived=Due_Serial.readline()
            if str(mymower.dueSerialReceived)!="b''":
                mymower.dueSerialReceived = mymower.dueSerialReceived.decode('utf-8', errors='ignore')
                if mymower.dueSerialReceived[:1] != '$' : #it is console message because the first digit is not $
                    if(len(mymower.dueSerialReceived))>2:
                        consoleInsertText(mymower.dueSerialReceived)                  
                else :
                    message = pynmea2.parse(mymower.dueSerialReceived)
                    decode_message(message)

    except Exception:
        ConsolePage.tkraise()
        DueConnectedOnPi=False
        exc_type, exc_value, exc_traceback = sys.exc_info()
        #consoleInsertText("*** print_tb:")
        traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
        traceback.print_exception(exc_type, exc_value, exc_traceback,limit=2, file=sys.stdout)
        consoleInsertText(repr(traceback.print_exc()))
        formatted_lines = traceback.format_exc().splitlines()
        consoleInsertText(formatted_lines[0])
        consoleInsertText(formatted_lines[-1])
        consoleInsertText(repr(traceback.format_exception(exc_type, exc_value,exc_traceback)))
        print ("*** extract_tb:") 
        print (repr(traceback.extract_tb(exc_traceback)))
        print ("*** format_tb:")
        print (repr(traceback.format_tb(exc_traceback)))
        print ("*** tb_lineno:", exc_traceback.tb_lineno)
        print("ERROR PLEASE CHECK TRACEBACK INFO")
        consoleInsertText("ERROR PLEASE CHECK TRACEBACK INFO" + '\n')
        
        if (mymower.visionRunning):
            mymower.visionRunning=False
        if(useMqtt):
            consoleInsertText('Close Mqtt Connection'+ '\n')
            Mqqt_DisConnection() 
        consoleInsertText('Start to save all Console Data'+ '\n')
        ButtonSaveReceived_click()  #save the console txt
        
        
        


    if mymower.useJoystick :
        myps4.listen()
        if myps4.leftClick:
            ButtonLeft_click()
        if myps4.rightClick:
            ButtonRight_click()       
        if myps4.upClick:
            send_var_message('w','motorSpeedMaxPwm',''+str(manualSpeedSlider.get())+'','0','0','0','0','0','0','0')
            send_pfo_message('nf','1','2','3','4','5','6',)
        if myps4.downClick:
            ButtonStop_click()
  
        #self.crossClick=False
        #self.roundClick=False
        #self.squareClick=False
        #self.triangle_click=False

        if myps4.triangleClick:
            pass
            #print("triangleClick:")
        if myps4.squareClick:
            buttonBlade_start_click()
            #print("squareClick:")
        if myps4.crossClick:
            button_stop_all_click()
            #print("crossClick:")
        if myps4.roundClick:
            buttonBlade_stop_click()
            #print("roundClick:")
        
    if useDebugConsole:
        txtRecu.delete('5000.0',tk.END) #keep only  lines
        txtSend.delete('5000.0',tk.END) #keep only  lines
    txtConsoleRecu.delete('2500.0',tk.END) #keep only  lines
   
    
    if (useMqtt):
        start1=time.time()
        Mqqt_client.loop(0.05)
        duration=time.time()-start1
        if duration > 0.06 :
            consoleInsertText("MQTT take more than 60 ms in execu tion" + '\n')
        if (Mqqt_client.connected_flag):            
            if (time.time() > mymower.timeToSendMqttIdle):
                sendMqtt(Mqtt_MowerName + "/Idle",str(mymower.loopsPerSecond))
                mymower.timeToSendMqttIdle=time.time()+Mqtt_IdleFreqency
                
        else:
            
            if (time.time() > mymower.timeToReconnectMqtt):
                consoleInsertText("MQTT not connected retry each 2 minutes" + '\n')
                Mqqt_Connection()
                mymower.timeToReconnectMqtt=time.time()+120
                
    #vision detect something
    if (mymower.visionDetect==True):
        
        search_object=mymower.objectDetectedID
        areaThreshold=100
        scoreThreshold=0
        
        for i in range(0,len(vision_list)):
            
            if (str(vision_list[i][1])== str(search_object)):
                areaThreshold=int(vision_list[i][3])
                scoreThreshold=int(vision_list[i][2])
                #print(vision_list[i])
                break
        if(scoreThreshold==0):
            consoleInsertText("Can't find object ID into setting vision list ID : " + str(search_object) + '\n')
        else:
            consoleInsertText("Detected : " + str(vision_list[i][0]) + '\n')
            consoleInsertText("Detected Score : " + str(int(mymower.visionScore)) + " Treshold setting : " + str(scoreThreshold) + '\n')
            consoleInsertText("Detected Area : " + str(int(mymower.surfaceDetected)) + " Treshold setting : " + str(areaThreshold) + '\n')
        
        #only stop if area and score area are OK    
        if ((int(mymower.surfaceDetected) >= areaThreshold) and (int(mymower.visionScore) >= scoreThreshold)):
            mymower.visionDetectAt = time.time()
            mymower.userOut2=True
            send_pfo_message('re1','1','2','3','4','5','6',)



            
            if(mymower.VisionRollRight == 1):
                send_var_message('w','bumperRight','1','bumperLeft','0','0','0','0','0','0')
            else:
                send_var_message('w','bumperRight','0','bumperLeft','1','0','0','0','0','0')


        else:
            consoleInsertText("Object detected but size or score not enough for thresold" + '\n')
            
                
        mymower.visionDetect=False
        
    if((mymower.userOut2==True) and (time.time()>=mymower.visionDetectAt+2)):
            mymower.userOut2=False
            send_pfo_message('re0','1','2','3','4','5','6',)
    

    if (time.time() > mymower.oneMinuteTrigger):
        #consoleInsertText("CPU Température : " + str(cpu.temperature) + '\n')
        #start or stop the Raspberry fan
        if (cpu.temperature>=60.0):
            mymower.cpuFan=True
            send_pfo_message('rd1','1','2','3','4','5','6',)
        else:
            if(mymower.cpuFan==True):
                mymower.cpuFan=False
                send_pfo_message('rd0','1','2','3','4','5','6',)

        if (mymower.visionRunning==True) and (myRobot.statusNames[mymower.status]=="IN_STATION"):
            consoleInsertText("Station detected vision is not needed" + '\n')
            BtnVisionStop_click()
  
            
            
        mymower.oneMinuteTrigger=time.time()+60
    
    
    fen1.after(10,checkSerial)  # here is the main loop each 10ms
    
   
























    
#################################### END OF MAINLOOP ###############################################




def decode_message(message):  #decode the nmea message
    #KeyboardFocusManager.getCurrentKeyboardFocusManager().getActiveWindow()
            if useDebugConsole:
                txtRecu.insert('1.0', str(message)+ '\n')
            if message.sentence_type =='TOW': #message from towell
                mymower.laserSensor1=message.sensor1
                mymower.laserSensor2=message.sensor2
                mymower.laserSensor3=message.sensor3
                mymower.laserSensor4=message.sensor4
                mymower.laserSensor5=message.sensor5
                mymower.laserSensor6=message.sensor6
                if ((int(mymower.laserSensor1) <= 800) or (int(mymower.laserSensor2) <= 800) or (int(mymower.laserSensor3) <= 800)):
                    
                    consoleInsertText('Towel detect something :'+ '\n')
                    consoleInsertText(str(message))
                    
                 
                    #send_var_message('w','motorSpeedMaxPwm',''+str(int(myRobot.motorSpeedMaxPwm)/2)+'','0','0','0','0','0','0','0')
                
                if (int(message.rain) == 1):
                    mymower.rainDetect=True
                    consoleInsertText('RAIN DETECTED TIME TO GO TO STATION'+ '\n')
                    if(myRobot.statusNames[mymower.status] == "NORMAL_MOWING"):
                        button_home_click()
                    
                else:
                    mymower.rainDetect=False
     
            if message.sentence_type =='CMD': #receive a command from the DUE (need to do something
                if message.actuatorname == 'RestartPi':
                    consoleInsertText('TIME TO RESTART'+ '\n')
                    mymower.focusOnPage=4
                    ConsolePage.tkraise()
                    if (mymower.visionRunning):
                        mymower.visionRunning=False
                    if(useMqtt):
                        consoleInsertText('Close Mqtt Connection'+ '\n')
                        Mqqt_DisConnection()                        
                    consoleInsertText('PI Restart into 5 Seconds'+ '\n')
                    consoleInsertText('Start to save all Console Data'+ '\n')
                    ButtonSaveReceived_click()  #save the console txt
                    time.sleep(5)
                    subprocess.Popen('/home/pi/Documents/PiArdumower/Restart.py')
                    fen1.destroy()
                    time.sleep(1)
                    sys.exit("Restart ordered by Arduino Due")
                if message.actuatorname == 'PowerOffPi':
                    mymower.focusOnPage=4
                    ConsolePage.tkraise()
                    if (mymower.visionRunning):
                        mymower.visionRunning=False
                    if(useMqtt):
                        consoleInsertText('Close Mqtt Connection'+ '\n')
                        Mqqt_DisConnection() 
                    consoleInsertText('Start to save all Console Data'+ '\n')
                    ButtonSaveReceived_click()  #save the console txt
                    consoleInsertText('All Console Data are saved'+ '\n')
                    consoleInsertText('PI start Shutdown'+ '\n')
                    time.sleep(5)
                    subprocess.Popen('/home/pi/Documents/PiArdumower/PowerOff.py')
                    fen1.destroy()
                    time.sleep(1)
                    sys.exit("PowerOFF ordered by Arduino Due")
                    
                    
            if message.sentence_type =='RMC': #it's the GPS data message
                global firstFixFlag
                global firstFixDate
                if message.status == "A": # If the sentence shows that there's a fix, then we can log the line
                        txtGpsRecu.insert('1.0', str(message)+ '\n')
                        decimalLatitude=degrees_to_decimal(message.lat,message.lat_dir)
                        decimalLongitude=degrees_to_decimal(message.lon,message.lon_dir)
                        if firstFixFlag is False: # If we haven't found a fix before, then set the filename prefix with GPS date & time.
                            firstFixDate = message.datestamp + "-" + message.timestamp
                            firstFixFlag = True
                
                        else: # write the data to a simple log file and then the raw data as well:
                            with open(cwd+"/gpsdata/" + firstFixDate +".txt", "a") as myfile:
                                myfile.write(message.datestamp + "," + message.timestamp + "," + str(decimalLatitude) + "," + str(decimalLongitude) +"\n")
                        
                            with open(cwd+"/gpsdata/Recu" + firstFixDate +".ubx", "a") as myfile1:
                                myfile1.write(mymower.dueSerialReceived +"\n")


                 
        

            if message.sentence_type =='BYL': #to refresh the ByLane setting page 
                mymower.millis=int(message.millis)
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
                mymower.millis=int(message.millis)
                mymower.developerActive=message.developerActive
                mymower.version=message.version
                Infoline1.set("DUE Firmware : " + mymower.version + "           Pi version : " + PiVersion)
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
                
                mymower.millis=int(message.millis)
                #time.localtime()
                mymower.motorLeftPower=message.motorLeftPower
                mymower.motorRightPower=message.motorRightPower
                mymower.motorLeftPWMCurr=message.motorLeftPWMCurr
                mymower.motorRightPWMCurr=message.motorRightPWMCurr
                mymower.batVoltage=message.batVoltage
                if firstplotMotx==0:
                    firstplotMotx=int(mymower.millis)
                tk_millis.set(mymower.millis)
                tk_motorLeftPower.set(mymower.motorLeftPower)
                tk_motorRightPower.set(mymower.motorRightPower)
                tk_motorLeftPWMCurr.set(mymower.motorLeftPWMCurr)
                tk_motorRightPWMCurr.set(mymower.motorRightPWMCurr)

                f=open(cwd + "/plot/PlotMot.txt",'a+')
                f.write("{};{};{};{};{}\n".format(int((int(mymower.millis)-firstplotMotx)/100),float(mymower.motorLeftPower) , float(mymower.motorRightPower),float(mymower.motorLeftPWMCurr) , float(mymower.motorRightPWMCurr)))
                f.close()
                
            if message.sentence_type =='MOW': #to refresh the plot page of motor mow
                global firstplotMowx
                
                mymower.millis=int(message.millis)
                mymower.motorMowPower=message.motorMowPower
                mymower.motorMowPWMCurr=message.motorMowPWMCurr
                mymower.batVoltage=message.batVoltage
                
                if firstplotMowx==0:
                    firstplotMowx=int(mymower.millis)
                tk_millis.set(mymower.millis)
                tk_motorMowPower.set(mymower.motorMowPower)
                tk_motorMowPWMCurr.set(mymower.motorMowPWMCurr)
                tk_batVoltage.set(mymower.batVoltage)

                f=open(cwd + "/plot/PlotMow.txt",'a+')
                f.write("{};{};{}\n".format(int((int(mymower.millis)-firstplotMowx)/100),float(mymower.motorMowPower) , float(mymower.motorMowPWMCurr)))
                f.close()
               
                
            if message.sentence_type =='BAT': #to refresh the plot page of battery
                global firstplotBatx
                
                mymower.millis=int(message.millis)
                mymower.chgVoltage=message.chgVoltage
                mymower.chgSense=message.chgSense
                mymower.batVoltage=message.batVoltage
                
                if firstplotBatx==0:
                    firstplotBatx=int(mymower.millis)
                tk_millis.set(mymower.millis)
                tk_chgVoltage.set(mymower.chgVoltage)
                tk_chgSense.set(mymower.chgSense)
                tk_batVoltage.set(mymower.batVoltage)
                
                f=open(cwd + "/plot/PlotBat.txt",'a+')
                f.write("{};{};{};{}\n".format(int((int(mymower.millis)-firstplotBatx)/100),float(mymower.chgVoltage) , float(mymower.chgSense), float(mymower.batVoltage)))
                f.close()


            if message.sentence_type =='PER': #to refresh the plot page of perimeter
                global firstplotPerx
                
                mymower.millis=int(message.millis)
                mymower.perimeterMag=message.perimeterMag
                mymower.perimeterMagRight=message.perimeterMagRight
                mymower.areaInMowing=int(message.areaInMowing)
                                
                if firstplotPerx==0:
                    firstplotPerx=int(mymower.millis)
                
                tk_millis.set(mymower.millis)
                tk_perimeterMag.set(mymower.perimeterMag)
                tk_perimeterMagRight.set(mymower.perimeterMagRight)
                
                f=open(cwd + "/plot/PlotPeri.txt",'a+')
                f.write("{};{};{}\n".format(int((int(mymower.millis)-firstplotPerx)/100),float(mymower.perimeterMag) , float(mymower.perimeterMagRight)))
                f.close()

            if message.sentence_type =='IMU': #to refresh the plot page of Imu
                global firstplotImux
                mymower.millis=int(message.millis)
                mymower.gyroYaw=message.gyroYaw
                mymower.compassYaw=message.compassYaw
                
                                
                if firstplotImux==0:
                    firstplotImux=int(mymower.millis)
                
                tk_millis.set(mymower.millis)
                tk_gyroYaw.set(mymower.gyroYaw)
                tk_compassYaw.set(mymower.compassYaw)
                
                
                f=open(cwd + "/plot/PlotImu.txt",'a+')
                f.write("{};{};{}\n".format(int((int(mymower.millis)-firstplotPerx)/100),mymower.gyroYaw , mymower.compassYaw))
                f.close()
                
         
            if message.sentence_type =='STU': # message for status info send on change only       
                mymower.status=int(message.status)
               
                if(useMqtt and Mqqt_client.connected_flag):
                   sendMqtt(Mqtt_MowerName + "/Status",str(myRobot.statusNames[mymower.status]))
                         
                if(myRobot.statusNames[mymower.status]=="TRACK_TO_START"):
                    mymower.areaInMowing=int(message.val1)
                    mymower.areaToGo=int(message.val2)
                    tk_areaToGo.set(mymower.areaToGo)
                    
            if message.sentence_type =='RFI': # message for status info send on change only 
                mymower.status=int(message.status)
                mymower.lastRfidFind=message.rfidTagFind
                find_rfid_tag()
                
                    

            if message.sentence_type =='STA': #permanent each 500 ms message for state info
               
                mymower.millis=int(message.millis)
                mymower.odox=message.odox
                mymower.odoy=message.odoy
                mymower.prevYaw=message.prevYaw
                
                if(useMqtt and Mqqt_client.connected_flag):
                    if (mymower.state!=int(message.state)):
                        mymower.state=int(message.state)    
                        sendMqtt(Mqtt_MowerName + "/State",str(myRobot.stateNames[mymower.state]))    
                    if (mymower.batVoltage!=float(message.batVoltage)):
                        mymower.batVoltage=float(message.batVoltage)
                        ecart=mymower.lastMqttBatteryValue-float(message.batVoltage)
                        #only send Mqtt if 0.2 volt dif to avoid send too fast
                        if(abs(ecart)>0.2):
                            sendMqtt(Mqtt_MowerName + "/Battery",message.batVoltage)
                            mymower.lastMqttBatteryValue=float(message.batVoltage)
                    if (mymower.Dht22Temp!=message.Dht22Temp):
                        mymower.Dht22Temp=message.Dht22Temp
                        sendMqtt(Mqtt_MowerName + "/Temp",str(mymower.Dht22Temp))      
                else:
                   mymower.batVoltage=float(message.batVoltage)
                   mymower.Dht22Temp=message.Dht22Temp
                   mymower.state=int(message.state)   
                   
                mymower.yaw=message.yaw
                mymower.pitch=message.pitch
                mymower.roll=message.roll
                   
                               
                mymower.loopsPerSecond=message.loopsPerSecond
                #//bber17
                if AutoRecordBatCharging :
                    if ((mymower.autoRecordBatChargeOn==False) & (myRobot.stateNames[mymower.state]=='CHARG')): #the mower is now on charge
                        consoleInsertText("Start to record the Battery charging" + '\n')
                        SldMainBatRefresh.set(10) #data flow 10 times each minute
                        mymower.autoRecordBatChargeOn=True
                        BtnBatPlotStartRec_click()
                    if ((mymower.autoRecordBatChargeOn==True) & (myRobot.stateNames[mymower.state]=='STAT')): 
                        consoleInsertText("Stop to record the Battery charging" + '\n')
                        BtnBatPlotStopRec_click()
                        mymower.autoRecordBatChargeOn=False
                    
                    
                
                
   
                tk_batVoltage.set(mymower.batVoltage)         
                tk_ImuYaw.set(int(180*float(mymower.yaw)/math.pi))
                tk_ImuPitch.set(int(180*float(mymower.pitch)/math.pi))
                tk_ImuRoll.set(int(180*float(mymower.roll)/math.pi))
                tk_Dht22Temp.set(mymower.Dht22Temp)
                tk_loopsPerSecond.set(mymower.loopsPerSecond)
               
                tk_date_Now.set(time.strftime('%d/%m/%y %H:%M:%S',time.localtime()))
                tk_time_Now.set(time.strftime('%H:%M:%S',time.localtime()))
                #stop sender1 if we are into station
                if ((mymower.sigAreaOff==False) and (myRobot.stateNames[mymower.state]=='STAT')):
                    ButtonStopArea1_click()
                    mymower.sigAreaOff=True
                

                #Start sender 1 when leaving station
                if ((mymower.sigAreaOff) and (myRobot.stateNames[mymower.state]=='STREV')):
                        ButtonStartArea1_click()

                #start the sender of the area we use
                if ((mymower.sigAreaOff) and (myRobot.stateNames[mymower.state]=='WAITSIG2')):
                    if(time.time() >= mower.timeToStartAreaSignal):
                        mower.timeToStartAreaSignal=time.time()+5  #try to communicate with sender each 5 secondes
                        #ButtonWifiOn_click() #reset dns and acces point
                       
                        if (mymower.areaToGo==1):
                            ButtonStartArea1_click()
                        if (mymower.areaToGo==2):
                            ButtonStopArea1_click()
                            ButtonStartArea2_click()
                        if (mymower.areaToGo==3):
                            ButtonStopArea1_click()
                            ButtonStartArea3_click()
                        
                        
                tk_MainStatusLine.set(myRobot.statusNames[mymower.status] + "/" + myRobot.stateNames[mymower.state] )
                


            if message.sentence_type =='RET': #to fill the setting page All or name of the needed page
               
                if message.setting_page =='Time':
                                 
                    myDate.hour = message.val1
                    myDate.minute = message.val2
                    myDate.dayOfWeek = message.val3
                    myDate.day = message.val4
                    myDate.month = message.val5
                    myDate.year = message.val6
                    tk_date_hour.set(myDate.hour)
                    tk_date_minute.set(myDate.minute)
                    tk_date_dayOfWeek.set(myDate.dayOfWeek)
                    tk_date_day.set(myDate.day)
                    tk_date_month.set(myDate.month)
                    tk_date_year.set(myDate.year)
                    myDateFormatted= myDate.day+"/"+myDate.month+"/"+myDate.year+" "+myDate.hour+":"+myDate.minute
                    consoleInsertText("Date Time from PCB1.3       : " + myDateFormatted + '\n')
                    dateNow=time.strftime('%-d/%-m/%Y %-H:%-M',time.localtime())
                    consoleInsertText("Date Time from Raspberry Pi : " + dateNow + '\n')
                    if myDate.year != time.strftime('%Y',time.localtime()):
                        consoleInsertText("PLEASE SET YEAR CORRECTLY"+ '\n')
                    if myDate.month != time.strftime('%-m',time.localtime()):
                        consoleInsertText("PLEASE SET MONTH CORRECTLY"+ '\n')
                    if myDate.day != time.strftime('%-d',time.localtime()):
                        consoleInsertText("PLEASE SET DAY CORRECTLY"+ '\n')
                    if myDate.hour != time.strftime('%-H',time.localtime()):
                        consoleInsertText("PLEASE SET HOUR CORRECTLY"+ '\n')
                    if (abs(int(myDate.minute)-int(time.strftime('%-M',time.localtime()))) >=10):
                        consoleInsertText("PLEASE SET MINUTE CORRECTLY"+ '\n')
                   
            
                
                            
                     
                if message.setting_page =='Timer':
                    myRobot.Timeractive[int(message.pageNr)]=int(message.val1)
                    myRobot.TimerstartTime_hour[int(message.pageNr)]=int(message.val2)
                    myRobot.TimerstartTime_minute[int(message.pageNr)]=int(message.val3)
                    myRobot.TimerstopTime_hour[int(message.pageNr)]=int(message.val4)
                    myRobot.TimerstopTime_minute[int(message.pageNr)]=int(message.val5)
                    myRobot.TimerstartDistance[int(message.pageNr)]=int(message.val6)
                    myRobot.TimerstartMowPattern[int(message.pageNr)]=int(message.val7)
                    myRobot.TimerstartNrLane[int(message.pageNr)]=int(message.val8)
                    myRobot.TimerstartRollDir[int(message.pageNr)]=int(message.val9)
                    myRobot.TimerstartLaneMaxlengh[int(message.pageNr)]=int(message.val10)
                            
                    tk_timerActive[int(message.pageNr)].set(myRobot.Timeractive[int(message.pageNr)])
                    tk_timerStartTimehour[int(message.pageNr)].set(myRobot.TimerstartTime_hour[int(message.pageNr)])
                    tk_timerStartTimeMinute[int(message.pageNr)].set(myRobot.TimerstartTime_minute[int(message.pageNr)])
                    tk_timerStopTimehour[int(message.pageNr)].set(myRobot.TimerstopTime_hour[int(message.pageNr)])
                    tk_timerStopTimeMinute[int(message.pageNr)].set(myRobot.TimerstopTime_minute[int(message.pageNr)])
                    tk_timerStartDistance[int(message.pageNr)].set(myRobot.TimerstartDistance[int(message.pageNr)])
                    tk_timerStartMowPattern[int(message.pageNr)].set(myRobot.TimerstartMowPattern[int(message.pageNr)])
                    tk_timerStartNrLane[int(message.pageNr)].set(myRobot.TimerstartNrLane[int(message.pageNr)])
                    tk_timerStartRollDir[int(message.pageNr)].set(myRobot.TimerstartRollDir[int(message.pageNr)])
                    tk_timerStartLaneMaxlengh[int(message.pageNr)].set(myRobot.TimerstartLaneMaxlengh[int(message.pageNr)])
                           
                            
                             
                                
                                            
                #suite des timers tables
                if message.setting_page =='Timer1':
                    myRobot.TimerstartArea[int(message.pageNr)]=int(message.val1)
                    myRobot.TimerdaysOfWeek[int(message.pageNr)]=int(message.val2)
                    
                    if int(message.pageNr)== 4:#refresh when receive the last page
                        refreshTimerSettingPage()
                            
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
                        myRobot.motorMowSpeedMinPwm=message.val5
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
                        myRobot.perimeterTriggerMinSmag=message.val5
                        myRobot.perimeterOutRollTimeMax=message.val6
                        myRobot.perimeterOutRollTimeMin=message.val7
                        myRobot.perimeterOutRevTime=message.val8
                        myRobot.perimeterTrackRollTime =message.val9
                        myRobot.sonarLikeBumper=message.val10
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
                        myRobot.autoResetActive=message.val6
                        myRobot.CompassUse=message.val7
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
                        myRobot.sonarToFrontDist=message.val6
                        myRobot.UseBumperDock=message.val7
                        myRobot.dockingSpeed=message.val8
                        
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
                #text1.config(text= message.debug)
                consoleInsertText(message.debug)               
                
            if message.sentence_type == 'DEB':
                #text1.config(text= message.debug)
                consoleInsertText(message.debug)
        
        
    

    
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
    myRobot.motorMowSpeedMinPwm=slidermotorMowSpeedMinPwm.get()
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
    myRobot.sonarToFrontDist=slidersonarToFront.get()
    
    myRobot.sonarCenterUse='0'
    if SonVar1.get()==1:
        myRobot.sonarCenterUse='1'
    myRobot.sonarLeftUse='0'
    if SonVar2.get()==1:
        myRobot.sonarLeftUse='1'
    myRobot.sonarRightUse='0'
    if SonVar3.get()==1:
        myRobot.sonarRightUse='1'
    myRobot.sonarLikeBumper='0'
    if SonVar4.get()==1:
        myRobot.sonarLikeBumper='1'
        
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
    """create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotMow.txt",'w')
    f.write("{};{};{}\n".format("Time","motorMowPower","motorMowPWMCurr"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    mowPlotterKst.start('/home/pi/Documents/PiArdumower/plotMow.kst')
    send_req_message('MOW',''+str(SldMainMowRefresh.get())+'','1','10000','0','0','0',) #arduino start sending data
    
def BtnMowPlotStopRec_click():
    global firstplotMowx
    firstplotMowx=0  #initialise the first time plot for next plot
    mowPlotterKst.stop() #close the kst prog
    
    send_req_message('MOW','1','0','0','0','0','0',) #arduino stop sending data
    
    filename=cwd + "/plot/Plotmow" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:  #avoid error if file not exit 
        os.rename(cwd + "/plot/PlotMow.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file ")
        consoleInsertText("Error when rename file PlotMow.txt" + '\n')
        pass

    """recreate an empty txt file to have correct auto legend into the graph 
    f=open(cwd + "/plot/PlotMow.txt",'w')
    f.write("{};{};{}\n".format("Time","motorMowPower","motorMowPWMCurr"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    """

def BtnPeriPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotPeri.txt",'w')
    f.write("{};{};{}\n".format("Time","perimeterMag","perimeterMagRight"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    periPlotterKst.start('/home/pi/Documents/PiArdumower/plotPeri.kst')
    send_req_message('PERI',''+str(SldMainPeriRefresh.get())+'','1','10000','0','0','0',)
    
def BtnPeriPlotStopRec_click():
    global firstplotPerx
    firstplotPerx=0
    periPlotterKst.stop() #close the kst prog
    
    send_req_message('PERI','1','0','0','0','0','0',)
    
    filename=cwd + "/plot/PlotPeri" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotPeri.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotPeri.txt")
        consoleInsertText("Error when rename file /plot/PlotPeri.txt" + '\n')
        pass
    
    
def BtnBatPlotStartRec_click():
    #create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotBat.txt",'w')
    f.write("{};{};{};{}\n".format("Time","chgVoltage","chgSense","batVoltage"))
    f.write("{};{};{};{}\n".format("0","0","0","0"))
    f.close()
    #if(mymower.autoRecordBatChargeOn!=True):#it's not the auto record so need to start KST
    batPlotterKst.start('/home/pi/Documents/PiArdumower/plotBat.kst')
    send_req_message('BAT',''+str(SldMainBatRefresh.get())+'','1','10000','0','0','0',)    

def BtnBatPlotStopRec_click():
    global firstplotBatx
    firstplotBatx=0
    batPlotterKst.stop() #close the kst prog
    
    send_req_message('BAT','1','0','0','0','0','0',)
    
    filename=cwd + "/plot/PlotBattery" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotBat.txt",filename) #keep a copy of the plot and clear the last kst file
        #//bber17
        #if(mymower.autoRecordBatChargeOn==False): #it's not the auto record so send a message
        #messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotBat.txt")
        consoleInsertText("Error when rename file /plot/PlotBat.txt" + '\n')
        
    
    
def BtnImuPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotImu.txt",'w')
    f.write("{};{};{}\n".format("Time","GyroYaw","CompassYaw"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    ImuPlotterKst.start('/home/pi/Documents/PiArdumower/plotImu.kst')
    send_req_message('IMU',''+str(SldMainImuRefresh.get())+'','1','10000','0','0','0',)    

def BtnImuPlotStopRec_click():
    global firstplotImux
    firstplotImux=0
    ImuPlotterKst.stop() #close the kst prog
    
    send_req_message('IMU','1','0','0','0','0','0',)
    
    filename=cwd + "/plot/PlotImu" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotImu.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotImu.txt")
        consoleInsertText("Error when rename file /plot/PlotImu.txt" + '\n')
        pass
    
 

   
def BtnBylaneStartRec_click():
    send_req_message('BYL','3','1','6000','0','0','0',)
    
def BtnBylaneStopRec_click():
    send_req_message('BYL','1','0','0','0','0','0',)
    
def BtnMotPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotMot.txt",'w')
    f.write("{};{};{};{};{}\n".format("Time","motorLeftPower","motorRightPower","motorLeftPWMCurr","motorRightPWMCurr"))
    f.write("{};{};{};{};{}\n".format("0","0","0","0","0"))
    f.close()
    motPlotterKst.start('/home/pi/Documents/PiArdumower/plotMot.kst')
    send_req_message('MOT',''+str(SldMainWheelRefresh.get())+'','1','10000','0','0','0',)
    
def BtnMotPlotStopRec_click():
    global firstplotMotx
    firstplotMotx=0
    motPlotterKst.stop() #close the kst prog
    send_req_message('MOT','1','0','0','0','0','0',)
    filename=cwd + "/plot/PlotMotor" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotMot.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotMot.txt")
        consoleInsertText("Error when rename file /plot/PlotMot.txt" + '\n')
        pass
    

def BtnMotPlotStopAll_click():
    BtnMotPlotStopRec_click()
    BtnBatPlotStopRec_click()
    BtnPeriPlotStopRec_click()
    BtnMowPlotStopRec_click()
    BtnImuPlotStopRec_click()
    

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
    slidermotorMowSpeedMinPwm.set(myRobot.motorMowSpeedMinPwm)    
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
    slidersonarToFront.set(myRobot.sonarToFrontDist)
            
    ChkBtnsonarRightUse.deselect()
    if myRobot.sonarRightUse=='1':
        ChkBtnsonarRightUse.select()
    ChkBtnsonarLeftUse.deselect()
    if myRobot.sonarLeftUse=='1':
        ChkBtnsonarLeftUse.select()
    ChkBtnsonarCenterUse.deselect()
    if myRobot.sonarCenterUse=='1':
        ChkBtnsonarCenterUse.select()
    ChkBtnsonarLikeBumper.deselect()
    if myRobot.sonarLikeBumper=='1':
        ChkBtnsonarLikeBumper.select()

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
        tk_timerStartArea[i].set(myRobot.TimerstartArea[i])
                
        for j in range(7):                 
            result=[bool((myRobot.TimerdaysOfWeek[i]) & (1<<n)) for n in range(8)]
            tk_timerDayVar[i][j].set(result[j])
             
  

  
def refreshMotorSettingPage():
    manualSpeedSlider.set(myRobot.motorSpeedMaxPwm)
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
    sliderTrigMinSmag.set(myRobot.perimeterTriggerMinSmag)
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
    fileName=cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Received.txt" 
    with open(fileName,"w") as f:
        f.write(txtRecu.get('1.0','end'))
    fileName=cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Send.txt"
    with open(fileName,"w") as f:
        f.write(txtSend.get('1.0','end'))
    fileName=cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Console.txt"
    with open(fileName,"w") as f:
        f.write(txtConsoleRecu.get('1.0','end'))

    consoleInsertText('All Console file are saved' + '\n')
    

    



def button_stop_all_click():
    send_pfo_message('ro','1','2','3','4','5','6',)

def ButtonInfo_click():
    send_req_message('INF','2','0','1','0','0','0',)
    mymower.focusOnPage=11
    InfoPage.tkraise()
    
def ButtonCamera_click():
    mymower.focusOnPage=8
    StreamVideoPage.tkraise()
    
def ButtonGps_click():
    mymower.focusOnPage=9
    GpsPage.tkraise()
    
def ButtonRfid_click():
    mymower.focusOnPage=10
    RfidPage.tkraise()
    
def ButtonSchedule_click():
    mymower.focusOnPage=7
    TabTimer.tkraise()
    
    
def ButtonGyroCal_click():
    send_pfo_message('g18','1','2','3','4','5','6',)
    mymower.focusOnPage=4
    ConsolePage.tkraise()

def ButtonCompasCal_click():
    send_pfo_message('g19','1','2','3','4','5','6',)
    mymower.focusOnPage=4
    ConsolePage.tkraise()


    
def ButtonSendSettingToEeprom_click():
    send_pfo_message('sz','1','2','3','4','5','6',)
    mymower.focusOnPage=4
    ConsolePage.tkraise()
    

def ButtonManual_click():
    manualSpeedSlider.set(myRobot.motorSpeedMaxPwm)
    mymower.focusOnPage=2
    ManualPage.tkraise()
        
def ButtonPlot_click():
    mymower.focusOnPage=6
    TabPlot.tkraise()

def ButtonSetting_click():
    mymower.focusOnPage=3
    TabSetting.tkraise()
   
def ButtonBackToMain_click():
    mymower.focusOnPage=0
    MainPage.tkraise()
    
                    
def ButtonConsole_click():
    mymower.focusOnPage=4
    ConsolePage.tkraise()

def ButtonTest_click():
    mymower.focusOnPage=5
    TestPage.tkraise()
    
def ButtonAuto_click():
    mymower.focusOnPage=1
    AutoPage.tkraise()
    
 
    
    
   
    
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
    myRobot.yawSet1=slideryawSet1.get()
    myRobot.yawSet2=slideryawSet2.get()
    myRobot.yawSet3=slideryawSet3.get()
    myRobot.yawOppositeLane1RollRight=slideryawOppositeLane1RollRight.get()
    myRobot.yawOppositeLane2RollRight=slideryawOppositeLane2RollRight.get()
    myRobot.yawOppositeLane3RollRight=slideryawOppositeLane3RollRight.get()
    myRobot.yawOppositeLane1RollLeft=slideryawOppositeLane1RollLeft.get()
    myRobot.yawOppositeLane2RollLeft=slideryawOppositeLane2RollLeft.get()
    myRobot.yawOppositeLane3RollLeft=slideryawOppositeLane3RollLeft.get()
    myRobot.DistBetweenLane=sliderDistBetweenLane.get()
    myRobot.maxLenghtByLane=slidermaxLenghtByLane.get()

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
                            '',''+str(myRobot.motorMowSpeedMinPwm)+\
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
                            '',''+str(myRobot.perimeterTriggerMinSmag)+\
                            '',''+str(myRobot.perimeterOutRollTimeMax)+\
                            '',''+str(myRobot.perimeterOutRollTimeMin)+\
                            '',''+str(myRobot.perimeterOutRevTime)+\
                            '',''+str(myRobot.perimeterTrackRollTime)+\
                            '',''+str(myRobot.sonarLikeBumper)+'',)

 
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
                            '',''+str(myRobot.autoResetActive)+\
                            '',''+str(myRobot.CompassUse)+\
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
                            '',''+str(myRobot.sonarToFrontDist)+\
                            '',''+str(myRobot.UseBumperDock)+\
                            '',''+str(myRobot.dockingSpeed)+\
                            '',''+str(0)+\
                            '',''+str(0)+'',)



    consoleInsertText("All Setting are change into the Due but not save for the moment" + '\n') 
    

     
def read_all_setting():
    Send_reqSetting_message('All','r','0','0','0','0','0','0','0','0','0','0','0')
    
def read_time_setting():
    Send_reqSetting_message('Time','r','0','0','0','0','0','0','0','0','0','0','0')
    
       
def send_req_message(val1,val2,val3,val4,val5,val6,val7):
    message = pynmea2.VAR('RM', 'REQ', (val1,val2,val3,val4,val5,val6,val7))
    message=str(message)
    message=message + '\r' +'\n'
    send_serial_message(message)
   

def send_var_message(val1,val2,val3,val4,val5,val6,val7,val8,val9,val10):
    message = pynmea2.VAR('RM', 'VAR', (val1,val2,val3,val4,val5,val6,val7,val8,val9,val10))
    message=str(message)
    message=message + '\r' +'\n'
    send_serial_message(message)
    
def send_cmd_message(val1,val2,val3,val4,val5):
    message = pynmea2.CMD('RM', 'CMD', (val1,val2,val3,val4,val5))
    message=str(message)
    message=message + '\r' +'\n'
    send_serial_message(message)

    
    
def send_pfo_message(val1,val2,val3,val4,val5,val6,val7):
    message = pynmea2.PFO('RM', 'PFO', (val1,val2,val3,val4,val5,val6,val7))
    message=str(message)
    message=message + '\r' +'\n'
    send_serial_message(message)
    
def Send_reqSetting_message(val1,val2,val3,val4,val5,val6,val7,val8,val9,val10,val11,val12,val13):
    message = pynmea2.SET('RM', 'SET', (val1,val2,val3,val4,val5,val6,val7,val8,val9,val10,val11,val12,val13))
    message=str(message)
    message=message + '\r' +'\n'
    send_serial_message(message)   
               
def send_serial_message(message1):
    
    try:
        if DueConnectedOnPi:
            #checkSerial()
            Due_Serial.flushOutput()
            Due_Serial.write(bytes(message1,'utf-8'))
            #print("Send Message :" , message1)
            if useDebugConsole:
                txtSend.insert('1.0',  message1) 
            
    except :
            print("ERREUR while transfert")
            time.sleep(2)


""" ------------------- connecting to the laser nano tower ------------ """
try:
    if NanoConnectedOnPi :
        
        if myOS == "Linux":
            Nano_Serial = serial.Serial('/dev/ttyUSB0',115200,timeout=0)
        else:
            Nano_Serial = serial.Serial('COM10',115200,timeout=0)
            
        byteResponse=Nano_Serial.readline()
        print(str(byteResponse))

        
        
except:
        print(" ")
        print("******************************")
        print("ERREUR DE CONNECTION WITH NANO")
        print("******************************")
        print(" ")
        
        time.sleep(1)
        #sys.exit("Impossible de continuer")

""" ------------------- connecting the the PCB1.3 arduino due ------------ """
try:
    if DueConnectedOnPi :
        if myOS == "Linux":
            if os.path.exists('/dev/ttyACM0') == True:
                Due_Serial = serial.Serial('/dev/ttyACM0',115200,timeout=10,write_timeout = 10)
                Due_Serial.flushInput()
                Due_Serial.flushOutput()  # clear the output buffer
                print("Find Serial on ttyACM0")

            if os.path.exists('/dev/ttyACM1') == True:
                Due_Serial = serial.Serial('/dev/ttyACM1',115200,timeout=10,write_timeout = 10)
                Due_Serial.flushInput()
                Due_Serial.flushOutput()  # clear the output buffer
                print("Find Serial on ttyACM1")
        else:
            Due_Serial = serial.Serial('COM9',115200,timeout=0)
            
        byteResponse=Due_Serial.readline()
        print(str(byteResponse))

        
        
except:
        print(" ")
        print("************************************")
        print("ERREUR DE CONNECTION WITH PCB1.3 DUE")
        print("************************************")
        print(" ")
        time.sleep(1)
        #sys.exit("Impossible de continuer")


#print(cwd)
#cwd1=cwd+"/icons/home.png"
#print(cwd1)

"""-------------------ICONS LOADING---------------------"""
imgHome=tk.PhotoImage(file=cwd + "/icons/home.png")
imgTrack=tk.PhotoImage(file=cwd + "/icons/track.png")
imgStopAll=tk.PhotoImage(file=cwd + "/icons/stop all.png")
imgstartMow=tk.PhotoImage(file=cwd + "/icons/startmow.png")
imgBack=tk.PhotoImage(file=cwd + "/icons/back.png")
imgBladeStop = tk.PhotoImage(file=cwd + "/icons/bladeoff.png")
imgBladeStart = tk.PhotoImage(file=cwd + "/icons/bladeon.png")
imgForward=tk.PhotoImage(file=cwd + "/icons/forward.png")
imgReverse=tk.PhotoImage(file=cwd + "/icons/reverse.png")
imgLeft=tk.PhotoImage(file=cwd + "/icons/left.png")
imgRight=tk.PhotoImage(file=cwd + "/icons/right.png")
imgStop=tk.PhotoImage(file=cwd + "/icons/stop.png")
imgArdumower = tk.PhotoImage(file=cwd + "/icons/ardumower.png")
imgManual=tk.PhotoImage(file=cwd + "/icons/manual.png")
imgAuto=tk.PhotoImage(file=cwd + "/icons/auto.png")
imgTest=tk.PhotoImage(file=cwd + "/icons/test.png")
imgConsole=tk.PhotoImage(file=cwd + "/icons/console.png")
imgSetting=tk.PhotoImage(file=cwd + "/icons/setting.png")
imgPowerOff=tk.PhotoImage(file=cwd + "/icons/off.png")
imgPlot=tk.PhotoImage(file=cwd + "/icons/plot.png")
imgSchedule=tk.PhotoImage(file=cwd + "/icons/schedule.png")
imgCamera=tk.PhotoImage(file=cwd + "/icons/camera.png")
imgGps=tk.PhotoImage(file=cwd + "/icons/gps.png")
imgJoystickON=tk.PhotoImage(file=cwd + "/icons/joystick_on.png")
imgJoystickOFF=tk.PhotoImage(file=cwd + "/icons/joystick_off.png")
imgJoystick=tk.PhotoImage(file=cwd + "/icons/joystick.png")
imgRfid=tk.PhotoImage(file=cwd + "/icons/rfid.png")






""" THE SETTING PAGE ****************************************************"""

TabSetting=ttk.Notebook(fen1)

TabSetting.place(x=0,y=0)

#TabConsole=ttk.Notebook(fen1)
#img1=tk.PhotoImage(file="/pi/Ardumawer/img/setting1.png")
tabMain=tk.Frame(TabSetting,width=800,height=380)
tabWheelMotor=tk.Frame(TabSetting,width=800,height=380)
tabMowMotor=tk.Frame(TabSetting,width=800,height=380)
tabPerimeter=tk.Frame(TabSetting,width=800,height=380)
tabImu=tk.Frame(TabSetting,width=800,height=380)
tabSonar=tk.Frame(TabSetting,width=800,height=380)
tabBattery=tk.Frame(TabSetting,width=800,height=380)
tabOdometry=tk.Frame(TabSetting,width=800,height=380)
tabDateTime=tk.Frame(TabSetting,width=800,height=380)
tabByLane=tk.Frame(TabSetting,width=800,height=380)
tabVision=tk.Frame(TabSetting,width=800,height=380)




TabSetting.add(tabMain,text="Main")
TabSetting.add(tabWheelMotor,text="Wheels Motor")
TabSetting.add(tabMowMotor,text="Mow Motor")
TabSetting.add(tabPerimeter,text="Perimeter")
TabSetting.add(tabImu,text="Imu")
TabSetting.add(tabSonar,text="Sonar")
TabSetting.add(tabBattery,text="Battery")
TabSetting.add(tabOdometry,text="Odometry")
TabSetting.add(tabDateTime,text="Date Time")
TabSetting.add(tabByLane,text="ByLane")
TabSetting.add(tabVision,text="Vision")



#print (TabSetting.index(TabSetting.select()))



"""************* Main setting *****************************"""

ButtonSaveSettingToFile= tk.Button(tabMain)
ButtonSaveSettingToFile.place(x=30,y=15, height=25, width=200)
ButtonSaveSettingToFile.configure(command = ButtonSaveSettingToFile_click)
ButtonSaveSettingToFile.configure(text="Save Setting To File")

ButtonReadSettingFromFile= tk.Button(tabMain)
ButtonReadSettingFromFile.place(x=30,y=65, height=25, width=200)
ButtonReadSettingFromFile.configure(command = ButtonReadSettingFromFile_click)
ButtonReadSettingFromFile.configure(text="Read Setting From File")

# with PCB1.3 need to set JP8 at always ON before flashing because when due flash the PCB poweroff the PI 
ButtonFlashDue= tk.Button(tabMain)
ButtonFlashDue.place(x=30,y=175, height=40, width=200)
ButtonFlashDue.configure(command = ButtonFlashDue_click)
ButtonFlashDue.configure(text="Update the DUE Firmware")

def ButtonReboot_click():
    print("reboot")
    send_pfo_message('h04','1','2','3','4','5','6',)

    
    
ButtonReboot = tk.Button(tabMain,text="Reboot All",  command = ButtonReboot_click)
ButtonReboot.place(x=30,y=115, height=40, width=200)


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
ButtonRequestMainSettingFomMower.place(x=10,y=350, height=25, width=150)
ButtonRequestMainSettingFomMower.configure(command = read_all_setting)
ButtonRequestMainSettingFomMower.configure(text="Read All From Mower")

ButtonSetMainApply = tk.Button(tabMain)
ButtonSetMainApply.place(x=300,y=350, height=25, width=100)
ButtonSetMainApply.configure(command = ButtonSetMainApply_click,text="Send To Mower")

ButtonSendSettingToEeprom= tk.Button(tabMain)
ButtonSendSettingToEeprom.place(x=30,y=300, height=25, width=200)
ButtonSendSettingToEeprom.configure(command = ButtonSendSettingToEeprom_click)
ButtonSendSettingToEeprom.configure(text="Save setting into RTC Eeprom")


ButtonBackHome = tk.Button(TabSetting, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)



"""************* Mow motor setting *****************************"""

ChkBtnmotorMowForceOff=tk.Checkbutton(tabMowMotor, text="SAFETY Force Mowing OFF ",relief=tk.SOLID,variable=MowVar1,anchor='nw')
ChkBtnmotorMowForceOff.place(x=10,y=300,width=250, height=20)

slidermotorMowSpeedMaxPwm = tk.Scale(tabMowMotor, from_=100, to=255, label='Max PWM Speed',relief=tk.SOLID,orient='horizontal')
slidermotorMowSpeedMaxPwm.place(x=10,y=10,width=250, height=50)
slidermotorMowSpeedMinPwm = tk.Scale(tabMowMotor, from_=100, to=255, label='Min PWM Speed',relief=tk.SOLID,orient='horizontal')
slidermotorMowSpeedMinPwm.place(x=10,y=60,width=250, height=50)
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
ButtonRequestMainSettingFomMower.place(x=10,y=350, height=25, width=150)

ButtonSetMainApply = tk.Button(tabMowMotor,command = ButtonSetMowMotorApply_click,text="Send To Mower")
ButtonSetMainApply.place(x=300,y=350, height=25, width=100)


"""************* Odometry setting *****************************"""

sliderodometryTicksPerRevolution = tk.Scale(tabOdometry, from_=500, to=1500, label='Ticks for one full revolution',relief=tk.SOLID,orient='horizontal')
sliderodometryTicksPerRevolution.place(x=10,y=10,width=250, height=50)
sliderodometryTicksPerCm = tk.Scale(tabOdometry, from_=0, to=30,resolution=0.1, label='Ticks Odo per CM  ',relief=tk.SOLID,orient='horizontal')
sliderodometryTicksPerCm.place(x=10,y=60,width=250, height=50)
sliderodometryWheelBaseCm = tk.Scale(tabOdometry, from_=0, to=50, label='Distance between the 2 Wheels Motor (CM)',relief=tk.SOLID,orient='horizontal')
sliderodometryWheelBaseCm.place(x=10,y=110,width=250, height=50)

ButtonRequestMainSettingFomMower = tk.Button(tabOdometry,command = read_all_setting,text="Read All From Mower")
ButtonRequestMainSettingFomMower.place(x=10,y=350, height=25, width=150)

ButtonSetMainApply = tk.Button(tabOdometry,command = ButtonSetOdometryApply_click,text="Send To Mower")
ButtonSetMainApply.place(x=300,y=350, height=25, width=100)




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
ButtonRequestTimeSettingFomMower.place(x=10,y=350, height=25, width=150)

ButtonSetMainApply = tk.Button(tabDateTime,command = ButtonSendSettingDateTimeToDue_click,text="Send Time To Mower")
ButtonSetMainApply.place(x=300,y=350, height=25, width=150)


"""************* Battery setting *****************************"""

ChkBtnbatMonitor=tk.Checkbutton(tabBattery, text="Monitor low Battery",relief=tk.SOLID,variable=BatVar1,anchor='nw')
ChkBtnbatMonitor.place(x=10,y=300,width=250, height=20)

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
ButtonRequestMainSettingFomMower.place(x=10,y=350, height=25, width=150)

ButtonSetMainApply = tk.Button(tabBattery,command = ButtonSetBatteryApply_click,text="Send To Mower")
ButtonSetMainApply.place(x=300,y=350, height=25, width=100)


"""************* SONAR setting *****************************"""

ChkBtnsonarCenterUse=tk.Checkbutton(tabSonar, text="Use Sonar center",relief=tk.SOLID,variable=SonVar1,anchor='nw')
ChkBtnsonarCenterUse.place(x=10,y=10,width=250, height=20)
ChkBtnsonarLeftUse=tk.Checkbutton(tabSonar, text="Use Sonar Left",relief=tk.SOLID,variable=SonVar2,anchor='nw')
ChkBtnsonarLeftUse.place(x=10,y=40,width=250, height=20)
ChkBtnsonarRightUse=tk.Checkbutton(tabSonar, text="Use Sonar Right",relief=tk.SOLID,variable=SonVar3,anchor='nw')
ChkBtnsonarRightUse.place(x=10,y=70,width=250, height=20)

ChkBtnsonarLikeBumper=tk.Checkbutton(tabSonar, text="Sonar like a Bumper",relief=tk.SOLID,variable=SonVar4,anchor='nw')
ChkBtnsonarLikeBumper.place(x=10,y=100,width=250, height=20)

slidersonarTriggerBelow = tk.Scale(tabSonar,orient='horizontal',relief=tk.SOLID, from_=20, to=150, label='Detect Below in CM ')
slidersonarTriggerBelow.place(x=10,y=130,width=250, height=50)

slidersonarToFront = tk.Scale(tabSonar,orient='horizontal',relief=tk.SOLID, from_=0, to=100, label='Sonar to mower Front in CM ')
slidersonarToFront.place(x=10,y=200,width=250, height=50)

ButtonRequestMainSettingFomMower = tk.Button(tabSonar)
ButtonRequestMainSettingFomMower.place(x=10,y=350, height=25, width=150)
ButtonRequestMainSettingFomMower.configure(command = read_all_setting)
ButtonRequestMainSettingFomMower.configure(text="Read All From Mower")

ButtonSetMainApply = tk.Button(tabSonar)
ButtonSetMainApply.place(x=300,y=350, height=25, width=100)
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
ChkBtnstopMotorDuringCalib.place(x=5,y=320,width=250, height=20)
ButtonGyroCal= tk.Button(tabImu)
ButtonGyroCal.place(x=330,y=10, height=40, width=200)
ButtonGyroCal.configure(command = ButtonGyroCal_click,text="Start GYRO Calibration")
ButtonCompasCal= tk.Button(tabImu)
ButtonCompasCal.place(x=330,y=60, height=40, width=200)
ButtonCompasCal.configure(command = ButtonCompasCal_click,text="Start COMPAS Calibration")

ButtonRequestMainSettingFomMower = tk.Button(tabImu)
ButtonRequestMainSettingFomMower.place(x=10,y=350, height=25, width=150)
ButtonRequestMainSettingFomMower.configure(command = read_all_setting)
ButtonRequestMainSettingFomMower.configure(text="Read All From Mower")

ButtonSetMainApply = tk.Button(tabImu)
ButtonSetMainApply.place(x=300,y=350, height=25, width=100)
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



ChkBtnMotorSwapLeftDir=tk.Checkbutton(tabWheelMotor, text="Swap left Dir",relief=tk.SOLID,variable=MotVar1,anchor='nw')
ChkBtnMotorSwapLeftDir.place(x=535,y=215,width=130, height=20)

ChkBtnMotorSwapRightDir=tk.Checkbutton(tabWheelMotor, text="Swap right Dir",relief=tk.SOLID,variable=MotVar2,anchor='nw')
ChkBtnMotorSwapRightDir.place(x=535,y=240,width=130, height=20)


ButtonRequestMotSettingFomMower = tk.Button(tabWheelMotor)
ButtonRequestMotSettingFomMower.place(x=535,y=270, height=25, width=140)
ButtonRequestMotSettingFomMower.configure(command = read_all_setting)
ButtonRequestMotSettingFomMower.configure(text="Read From Mower")

ButtonSetMotApply = tk.Button(tabWheelMotor)
ButtonSetMotApply.place(x=535,y=300, height=25, width=100)
ButtonSetMotApply.configure(command = ButtonSetMotApply_click,text="Send To Mower")



"""************* Perimeter setting *****************************"""
def ButtonSetPerimeterApply_click():
    myRobot.perimeter_timedOutIfBelowSmag=sliderTimeBelowSmag.get()
    myRobot.perimeter_timeOutSecIfNotInside=sliderTimeNotInside.get()
    myRobot.perimeterTriggerMinSmag=sliderTrigMinSmag.get()
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
     
def ButtonStartArea1_click():
    ipMessage="curl "+Sender1AdressIP+"/A1"
    consoleInsertText("Try to Start Area1 Sender " + ipMessage + '\n')
    sub = subprocess.Popen("curl "+Sender1AdressIP+"/A1", stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
    output,error_output = sub.communicate()
    if str(output)=="b'SENDER A IS ON'":
        mymower.sigAreaOff=False
        consoleInsertText("Area1 Sender is Running" + '\n')    
    else:
        mymower.sigAreaOff=True
        consoleInsertText("*********** Area1 Sender FAIL To Start ************" + '\n')     
        #print (error_output)
        consoleInsertText(str(error_output)+ '\n')
    #avoid when start from station to change the area to go ??
    if (myRobot.stateNames[mymower.state]!='STREV'):
        send_var_message('w','areaInMowing','1','0','0','0','0','0','0','0')
    
def ButtonStopArea1_click():
    ipMessage="curl "+Sender1AdressIP+"/A0"
    consoleInsertText("Try to Stop Area1 Sender " + ipMessage + '\n')  
    sub = subprocess.Popen("curl "+Sender1AdressIP+"/A0", stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
    output, error_output = sub.communicate()   
    if str(output)=="b'SENDER A IS OFF'":
        mymower.sigAreaOff=True
        consoleInsertText("Area1 Sender is Stop" + '\n')   
    else:
        print ("Area1 Sender FAIL TO response")
        consoleInsertText("*********** Area1 Sender FAIL To Stop ************" + '\n')   
        print (error_output)
        consoleInsertText(str(error_output)+ '\n') 
    send_var_message('w','areaInMowing','1','0','0','0','0','0','0','0')
    
def ButtonStartArea2_click():
    ipMessage="curl "+Sender2AdressIP+"/B1"
    consoleInsertText("Try to Start Area2 Sender" + ipMessage + '\n')
    sub = subprocess.Popen("curl "+Sender2AdressIP+"/B1", stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
    output,error_output = sub.communicate()
    if str(output)=="b'SENDER B IS ON'":
        mymower.sigAreaOff=False
        consoleInsertText("Area2 Sender is Running" + '\n')    
    else:
        mymower.sigAreaOff=True
        consoleInsertText("*********** Area2 Sender FAIL To Start ************" + '\n')     
        #print (error_output)
        consoleInsertText(str(error_output)+ '\n') 
    send_var_message('w','areaInMowing','2','0','0','0','0','0','0','0')
         
def ButtonStopArea2_click():
    ipMessage="curl "+Sender2AdressIP+"/B0"
    consoleInsertText("Try to Stop Area2 Sender " + ipMessage + '\n')  
    sub = subprocess.Popen("curl "+Sender2AdressIP+"/B0", stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
    output, error_output = sub.communicate()   
    if str(output)=="b'SENDER B IS OFF'":
        mymower.sigAreaOff=True
        consoleInsertText("Area2 Sender is Stop" + '\n')   
    else:
        print ("Area2 Sender FAIL TO response")
        consoleInsertText("*********** Area2 Sender FAIL To Stop ************" + '\n')   
        print (error_output)
        consoleInsertText(str(error_output)+ '\n') 
    send_var_message('w','areaInMowing','1','0','0','0','0','0','0','0')

def ButtonStartArea3_click():
    ipMessage="curl "+Sender3AdressIP+"/B1"
    consoleInsertText("Try to Start Area3 Sender" + ipMessage + '\n')
    sub = subprocess.Popen("curl "+Sender3AdressIP+"/B1", stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
    output,error_output = sub.communicate()
    if str(output)=="b'SENDER B IS ON'":
        mymower.sigAreaOff=False
        consoleInsertText("Area3 Sender is Running" + '\n')    
    else:
        mymower.sigAreaOff=True
        consoleInsertText("*********** Area3 Sender FAIL To Start ************" + '\n')     
        #print (error_output)
        consoleInsertText(str(error_output)+ '\n') 
    send_var_message('w','areaInMowing','3','0','0','0','0','0','0','0')
    
def ButtonStopArea3_click():
    ipMessage="curl "+Sender3AdressIP+"/B0"
    consoleInsertText("Try to Stop Area3 Sender" + '\n')  
    sub = subprocess.Popen("curl "+Sender3AdressIP+"/B0", stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
    output, error_output = sub.communicate()   
    if str(output)=="b'SENDER B IS OFF'":
        mymower.sigArea3Off=True
        consoleInsertText("Area3 Sender is Stop" + '\n')   
    else:
        print ("Area3 Sender FAIL TO response")
        consoleInsertText("*********** Area3 Sender FAIL To Stop ************" + '\n')   
        print (error_output)
        consoleInsertText(str(error_output)+ '\n') 
    send_var_message('w','areaInMowing','1','0','0','0','0','0','0','0')                 

sliderTimeBelowSmag = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=500, label='Error if Smag below (0 to 500)')
sliderTimeBelowSmag.place(x=5,y=10,width=250, height=50)
sliderTimeNotInside = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=20, label='Timeout if not inside (0 to 20) in Sec')
sliderTimeNotInside.place(x=5,y=60,width=250, height=50)
sliderTrigMinSmag = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=1000, label='Big Area Smag Center (0 to 1000)')
sliderTrigMinSmag.place(x=5,y=110,width=250, height=50)
sliderTrackingSpeed = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=255, label='Tracking PWM Max Speed (0 to 255)')
sliderTrackingSpeed.place(x=5,y=160,width=250, height=50)
sliderCircleArcDistance = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=1, to=250, label='Circle Arc Distance (cm)')
sliderCircleArcDistance.place(x=5,y=210,width=250, height=50)
sliderPeriMagMaxValue = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=500, to=2500, label='Perimeter MAG Max Value (500 to 2500)')
sliderPeriMagMaxValue.place(x=5,y=260,width=250, height=50)

sliderTrackErrTimeout = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=10000, label='Track error Timeout (0 to 10000)')
sliderTrackErrTimeout.place(x=270,y=10,width=250, height=50)
sliderTrackPid_P = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=50,resolution=0.1, label='Tracking PID value P (0 to 50)')
sliderTrackPid_P.place(x=270,y=60,width=250, height=50)
sliderTrackPid_I = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=50, resolution=0.1,label='Tracking PID value I (0 to 50)')
sliderTrackPid_I.place(x=270,y=110,width=250, height=50)
sliderTrackPid_D = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=50, resolution=0.1,label='Tracking PID value D (0 to 50)')
sliderTrackPid_D.place(x=270,y=160,width=250, height=50)
sliderTransitionTimeout = tk.Scale(tabPerimeter,orient='horizontal',relief=tk.SOLID, from_=0, to=5000, label='Transition Timeout (0 to 5000) in msec')
sliderTransitionTimeout.place(x=270,y=210,width=250, height=50)


ChkBtnPeriSwapLeftCoil=tk.Checkbutton(tabPerimeter, text="Swap Left Coil polarity",relief=tk.SOLID,variable=PeriVar1,anchor='nw')
ChkBtnPeriSwapLeftCoil.place(x=535,y=40,width=250, height=20)

ChkBtnPeriSwapRightCoil=tk.Checkbutton(tabPerimeter, text="Swap Right Coil polarity",relief=tk.SOLID,variable=PeriVar2,anchor='nw')
ChkBtnPeriSwapRightCoil.place(x=535,y=70,width=250, height=20)
ChkBtnPeriRead2Coil=tk.Checkbutton(tabPerimeter, text="Read The 2 Coil",relief=tk.SOLID,variable=PeriVar3,anchor='nw')
ChkBtnPeriRead2Coil.place(x=535,y=100,width=250, height=20)
ChkBtnPeriBlockInnWheel=tk.Checkbutton(tabPerimeter, text="Block Inner Wheel",relief=tk.SOLID,variable=PeriVar4,anchor='nw')
ChkBtnPeriBlockInnWheel.place(x=535,y=130,width=250, height=20)


ButtonRequestSettingFomMower = tk.Button(tabPerimeter)
ButtonRequestSettingFomMower.place(x=10,y=350, height=25, width=150)
ButtonRequestSettingFomMower.configure(command = read_all_setting)
ButtonRequestSettingFomMower.configure(text="Read From Mower")

ButtonSetPerimeterApply = tk.Button(tabPerimeter)
ButtonSetPerimeterApply.place(x=200,y=350, height=25, width=100)
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
Frame1.place(x=10, y=260, height=90, width=650)
Frame1.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")


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
ButtonRequestSettingFomMower.place(x=10,y=350, height=25, width=150)
ButtonRequestSettingFomMower.configure(command = read_all_setting)
ButtonRequestSettingFomMower.configure(text="Read All From Mower")


ButtonSetByLaneApply = tk.Button(tabByLane)
ButtonSetByLaneApply.place(x=300,y=350, height=25, width=150)
ButtonSetByLaneApply.configure(command = ButtonSendSettingByLaneToDue_click,text="Send By Lane To Mower")


"""************* Vision setting *****************************"""


#exemple of vision_list=[['person',1,80,15],['chair',52,85,18,]]
#vision_list=[['person',1,80,15],['chair',52,85,18]]
#Read the file and create the list
with open("vision_list.bin","rb") as fp :
        vision_list=pickle.load(fp)
        print("Vision file OK")
        
      
def OnClick_vision(app):
    try:
        itemVision = treeVision.selection()[0]
        txtVisionObject.delete('1.0', 'end')
        txtVisionObject.insert('1.0', vision_list[treeVision.item(itemVision,"text")][0])
        txtVisionID.delete('1.0', 'end')
        txtVisionID.insert('1.0', vision_list[treeVision.item(itemVision,"text")][1])
        txtVisionScore.delete('1.0', 'end')
        txtVisionScore.insert('1.0', vision_list[treeVision.item(itemVision,"text")][2])
        txtVisionArea.delete('1.0', 'end')
        txtVisionArea.insert('1.0', vision_list[treeVision.item(itemVision,"text")][3])
              
    except:
        print("Please click on correct line")


    
#Building the treeVisionView    
treeVision = ttk.Treeview(tabVision)
scrollbarVision1 = ttk.Scrollbar(treeVision,orient="vertical",command=treeVision.yview)
scrollbarVision1.pack(side=tk.RIGHT, fill=tk.Y)
treeVision.configure(yscrollcommand=scrollbarVision1.set)

minwidth = treeVision.column('#0', option='minwidth')
treeVision.column('#0', width=minwidth)

treeVision["columns"]=("0","1","2","3")
treeVision.column("0", width=150)
treeVision.column("1", width=60,anchor='center')
treeVision.column("2", width=60,anchor='center')
treeVision.column("3", width=60,anchor='center')

treeVision.heading("0", text="Name")
treeVision.heading("1", text="Object ID ")
treeVision.heading("2", text=" % Score")
treeVision.heading("3", text=" % Image Area")

treeVision.bind("<ButtonRelease-1>", OnClick_vision)
treeVision.place(x=0, y=0, height=300, width=520)



def rebuild_treeVisionview():
    for i in treeVision.get_children():
        treeVision.delete(i)   
    for i in range(0,len(vision_list)):
        treeVision.insert("" ,'end' ,text=i, values=(vision_list[i][0],vision_list[i][1],vision_list[i][2],vision_list[i][3]))
    

def save_VisionList():
    with open("vision_list.bin","wb") as fp :
        pickle.dump(vision_list,fp)
            
def del_vision_line():
    curr = treeVision.focus()
    if '' == curr: return
    search_object=txtVisionObject.get("1.0",'end-1c')
     
    for i in range(0,len(vision_list)):
        if (str(vision_list[i][0])== str(search_object)):
            for value in vision_list[:]:
                if (value[0] == search_object):
                    vision_list.remove(value)
                    print("remov ok")
                
            break
    
    rebuild_treeVisionview()
        
def add_vision_line(): 
    maVisionList=[txtVisionObject.get("1.0",'end-1c'),txtVisionID.get("1.0",'end-1c'),txtVisionScore.get("1.0",'end-1c'),txtVisionArea.get("1.0",'end-1c')]
    vision_list.append(maVisionList)
    rebuild_treeVisionview()

def test1():
    search_code=(b'dog')
    for i in range(0,len(vision_list)):
        if (str("b'"+vision_list[i][0]+"'")== str(search_code)):
            print("trouv")
            print(vision_list[i])   

tk.Label(tabVision,text="Name:",font=("Arial", 10), fg='green').place(x=530,y=5, height=20, width=80)
tk.Label(tabVision,text="ID:",font=("Arial", 10), fg='green').place(x=530,y=25, height=20, width=80)
tk.Label(tabVision,text="Score:",font=("Arial", 10), fg='green').place(x=530,y=45, height=20, width=80)
tk.Label(tabVision,text="Area:",font=("Arial", 10), fg='green').place(x=530,y=65, height=20, width=80)

txtVisionObject = tk.Text(tabVision)
txtVisionObject.place(x=630,y=5,width=120, height=20)
txtVisionObject.delete('1.0', 'end')
txtVisionObject.insert('1.0', vision_list[0][0])
txtVisionID = tk.Text(tabVision)
txtVisionID.place(x=630,y=25,width=120, height=20)
txtVisionID.delete('1.0', 'end')
txtVisionID.insert('1.0', vision_list[0][1])
txtVisionScore = tk.Text(tabVision)
txtVisionScore.place(x=630,y=45,width=120, height=20)
txtVisionScore.delete('1.0', 'end')
txtVisionScore.insert('1.0', vision_list[0][2])
txtVisionArea = tk.Text(tabVision)
txtVisionArea.place(x=630,y=65,width=120, height=20)
txtVisionArea.delete('1.0', 'end')
txtVisionArea.insert('1.0', vision_list[0][3])


ButtonVisionAdd = tk.Button(tabVision, text="Add ",command = add_vision_line)
ButtonVisionAdd.place(x=530, y=170, height=30, width=110)
ButtonVisionDel = tk.Button(tabVision, text="Delete ",command = del_vision_line)
ButtonVisionDel.place(x=660, y=170, height=30, width=100)
ButtonVisionSave = tk.Button(tabVision, text="Save to File",command = save_VisionList)
ButtonVisionSave.place(x=530, y=210, height=60, width=140)

#ButtonVisionTest = tk.Button(tabVision, text="seek test",command = test1)
#ButtonVisionTest.place(x=530, y=260, height=60, width=140)

rebuild_treeVisionview()



""" THE AUTO PAGE ***************************************************"""
def button_home_click():
    tempVar=mymower.millis+3600000
    send_var_message('w','nextTimeTimer',''+str(int(tempVar))+'','0','0','0','0','0','0','0')
    send_var_message('w','statusCurr','3','0','0','0','0','0','0','0') #set the status to back to station 
    send_pfo_message('rh','1','2','3','4','5','6',)
    
def button_track_click():
    mymower.areaToGo=tk_areaToGo.get()
    send_var_message('w','mowPatternCurr',''+str(tk_mowingPattern.get())+'','laneUseNr','1','rollDir','0','0','0','0')
    send_var_message('w','whereToStart','1','areaToGo',''+str(tk_areaToGo.get())+'','actualLenghtByLane','50','0','0','0')
    send_pfo_message('rk','1','2','3','4','5','6',)

def buttonStartMow_click():
    mymower.areaToGo=tk_areaToGo.get()
    send_var_message('w','areaToGo',''+str(tk_areaToGo.get())+'','0','0','0','0','0','0','0')
    send_var_message('w','mowPatternCurr',''+str(tk_mowingPattern.get())+'','0','0','0','0','0','0','0')
    send_pfo_message('ra','1','2','3','4','5','6',)
   


AutoPage = tk.Frame(fen1)
AutoPage.place(x=0, y=0, height=400, width=800)

batteryFrame= tk.Frame(AutoPage)
batteryFrame.place(x=230, y=10, height=60, width=100)
batteryFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")

tk.Label(batteryFrame,text="BATTERY",fg='green').pack(side='top',anchor='n')
tk.Label(batteryFrame,textvariable=tk_batVoltage, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

temperatureFrame= tk.Frame(AutoPage)
temperatureFrame.place(x=340, y=10, height=60, width=100)
temperatureFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")

tk.Label(temperatureFrame,text="TEMPERATURE",fg='green').pack(side='top',anchor='n')
tk.Label(temperatureFrame,textvariable=tk_Dht22Temp, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

loopsFrame= tk.Frame(AutoPage)
loopsFrame.place(x=445, y=10, height=60, width=100)
loopsFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")

tk.Label(loopsFrame,text="LOOP/SEC",fg='green').pack(side='top',anchor='n')
tk.Label(loopsFrame,textvariable=tk_loopsPerSecond, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

yawFrame= tk.Frame(AutoPage)
yawFrame.place(x=245, y=70, height=60, width=80)
yawFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
tk.Label(yawFrame,text="YAW",fg='green').pack(side='top',anchor='n')
tk.Label(yawFrame,textvariable=tk_ImuYaw, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

pitchFrame= tk.Frame(AutoPage)
pitchFrame.place(x=355, y=70, height=60, width=80)
pitchFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
tk.Label(pitchFrame,text="PITCH",fg='green').pack(side='top',anchor='n')
tk.Label(pitchFrame,textvariable=tk_ImuPitch, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')


rollFrame= tk.Frame(AutoPage)
rollFrame.place(x=460, y=70, height=60, width=80)
rollFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
tk.Label(rollFrame,text="ROLL",fg='green').pack(side='top',anchor='n')
tk.Label(rollFrame,textvariable=tk_ImuRoll, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')




Frame1 = tk.Frame(AutoPage)
Frame1.place(x=10, y=180, height=150, width=520)
Frame1.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
tk.Label(Frame1,text="MOW PATTERN",fg='green').place(x=5,y=5)
RdBtn_Random=tk.Radiobutton(Frame1, text="Random",font=("Arial", 12), variable=tk_mowingPattern, value=0).place(x=5,y=25,width=90, height=30)
RdBtn_ByLane=tk.Radiobutton(Frame1, text="By Lane", font=("Arial", 12),variable=tk_mowingPattern, value=1).place(x=105,y=25,width=90, height=30)
RdBtn_Perimeter=tk.Radiobutton(Frame1, text="Wire", font=("Arial", 12),variable=tk_mowingPattern, value=2).place(x=205,y=25,width=90, height=30)

tk.Label(Frame1,text="AREA Nr",fg='green').place(x=5,y=65)
RdBtn_Random=tk.Radiobutton(Frame1, text="1",font=("Arial", 12), variable=tk_areaToGo, value=1).place(x=5,y=85,width=90, height=30)
RdBtn_ByLane=tk.Radiobutton(Frame1, text="2", font=("Arial", 12),variable=tk_areaToGo, value=2).place(x=105,y=85,width=90, height=30)
RdBtn_Perimeter=tk.Radiobutton(Frame1, text="3", font=("Arial", 12),variable=tk_areaToGo, value=3).place(x=205,y=85,width=90, height=30)

ButtonStartMow = tk.Button(AutoPage, image=imgstartMow, command = buttonStartMow_click)
ButtonStartMow.place(x=10,y=10,width=100, height=130)
Buttonhome = tk.Button(AutoPage, image=imgHome, command = button_home_click)
Buttonhome.place(x=120,y=10,width=100, height=130)
Buttontrack = tk.Button(Frame1, image=imgTrack, command = button_track_click)
Buttontrack.place(x=380,y=10,width=100, height=130)
ButtonStopAllAuto = tk.Button(AutoPage, image=imgStopAll, command = button_stop_all_click)
ButtonStopAllAuto.place(x=580,y=10,width=100, height=130)


ButtonBackHome = tk.Button(AutoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)






"""**************************THE MANUAL PAGE  **********************************************"""


    

    

def ButtonJoystickON_click() :
    subprocess.call(["rfkill","unblock","bluetooth"])
    print ("BlueTooth Start")
    returnval=messagebox.askyesno('Info',"Power ON the joystick now and wait until the led is fix")
    if returnval :
        try:
            #if not pygame.joystick.get_init():
            #print(myps4.get_init())
            myps4.init()
            myps4.listen()
            mymower.useJoystick=True
            #print(myps4.get_init())
        except:
            subprocess.call(["rfkill","block","bluetooth"])
            returnval=messagebox.showerror('Error',"The joystick is not found: Please try to connect under PI GUI First")

def ButtonJoystickOFF_click() :
    subprocess.call(["rfkill","block","bluetooth"])
    print ("BlueTooth Stop")
    mymower.useJoystick=False
    messagebox.showinfo('Info',"Bluetooth and joystick are OFF")
        
def buttonBlade_stop_click():
    send_var_message('w','stateCurr','17','0','0','0','0','0','0','0') #set the state to Manual before stop the blade
    send_cmd_message('mowmotor','0','0','0','0')
    
def buttonBlade_start_click():
    send_var_message('w','stateCurr','17','0','0','0','0','0','0','0') #set the state to Manual before start the blade
    send_cmd_message('mowmotor','1','0','0','4')
    

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
     
    
ManualPage = tk.Frame(fen1)
ManualPage.place(x=0, y=0, height=400, width=800)
Frame1 = tk.Frame(ManualPage)
Frame1.place(x=0, y=0, height=300, width=300)


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
ButtonBladeStart.place(x=680,y=5,width=100, height=50)
ButtonBladeStop = tk.Button(ManualPage, image=imgBladeStop, command = buttonBlade_stop_click)
ButtonBladeStop.place(x=680,y=55,width=100, height=80)



ButtonStopAllManual = tk.Button(ManualPage, image=imgStopAll, command = button_stop_all_click)
ButtonStopAllManual.place(x=400,y=200,width=100, height=130)

tk.Label(ManualPage, image=imgJoystick).place(x=400,y=5)
ButtonJoystickON = tk.Button(ManualPage, image=imgJoystickON, command = ButtonJoystickON_click)
ButtonJoystickON.place(x=450,y=80,width=50, height=50)
ButtonJoystickOFF = tk.Button(ManualPage, image=imgJoystickOFF, command = ButtonJoystickOFF_click)
ButtonJoystickOFF.place(x=400,y=80,width=50, height=50)

RdBtn_keyboard=tk.Checkbutton(ManualPage, text="Use Keyboard to drive the mower", variable=ManualKeyboardUse).place(x=0,y=365)



manualSpeedSlider = tk.Scale(ManualPage,orient='horizontal', from_=0, to=255)
manualSpeedSlider.place(x=0,y=300,width=300, height=60)
tk.Label(ManualPage, text='SPEED',fg='green').place(x=0,y=300)

"""
slider1 = tk.Scale(orient='horizontal', from_=0, to=350)
slider1.place(x=50,y=50,anchor='nw',width=300, height=50)
slider2 = tk.Scale(orient='horizontal', from_=0, to=100)
slider2.place(x=50,y=100,anchor='nw',width=300, height=50)
"""
ButtonBackHome = tk.Button(ManualPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)



""" The Console page  ************************************"""

def ButtonListVar_click():
    send_pfo_message('h02','1','2','3','4','5','6',)


def ButtonConsoleMode_click():
    send_pfo_message('h03','1','2','3','4','5','6',)

def ButtonClearConsole_click():
    txtConsoleRecu.delete('1.0',tk.END) #delete all
    

ConsolePage = tk.Frame(fen1)
ConsolePage.place(x=0, y=0, height=400, width=800)


txtRecu = tk.Text(ConsolePage)
ScrollTxtRecu = tk.Scrollbar(txtRecu)
ScrollTxtRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtRecu.config(command=txtRecu.yview)
txtRecu.config(yscrollcommand=ScrollTxtRecu.set)
txtRecu.place(x=0,y=300,anchor='nw',width=480, height=100)

txtSend = tk.Text(ConsolePage)
ScrollTxtSend = tk.Scrollbar(txtSend)
ScrollTxtSend.pack(side=tk.RIGHT, fill=tk.Y)
txtSend.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtSend.config(command=txtSend.yview)
txtSend.config(yscrollcommand=ScrollTxtSend.set)
txtSend.place(x=490,y=300,anchor='nw',width=300, height=100)

txtConsoleRecu = tk.Text(ConsolePage)
ScrolltxtConsoleRecu = tk.Scrollbar(txtConsoleRecu)
ScrolltxtConsoleRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtConsoleRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrolltxtConsoleRecu.config(command=txtConsoleRecu.yview)
txtConsoleRecu.config(yscrollcommand=ScrolltxtConsoleRecu.set)
txtConsoleRecu.place(x=0,y=5,anchor='nw',width=800, height=290)


ButtonConsoleMode = tk.Button(ConsolePage)
ButtonConsoleMode.place(x=660,y=15, height=25, width=120)
ButtonConsoleMode.configure(command = ButtonConsoleMode_click,text="Mode")


ButtonListVar = tk.Button(ConsolePage)
ButtonListVar.place(x=660,y=45, height=25, width=120)
ButtonListVar.configure(command = ButtonListVar_click,text="List Var")

ButtonClearConsole = tk.Button(ConsolePage)
ButtonClearConsole.place(x=660,y=75, height=35, width=120)
ButtonClearConsole.configure(command = ButtonClearConsole_click,text="Clear Console")

ButtonSaveReceived = tk.Button(ConsolePage)
ButtonSaveReceived.place(x=660,y=120, height=35, width=120)
ButtonSaveReceived.configure(command = ButtonSaveReceived_click,text="Save To File")





ButtonBackHome = tk.Button(ConsolePage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=660, y=160, height=120, width=120)

""" THE PLOT PAGE ***************************************************"""

TabPlot=ttk.Notebook(fen1)
tabPlotMain=tk.Frame(TabPlot,width=800,height=400)
tabPlotWheelMotor=tk.Frame(TabPlot,width=800,height=200)
tabPlotMowMotor=tk.Frame(TabPlot,width=800,height=200)
tabPlotPerimeter=tk.Frame(TabPlot,width=800,height=200)
tabPlotBattery=tk.Frame(TabPlot,width=800,height=200)
tabPlotImu=tk.Frame(TabPlot,width=800,height=200)

TabPlot.add(tabPlotMain,text="Main")
TabPlot.add(tabPlotWheelMotor,text="Wheels Motor")
TabPlot.add(tabPlotMowMotor,text="Mow Motor")
TabPlot.add(tabPlotPerimeter,text="Perimeter")
TabPlot.add(tabPlotBattery,text="Battery")
TabPlot.add(tabPlotImu,text="Imu")

TabPlot.place(x=0, y=0, height=400, width=800)

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


tk.Label(Frame12, text='Power',fg='green').place(x=400,y=0)
tk.Label(Frame12, text='Left',fg='green').place(x=350,y=15)
tk.Label(Frame12, textvariable=tk_motorLeftPower).place(x=400,y=15)
tk.Label(Frame12, text='Right',fg='green').place(x=350,y=35)
tk.Label(Frame12, textvariable=tk_motorRightPower).place(x=400,y=35)
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

tk.Label(Frame13, text='Power',fg='green').place(x=400,y=0)
tk.Label(Frame13, textvariable=tk_motorMowPower).place(x=400,y=15)
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

#'Imu'
tk.Label(tabPlotImu, text="Mower Millis : ").place(x=0,y=200)
tk.Label(tabPlotImu, textvariable=tk_millis).place(x=100,y=200)
Frame16= tk.Frame(tabPlotImu,relief=tk.GROOVE,borderwidth="3")
Frame16.place(x=5, y=5, height=155, width=390)
BtnImuPlotStartRec= tk.Button(Frame16,command = BtnImuPlotStartRec_click,text="Start")
BtnImuPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnImuPlotStopRec= tk.Button(Frame16,command = BtnImuPlotStopRec_click,text="Stop")
BtnImuPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainImuRefresh = tk.Scale(Frame16, from_=1, to=10, label='Refresh Rate per seconde',relief=tk.SOLID,orient='horizontal')
SldMainImuRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame16, text='Gyro',fg='green').place(x=50,y=75)
tk.Label(Frame16, text='Compass',fg='green').place(x=50,y=95)
tk.Label(Frame16, text='Value',fg='green').place(x=160,y=60)
tk.Label(Frame16, textvariable=tk_gyroYaw).place(x=160,y=75)
tk.Label(Frame16, textvariable=tk_compassYaw).place(x=160,y=95)

ButtonBackHome = tk.Button(TabPlot, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)


"""
 THE INFO PAGE ***************************************************
"""
InfoPage = tk.Frame(fen1)
InfoPage.place(x=0, y=0, height=400, width=800)
Infoline1=tk.StringVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline1)
LabInfoline.place(x=10,y=10, height=25, width=450)
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
ButtonBackHome = tk.Button(InfoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)


#myWindows.loadPageInfo()
""" THE GPS PAGE ***************************************************"""
def degrees_to_decimal(data, hemisphere):
        try:
            decimalPointPosition = data.index('.')
            degrees = float(data[:decimalPointPosition-2])
            minutes = float(data[decimalPointPosition-2:])/60
            output = degrees + minutes
            if hemisphere is 'N' or hemisphere is 'E':
                return output
            if hemisphere is 'S' or hemisphere is 'W':
                return -output
        except:
            return ""

def parse_GPRMC(data):
       data = data.split(',')
       dict = {
               'fix_time': data[1],
               'validity': data[2],
               'latitude': data[3],
               'latitude_hemisphere' : data[4],
               'longitude' : data[5],
               'longitude_hemisphere' : data[6],
               'speed': data[7],
               'true_course': data[8],
               'fix_date': data[9],
               'variation': data[10],
               'variation_e_w' : data[11],
               'checksum' : data[12]
       }
       dict['decimal_latitude'] = degrees_to_decimal(dict['latitude'], dict['latitude_hemisphere'])
       dict['decimal_longitude'] = degrees_to_decimal(dict['longitude'], dict['longitude_hemisphere'])
       return dict



def BtnGpsRecordStart_click():
    BtnGpsRecordStart.configure(state='disabled')
    BtnGpsRecordStop.configure(state='normal')
    if GpsConnectedOnPi :
        mygpsRecord.start()
    else:
        send_var_message('w','gpsUse','1','0','0','0','0','0','0','0')
        
       
        
def BtnGpsRecordStop_click():
    BtnGpsRecordStart.configure(state='normal')
    BtnGpsRecordStop.configure(state='disabled')
    if GpsConnectedOnPi :
        mygpsRecord.stop()
    else:
        send_var_message('w','gpsUse','0','0','0','0','0','0','0','0')
        
GpsPage =tk.Frame(fen1)
GpsPage.place(x=0, y=0, height=400, width=800)

#tk.Label(GpsPage, text='The gps run as service and write into directory /gpsdata/').place(x=10,y=15)


txtGpsRecu = tk.Text(GpsPage)
ScrolltxtGpsRecu = tk.Scrollbar(txtGpsRecu)
ScrolltxtGpsRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtGpsRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrolltxtGpsRecu.config(command=txtGpsRecu.yview)
txtGpsRecu.config(yscrollcommand=ScrolltxtGpsRecu.set)
txtGpsRecu.place(x=5,y=35,anchor='nw',width=600, height=300)




BtnGpsRecordStart= tk.Button(GpsPage,command = BtnGpsRecordStart_click,text="Start")
BtnGpsRecordStart.place(x=5,y=5, height=25, width=60)
BtnGpsRecordStop= tk.Button(GpsPage,command = BtnGpsRecordStop_click,text="Stop")
BtnGpsRecordStop.place(x=150,y=5, height=25, width=60)

ButtonBackHome = tk.Button(GpsPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)



""" THE RFID PAGE ***************************************************"""

RfidPage =tk.Frame(fen1)
RfidPage.place(x=0, y=0, height=400, width=800)



"""************* Rfid setting *****************************"""
#exemple of rfid_list=[['01254456700','FR0',60,0,0],['01254456711','FR1',61,1,1],['01254456722','FR2',62,2,2],['01254456733','FR3',63,3,3]]

#Read the file and create the list
with open("tag_list.bin","rb") as fp :
        #rfid_list=[['01254456700','FR0',60,0,0,0,0,0],['01254456711','FR1',61,1,1,0,0,0]]
        rfid_list=pickle.load(fp)
        print("RFID file OK")
        



"""
        
def handler(event):
    current = combobox.current()
    if current != -1:
        print(myRobot.stateNames[current])

combobox = ttk.Combobox(RfidPage,values=myRobot.stateNames, height=60, width=140)
combobox.bind('<<ComboboxSelected>>', handler)
combobox.current(0)
combobox.place(x=530, y=310, height=60, width=140)

"""

      
def OnClick(app):
    try:
        item = tree.selection()[0]
        txtTagNr.delete('1.0', 'end')
        txtTagNr.insert('1.0', rfid_list[tree.item(item,"text")][0])
        txtTagMowerState.delete('1.0', 'end')
        txtTagMowerState.insert('1.0', rfid_list[tree.item(item,"text")][1])
        txtTagToDo.delete('1.0', 'end')
        txtTagToDo.insert('1.0', rfid_list[tree.item(item,"text")][2])
        txtTagSpeed.delete('1.0', 'end')
        txtTagSpeed.insert('1.0', rfid_list[tree.item(item,"text")][3])
        txtTagAngle1.delete('1.0', 'end')
        txtTagAngle1.insert('1.0', rfid_list[tree.item(item,"text")][4])
        txtTagDist1.delete('1.0', 'end')
        txtTagDist1.insert('1.0', rfid_list[tree.item(item,"text")][5]) 
        txtTagAngle2.delete('1.0', 'end')
        txtTagAngle2.insert('1.0', rfid_list[tree.item(item,"text")][6])
        txtTagDist2.delete('1.0', 'end')
        txtTagDist2.insert('1.0', rfid_list[tree.item(item,"text")][7])
      
    except:
        print("Please click on correct line")


    
#Building the treeView    
tree = ttk.Treeview(RfidPage)
scrollbar1 = ttk.Scrollbar(tree,orient="vertical",command=tree.yview)
scrollbar1.pack(side=tk.RIGHT, fill=tk.Y)
tree.configure(yscrollcommand=scrollbar1.set) 
minwidth = tree.column('#0', option='minwidth')
tree.column('#0', width=minwidth)
tree["columns"]=("0","1","2","3","4","5","6","7","8")
tree.column("0", width=100)
tree.column("1", width=50)
tree.column("2", width=50)
tree.column("3", width=50)
tree.column("4", width=50)
tree.column("5", width=50)
tree.column("6", width=50)
tree.column("7", width=50)

tree.heading("0", text="Tag")
tree.heading("1", text="Status")
tree.heading("2", text="ToDO")
tree.heading("3", text="Speed")
tree.heading("4", text="Ang 1")
tree.heading("5", text="Dist 1")
tree.heading("6", text="Ang 2")
tree.heading("7", text="Dist 2")


tree.bind("<ButtonRelease-1>", OnClick)
tree.place(x=0, y=0, height=300, width=520)



def rebuild_treeview():
    #rebuild the treeview       
    for i in tree.get_children():
        tree.delete(i)   
    for i in range(0,len(rfid_list)):
        tree.insert("" ,'end' ,    text=i, values=(rfid_list[i][0],rfid_list[i][1],rfid_list[i][2],rfid_list[i][3],rfid_list[i][4],rfid_list[i][5],rfid_list[i][6],rfid_list[i][7]))

    #print(rfid_list)
    

def save_rfid():
    with open("tag_list.bin","wb") as fp :
        pickle.dump(rfid_list,fp)
    
        
def del_rfid_tag():
    curr = tree.focus()
    if '' == curr: return
    #search=txtTagNr.get("1.0",'end-1c')
    search_code=txtTagNr.get("1.0",'end-1c')
    search_status=txtTagMowerState.get("1.0",'end-1c')
 
    for i in range(0,len(rfid_list)):
        if (str(rfid_list[i][0])== str(search_code)) & (str(rfid_list[i][1])== str(search_status)):
            for value in rfid_list[:]:
                if (value[0] == search_code) & (value[1] == search_status):
                    rfid_list.remove(value)
                    print("remov")
                
            break
    
    rebuild_treeview()
        

def add_rfid_tag():
    #print(rfid_list)
    malist=[txtTagNr.get("1.0",'end-1c'),txtTagMowerState.get("1.0",'end-1c'),txtTagToDo.get("1.0",'end-1c'),txtTagSpeed.get("1.0",'end-1c'),txtTagAngle1.get("1.0",'end-1c'),txtTagDist1.get("1.0",'end-1c'),txtTagAngle2.get("1.0",'end-1c'),txtTagDist2.get("1.0",'end-1c')]
    rfid_list.append(malist)
    #print(rfid_list)
    rebuild_treeview()

def test1():
                          
    search_code=(b'01254456722')
    search_status="ALL"
    
    mymower.newtagToDo="Null"
    for i in range(0,len(rfid_list)):
        #if (str("b'"+rfid_list[i][0]+"'")== str(search_code)) :
        if (str("b'"+rfid_list[i][0]+"'")== str(search_code)) & (str(rfid_list[i][1])== str(search_status)):
            
            print("trouv")
            print(rfid_list[i])
            
     

tk.Label(RfidPage,text="TAG Nr:",font=("Arial", 10), fg='green').place(x=530,y=5, height=20, width=80)
tk.Label(RfidPage,text="Status:",font=("Arial", 10), fg='green').place(x=530,y=25, height=20, width=80)
tk.Label(RfidPage,text="What to Do:",font=("Arial", 10), fg='green').place(x=530,y=45, height=20, width=80)
tk.Label(RfidPage,text="Speed:",font=("Arial", 10), fg='green').place(x=530,y=65, height=20, width=80)
tk.Label(RfidPage,text="Angle 1:",font=("Arial", 10), fg='green').place(x=530,y=85, height=20, width=80)
tk.Label(RfidPage,text="Dist 1:",font=("Arial", 10), fg='green').place(x=530,y=105, height=20, width=80)
tk.Label(RfidPage,text="Angle 2:",font=("Arial", 10), fg='green').place(x=530,y=125, height=20, width=80)
tk.Label(RfidPage,text="Dist 2:",font=("Arial", 10), fg='green').place(x=530,y=145, height=20, width=80)
      


txtTagNr = tk.Text(RfidPage)
txtTagNr.place(x=630,y=5,width=120, height=20)
txtTagNr.delete('1.0', 'end')
txtTagNr.insert('1.0', rfid_list[0][0])
txtTagMowerState = tk.Text(RfidPage)
txtTagMowerState.place(x=630,y=25,width=120, height=20)
txtTagMowerState.delete('1.0', 'end')
txtTagMowerState.insert('1.0', rfid_list[0][1])
txtTagToDo = tk.Text(RfidPage)
txtTagToDo.place(x=630,y=45,width=120, height=20)
txtTagToDo.delete('1.0', 'end')
txtTagToDo.insert('1.0', rfid_list[0][2])
txtTagSpeed = tk.Text(RfidPage)
txtTagSpeed.place(x=630,y=65,width=120, height=20)
txtTagSpeed.delete('1.0', 'end')
txtTagSpeed.insert('1.0', rfid_list[0][3])
txtTagAngle1 = tk.Text(RfidPage)
txtTagAngle1.place(x=630,y=85,width=120, height=20)
txtTagAngle1.delete('1.0', 'end')
txtTagAngle1.insert('1.0', rfid_list[0][4])
txtTagDist1 = tk.Text(RfidPage)
txtTagDist1.place(x=630,y=105,width=120, height=20)
txtTagDist1.delete('1.0', 'end')
txtTagDist1.insert('1.0', rfid_list[0][5])
txtTagAngle2 = tk.Text(RfidPage)
txtTagAngle2.place(x=630,y=125,width=120, height=20)
txtTagAngle2.delete('1.0', 'end')
txtTagAngle2.insert('1.0', rfid_list[0][6])
txtTagDist2 = tk.Text(RfidPage)
txtTagDist2.place(x=630,y=145,width=120, height=20)
txtTagDist2.delete('1.0', 'end')
txtTagDist2.insert('1.0', rfid_list[0][7])


ButtonAdd = tk.Button(RfidPage, text="Add This Tag",command = add_rfid_tag)
ButtonAdd.place(x=530, y=170, height=30, width=110)

ButtonDel = tk.Button(RfidPage, text="Delete this Tag",command = del_rfid_tag)
ButtonDel.place(x=660, y=170, height=30, width=100)

ButtonSave = tk.Button(RfidPage, text="Save to File",command = save_rfid)
ButtonSave.place(x=530, y=210, height=60, width=140)

#ButtonTest = tk.Button(RfidPage, text="seek test",command = test1)
#ButtonTest.place(x=530, y=260, height=60, width=140)

rebuild_treeview()











ButtonBackHome = tk.Button(RfidPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)



""" THE CAMERA PAGE ***************************************************"""
def visionThread(num):
    import os
    import cv2
    import numpy as np
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import tensorflow as tf
    

    import sys

    count=0
    consoleInsertText("Initialise Vision " + '\n')
# Set up camera constants
    #IM_WIDTH = 1280
    #IM_HEIGHT = 720
    #IM_WIDTH = 640   # Use smaller resolution for
    #IM_HEIGHT = 480  # slightly faster framerate
    IM_WIDTH = 544   # Use smaller resolution for
    IM_HEIGHT = 400  # slightly faster framerate
    #IM_WIDTH = 320   # Use smaller resolution for
    #IM_HEIGHT = 240  # slightly faster framerate
    

# This is needed for working directory 
    sys.path.append('..')

# Import utilites
    from utils import label_map_util
    from utils import visualization_utils as vis_util

# Name of the directory containing the object detection module we're using
    MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

# Grab path to current working directory
    CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')
    
# Path to save image
    folder_name=time.strftime("%Y%m%d%H%M%S")
    PATH_TO_SAVE_IMG = cwd + "/vision/" + folder_name +"/"
    os.mkdir(PATH_TO_SAVE_IMG)
    


# Number of classes the object detector can identify
    NUM_CLASSES = 99

## Load the label map.
# Label maps map indices to category names, so that when the convolution
# network predicts `5`, we know that this corresponds to `airplane`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        sess = tf.compat.v1.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Initialize camera and perform object detection.
    #print(category_index) 
##    for index in category_index :
##        
##        print(category_index[index]['id'],category_index[index]['name'])
        

    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)
    consoleInsertText("Vision Started "+ '\n')
       #while(1):
        #videocapture.read(frame1);
        #cv2.videocapture(0)
    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        
        if (time.time() > mymower.visionDetectAt + 2):  #wait before next detection
            
        
            t1 = cv2.getTickCount()
            
        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value
            frame = np.copy(frame1.array)
            frame.setflags(write=1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_expanded = np.expand_dims(frame_rgb, axis=0)
            
        # Perform the actual detection by running the model with the image as input
            #if True:
            if ((myRobot.stateNames[mymower.state]=="OFF") or (myRobot.stateNames[mymower.state]=="FRWODO") or (myRobot.stateNames[mymower.state]=="PFND")):
                
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: frame_expanded})

        # Draw the box and results of the detection inside frame (aka 'visulaize the results')
                vis_util.visualize_boxes_and_labels_on_image_array(
                    frame,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    category_index,
                    use_normalized_coordinates=True,
                    line_thickness=2,
                    min_score_thresh=0.60)#draw box only if score >60%
        
        
        
                cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

        # All  results have been drawn on the frame, so it's time to display it.
                #photo1=ImageTk.PhotoImage(image=Image.fromarray(frame))
                #Video_canvas.create_image(0,0,image = photo1,anchor=tk.NW)
                cv2.imshow('Live Video', frame)
                
                #multiple detection are possible into one frame
                #test only index 0 because it's the higher detection score
                if (scores[0][0] > visionDetectMinScore/100): # see config.py for global min score value
             
                     x = int(((boxes[0][0][1]+boxes[0][0][3])/2)*IM_WIDTH)
                     y = int(((boxes[0][0][0]+boxes[0][0][2])/2)*IM_HEIGHT)

                     lx= int(boxes[0][0][3]*IM_WIDTH-boxes[0][0][1]*IM_WIDTH)
                     ly= int(boxes[0][0][2]*IM_HEIGHT+boxes[0][0][0]*IM_HEIGHT)
                     
                     consoleInsertText("vision detect something here : " + str(x) + "  " + str(y) + '\n')
                     consoleInsertText("size is : " + str(lx) + "  " + str(ly) + '\n')
                     mymower.visionScore=100*scores[0][0]
                     mymower.objectDetectedID=int(classes[0][0])
                     mymower.surfaceDetected=100*(lx*ly)/(IM_WIDTH*IM_HEIGHT)
                     #mymower.visionDetectAt = time.time()
                     mymower.visionDetect=True
                     if (x > IM_WIDTH/2) :
                         mymower.VisionRollRight = 1
                         #consoleInsertText("Mower need to rotate RIGHT" + '\n')

                     else :
                         mymower.VisionRollRight = 0
                         #consoleInsertText("Mower need to rotate LEFT" + '\n')
                                         
                     fileName1=PATH_TO_SAVE_IMG + str(count) + ".jpg"
                     print(fileName1)

                     cv2.imwrite(fileName1, frame)
               #     cv2.imwrite("/home/pi/Documents/PiArdumower/vision/vision/person%d.jpg" % count, frame)
                     count=count+1

                     photo1=ImageTk.PhotoImage(image=Image.fromarray(frame))
                
                     Video_canvas.create_image(0,0,image = photo1,anchor=tk.NW)

            

            else:
                #t1 = cv2.getTickCount()
                #photo1=ImageTk.PhotoImage(image=Image.fromarray(frame))
                #print(1000*(cv2.getTickCount()-t1)/freq)
                #Video_canvas.create_image(0,0,image = photo1,anchor=tk.NW)
                     
                #print(1000*(cv2.getTickCount()-t1)/freq)
                cv2.imshow('Live Video', frame)
                #print(1000*(cv2.getTickCount()-t1)/freq)

             
             
            t2 = cv2.getTickCount()
            time1 = (t2-t1)/freq
            frame_rate_calc = 1/time1
        
        # Press 'q' to quit
            if (cv2.waitKey(1) == ord('q') or not(mymower.visionRunning)):
                send_pfo_message('rd0','1','2','3','4','5','6',)
                break
        #clear frame capture for next loop
        rawCapture.truncate(0)

    camera.close()

    cv2.destroyAllWindows()

    consoleInsertText("Vision Stopped "+ '\n')
       
    
def BtnStreamVideoStart_click():
    consoleInsertText("Start Video streaming" + '\n')
    if CamVar1.get()==1:
        myStreamVideo.start(1)
        
        #webbrowser.open("http://localhost:8000/index.html")

    else:
        myStreamVideo.start(0)
        
def BtnStreamVideoStop_click():
    consoleInsertText("Stop Video streaming" + '\n')
    myStreamVideo.stop()

def BtnVisionStart_click():
    if not (mymower.visionRunning):
        BtnVisionStart.place_forget()
        BtnVisionStop.place(x=650,y=200, height=25, width=80)
        print("start new thread")
        mymower.visionRunning=True
        t1 = threading.Thread(target=visionThread, args=(10,))
        t1.start() 
    
def BtnVisionStop_click():
    if (mymower.visionRunning):
        mymower.visionRunning=False
        print("stop vision")
        BtnVisionStop.place_forget()
        BtnVisionStart.place(x=650,y=200, height=25, width=80)
        
    
        
StreamVideoPage =tk.Frame(fen1)
StreamVideoPage.place(x=0, y=0, height=400, width=800)
FrameStreamVideo = tk.Frame(StreamVideoPage,borderwidth="1",relief=tk.SOLID)
FrameStreamVideo.place(x=0, y=0, width=544, height=400)

Video_canvas=tk.Canvas(FrameStreamVideo,bg='red')
Video_canvas.place(x=0, y=0, width=544, height=400)


OptBtnStreamVideo1=tk.Radiobutton(StreamVideoPage, text="320*240",relief=tk.SOLID,variable=CamVar1,value=0,anchor='nw').place(x=620,y=10,width=100, height=20)
OptBtnStreamVideo2=tk.Radiobutton(StreamVideoPage, text="640*480",relief=tk.SOLID,variable=CamVar1,value=1,anchor='nw').place(x=620,y=30,width=100, height=20)
#tk.Label(FrameStreamVideo, text='To view the vidéo stream use a browser http://(Your PI IP Adress and):8000/index.html').place(x=10,y=180)
#tk.Label(FrameStreamVideo, text='Do not forget to activate the Camera into the RaspiConfig').place(x=10,y=200)

BtnStreamVideoStart= tk.Button(StreamVideoPage,command = BtnStreamVideoStart_click,text="Start Streaming")
BtnStreamVideoStart.place(x=650,y=100, height=25, width=110)
BtnStreamVideoStop= tk.Button(StreamVideoPage,command = BtnStreamVideoStop_click,text="Stop Streaming")
BtnStreamVideoStop.place(x=650,y=130, height=25, width=110)

BtnVisionStart= tk.Button(StreamVideoPage,command = BtnVisionStart_click,text="Start Vision")
if (useVision):
    BtnVisionStart.place(x=650,y=200, height=25, width=80)
BtnVisionStop= tk.Button(StreamVideoPage,command = BtnVisionStop_click,text="Stop Vision")

ButtonBackHome = tk.Button(StreamVideoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

        
        
""" THE TEST PAGE ***************************************************"""




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
#def ButtonGoTOArea2_click():
    #mymower.status=4
    #send_pfo_message('ru','1','2','3','4','5','6',)



TestPage =tk.Frame(fen1)
TestPage.place(x=0, y=0, height=400, width=800)

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




def ButtonWifiOn_click():
    #returnval=messagebox.askyesno('Info',"Turn On the Wifi")
    #if returnval :
        subprocess.Popen("sudo systemctl start hostapd", shell=True)
        subprocess.Popen("sudo systemctl start dnsmasq", shell=True)
        subprocess.Popen("sudo rfkill unblock wifi", shell=True)
        consoleInsertText('WIFI is ON'+ '\n')

def ButtonWifiOff_click():
    #returnval=messagebox.askyesno('Info',"Turn Off the Wifi")
    #if returnval :
        subprocess.Popen("sudo systemctl stop hostapd", shell=True)
        subprocess.Popen("sudo systemctl stop dnsmasq", shell=True)
        subprocess.Popen("sudo rfkill block wifi", shell=True)
        consoleInsertText('WIFI is OFF'+ '\n')

ButtonWifiOn= tk.Button(TestPage)
ButtonWifiOn.place(x=550,y=65, height=25, width=100)
ButtonWifiOn.configure(command = ButtonWifiOn_click)
ButtonWifiOn.configure(text="Wifi On")      


ButtonWifiOff= tk.Button(TestPage)
ButtonWifiOff.place(x=660,y=65, height=25, width=100)
ButtonWifiOff.configure(command = ButtonWifiOff_click)
ButtonWifiOff.configure(text="Wifi Off")

def ButtonBTOn_click():
    returnval=messagebox.askyesno('Info',"Turn On the Bluetooth")
    if returnval :
        subprocess.Popen("sudo rfkill unblock bluetooth", shell=True)
def ButtonBTOff_click():
    returnval=messagebox.askyesno('Info',"Turn Off the Bluetooth")
    if returnval :
        subprocess.Popen("sudo rfkill block bluetooth", shell=True)

ButtonBTOn= tk.Button(TestPage)
ButtonBTOn.place(x=550,y=15, height=25, width=100)
ButtonBTOn.configure(command = ButtonBTOn_click)
ButtonBTOn.configure(text="BT On")      


ButtonBTOff= tk.Button(TestPage)
ButtonBTOff.place(x=660,y=15, height=25, width=100)
ButtonBTOff.configure(command = ButtonBTOff_click)
ButtonBTOff.configure(text="BT Off")

ButtonStartArea1 = tk.Button(TestPage)
ButtonStartArea1.place(x=50,y=315, height=25, width=150)
ButtonStartArea1.configure(command = ButtonStartArea1_click,text="Start Sender Area1")

ButtonStopArea1 = tk.Button(TestPage)
ButtonStopArea1.place(x=50,y=350, height=25, width=150)
ButtonStopArea1.configure(command = ButtonStopArea1_click,text="Stop Sender Area1")



ButtonStartArea2 = tk.Button(TestPage)
ButtonStartArea2.place(x=210,y=315, height=25, width=150)
ButtonStartArea2.configure(command = ButtonStartArea2_click,text="Start Sender Area2")

ButtonStopArea2 = tk.Button(TestPage)
ButtonStopArea2.place(x=210,y=350, height=25, width=150)
ButtonStopArea2.configure(command = ButtonStopArea2_click,text="Stop Sender Area2")

ButtonStartArea3 = tk.Button(TestPage)
ButtonStartArea3.place(x=360,y=315, height=25, width=150)
ButtonStartArea3.configure(command = ButtonStartArea3_click,text="Start Sender Area3")

ButtonStopArea3 = tk.Button(TestPage)
ButtonStopArea3.place(x=360,y=350, height=25, width=150)
ButtonStopArea3.configure(command = ButtonStopArea3_click,text="Stop Sender Area3")

def ButtonStartOut2_click():
    send_pfo_message('re1','1','2','3','4','5','6',)
    
def ButtonStopOut2_click():
    send_pfo_message('re0','1','2','3','4','5','6',)

ButtonStartOut2 = tk.Button(TestPage)
ButtonStartOut2.place(x=560,y=115, height=25, width=75)
ButtonStartOut2.configure(command = ButtonStartOut2_click,text="Start Out2")

ButtonStopOut2 = tk.Button(TestPage)
ButtonStopOut2.place(x=660,y=115, height=25, width=75)
ButtonStopOut2.configure(command = ButtonStopOut2_click,text="Stop Out2")

def ButtonStartOut3_click():
    send_pfo_message('rf1','1','2','3','4','5','6',)
    
def ButtonStopOut3_click():
    send_pfo_message('rf0','1','2','3','4','5','6',)

ButtonStartOut3 = tk.Button(TestPage)
ButtonStartOut3.place(x=560,y=150, height=25, width=75)
ButtonStartOut3.configure(command = ButtonStartOut3_click,text="Start Out3")

ButtonStopOut3 = tk.Button(TestPage)
ButtonStopOut3.place(x=660,y=150, height=25, width=75)
ButtonStopOut3.configure(command = ButtonStopOut3_click,text="Stop Out3")



ButtonBackHome = tk.Button(TestPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)




""" THE TIMER PAGE ***************************************************"""
def SliderHourStartGroup_click(var1):
    pass
    #print("heure change "+str(var1))
     



TabTimer=ttk.Notebook(fen1)
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
tk_timerStartArea = []
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
    tk_timerStartArea.append(tk.IntVar())
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
SliderStartArea=[None]*5
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
    SliderHourStartGroup[i] = tk.Scale(FrameStartGroup[i],command = SliderHourStartGroup_click(i),from_=0, to=23,variable=tk_timerStartTimehour[i],relief=tk.SOLID,orient='horizontal')
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
    SliderStartLaneMaxlenghGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=50,font=("Arial", 8),variable=tk_timerStartLaneMaxlengh[i],relief=tk.SOLID,orient='horizontal')
    SliderStartLaneMaxlenghGroup[i].place(x=10,y=20, height=35, width=360)
    
    tk.Label(FrameLaneParameter[i], text='Roll Dir',font=("Arial", 12), fg='green').place(x=10,y=60, height=15, width=80)
    RdBtn_Right=tk.Radiobutton(FrameLaneParameter[i], text="Right",variable=tk_timerStartRollDir[i], value=0).place(x=10,y=75, height=20, width=80)
    RdBtn_Left=tk.Radiobutton(FrameLaneParameter[i], text="Left ",variable=tk_timerStartRollDir[i], value=1).place(x=10,y=95, height=20, width=80)


    tk.Label(FrameLaneParameter[i],text="START Lane",font=("Arial", 12), fg='green').place(x=150,y=60, height=15, width=90)
    SliderStartNrLaneGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=3,variable=tk_timerStartNrLane[i],relief=tk.SOLID,orient='horizontal').place(x=160,y=75, height=40, width=70)

    tk.Label(FrameLaneParameter[i],text="START Area",font=("Arial", 12), fg='green').place(x=260,y=60, height=15, width=90)
    SliderStartArea[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=3,variable=tk_timerStartArea[i],relief=tk.SOLID,orient='horizontal').place(x=260,y=75, height=40, width=70)

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
        #the 7 days of the week as byte
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
                                '',''+str(myRobot.TimerstartLaneMaxlengh[i]),)
                                
    
    for i in range(5):
        myRobot.TimerstartArea[i]=tk_timerStartArea[i].get()  
        Send_reqSetting_message('Timer1','w',''+str(i)+'',''+str(myRobot.TimerstartArea[i])+\
                                '',''+str(myRobot.TimerdaysOfWeek[i])+\
                                '','0','0','0','0','0','0','0','0',)
    
    
    
    
def ButtonReadTimerFromDue_click():
    Send_reqSetting_message('Timer','r','0','0','0','0','0','0','0','0','0','0','0')

    
ButtonRequestTimerFomMower = tk.Button(TabTimer)
ButtonRequestTimerFomMower.place(x=10,y=400, height=25, width=150)
ButtonRequestTimerFomMower.configure(command = ButtonReadTimerFromDue_click)
ButtonRequestTimerFomMower.configure(text="Read Timer From Mower")


ButtonSetTimerApply = tk.Button(TabTimer)
ButtonSetTimerApply.place(x=300,y=400, height=25, width=150)
ButtonSetTimerApply.configure(command = ButtonSendTimerToDue_click,text="Send Timer To Mower")

ButtonBackHome = tk.Button(TabTimer, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=310, height=120, width=120)






 
""" THE MAIN PAGE ***************************************************"""


def ButtonPowerOff_click():
    returnval=messagebox.askyesno('Info',"Are you sure you want to shutdown all the PCB ?")
    if returnval :
        button_stop_all_click()
        send_pfo_message('rt','1','2','3','4','5','6',)
        

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

ButtonGps = tk.Button(MainPage, image=imgGps, command = ButtonGps_click)
ButtonGps.place(x=415,y=145,width=100, height=130)

ButtonRfid = tk.Button(MainPage, image=imgRfid, command = ButtonRfid_click)
ButtonRfid.place(x=550,y=145,width=100, height=130)



ButtonPowerOff = tk.Button(MainPage, image=imgPowerOff, command = ButtonPowerOff_click)
ButtonPowerOff.place(x=685,y=280,width=100, height=120)

Buttonimgardu=tk.Button(MainPage,image=imgArdumower,command = ButtonInfo_click)
Buttonimgardu.place(x=10,y=280,height=120,width=650)

Datetext = tk.Label(MainPage, text='',textvariable=tk_date_Now,font=("Arial", 20), fg='red')
Datetext.place(x=10,y=400, height=25, width=240)
Statustext = tk.Label(MainPage, text='',textvariable=tk_MainStatusLine,font=("Arial", 20), fg='red')
Statustext.place(x=240,y=400, height=25, width=400)

################## DESACTIVATE THE BT TO SAVE BATTERY
#subprocess.call(["rfkill","block","bluetooth"])
subprocess.Popen("sudo rfkill block bluetooth", shell=True)
subprocess.Popen("sudo iw wlan0 set power_save off", shell=True)
#subprocess.Popen("sudo rfkill block wifi", shell=True)
#sudo rfkill block wifi
#sudo rfkill unblock wifi
#sudo rfkill block bluetooth
#sudo rfkill unblock bluetooth

# use to drive the mower using the keyboard in manual
def check_keyboard(e) :   
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        if (e.char=="m"):  
            buttonBlade_start_click()            
        if (e.char=='q'):
            buttonBlade_stop_click()
              
        
def kbd_spaceKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        button_stop_all_click()    
def kbd_leftKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        ButtonLeft_click()
def kbd_rightKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        ButtonRight_click()
def kbd_upKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        send_var_message('w','motorSpeedMaxPwm',''+str(manualSpeedSlider.get())+'','0','0','0','0','0','0','0')
        send_pfo_message('nf','1','2','3','4','5','6',)
def kbd_downKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        ButtonStop_click()    

fen1.bind("<KeyPress>", check_keyboard)
fen1.bind('<Left>', kbd_leftKey)
fen1.bind('<Right>', kbd_rightKey)
fen1.bind('<Up>', kbd_upKey)
fen1.bind('<Down>', kbd_downKey)
fen1.bind('<space>', kbd_spaceKey)





checkSerial()
#on startup PI update Date time from Internet
#subprocess.Popen("sudo systemctl stop ntp.service", shell=True)
#subprocess.Popen("sudo systemctl disable ntp.service", shell=True)
#time.sleep(10)


consoleInsertText('Read Setting from PCB1.3'+ '\n')
read_all_setting()
consoleInsertText('Read Area In Mowing from PCB1.3'+ '\n')
send_req_message('PERI','1000','1','1','0','0','0',)
if(useMqtt):
    consoleInsertText('Wait update Date/Time from internet'+ '\n')
    consoleInsertText('Initial start MQTT after 10 secondes'+ '\n')
    mymower.timeToReconnectMqtt=time.time()+10

consoleInsertText('Control PI time and PCB1.3 time'+ '\n')
read_time_setting()
ButtonAuto_click()
BtnGpsRecordStop_click()

if (streamVideoOnPower):
    BtnStreamVideoStart_click()

fen1.mainloop()




            
            
        
