#!/usr/bin/env python3

import sys
import serial
#import pynmea2
import time
#import numpy as np
import subprocess
import pickle

import os
from tkinter.ttk import Notebook
from tkinter import messagebox
from tkinter import filedialog
from config import cwd
from config import myOS
import tkinter as tk
import math


#sys.path.insert(0, "/home/pi/Documents/PiArdumower") #add full path to avoid KST plot error on path

class streamVideo_class(object):
    """class use to start and stop the video stream"""
    def __init__(self):
        self.streamVideo = None

    def start(self,resolution):
        self.stop()
        print(resolution)
        if resolution==0:
            self.streamVideo=subprocess.Popen([cwd+"/streamVideo320.py","shell=True","stdout=subprocess.PIPE"])
        if resolution==1:
            self.streamVideo=subprocess.Popen([cwd+"/streamVideo640.py","shell=True","stdout=subprocess.PIPE"])
        
    def stop(self):
        if self.streamVideo:
            self.streamVideo.kill()
            self.streamVideo.wait()
            self.streamVideo = None
            
myStreamVideo=streamVideo_class()




#actualRep=os.getcwd()
dateNow=time.strftime('%d/%m/%y %H:%M:%S',time.localtime())
ManualSpeedL=0
ManualSpeedR=0
direction_list=['LEFT','RIGHT']
days_list=['SUNDAY','MONDAY','TUESDAY','WEDNESDAY','THURSDAY','FRIDAY','SATURDAY']

fen1 =tk.Tk()

"""variable use into Auto Menu"""
tk_batVoltage=tk.DoubleVar()
#tk_ImuYaw=tk.DoubleVar()
#tk_ImuPitch=tk.DoubleVar()
#tk_ImuRoll=tk.DoubleVar()
tk_Dht22Temp=tk.DoubleVar()
#tk_Dht22Humid=tk.DoubleVar()


CamVar1=tk.IntVar()
receivedList=[0]

fen1.title('ARDUMOWER')
fen1.geometry("800x480")


def MainLoop():  #*******************************the main loop is here
    
    byteResponse=ser.readline()
    
    if str(byteResponse)!="b''": #receive OK
        strResponse=byteResponse.decode('UTF-8') #change byte to string
        #print(strResponse)
        if strResponse[:1] != '$' : # the first digit is not $ so nothing to do except terminal print
            txtConsoleRecu.insert('1.0', strResponse)
            
        
        if strResponse[:1] == '$': # the first digit is $ so something to do
            txtRecu.insert('1.0', strResponse)
            #receivedList.clear()
            strResponse=strResponse.strip()
            receivedList=strResponse.split(",")
            
            #print(strResponse)
            #print(receivedList)
            if receivedList[0]=='$temp':
                tk_Dht22Temp.set('{:04.2f}'.format(float(receivedList[1])))
                
            if receivedList[0]=='$time':

                #$time,12,21,6,28,7,2018 what is receive from due
                cmdline="sudo date -s " + "'" + receivedList[6] + "-"
                cmdline=cmdline + receivedList[5] + "-" + receivedList[4] +" "
                cmdline=cmdline + receivedList[1] +":" + receivedList[2] + ":"
                cmdline=cmdline + receivedList[3] +"'"
            
                print (cmdline)
                if myOS == "Linux":
                    print("Set the new time and date to PI" )
                    os.system(cmdline)

            


            if receivedList[0]=='$PwrOff':
                print("recu OFF")
                ConsolePage.tkraise()
                #text1.config(text="Start to save all Data")
                txtConsoleRecu.insert('1.0', 'Start to save all Console Data')
                ButtonSaveReceived_click()  #save the console txt
                txtConsoleRecu.insert('1.0', 'All Console Data are saved')
                print("All Console Data are saved")
                txtConsoleRecu.insert('1.0', 'Start to stop the GPS Record')
                #mygpsRecord.stop()
                txtConsoleRecu.insert('1.0', 'The GPS Record is stopped')
                print("The GPS Record is stopped")
                #text1.config(text="Start to shutdown")
                txtConsoleRecu.insert('1.0', 'PI start Shutdown in 1 Second')
                time.sleep(1)
                print("Start subprocess /home/pi/Documents/PiArdumower/PowerOffNow.py")
                if myOS == "Linux":
                    pass
                    #subprocess.Popen('/home/pi/Documents/PiArdumower/PowerOffNow.py')
                fen1.destroy()
                time.sleep(1)
                print("Fen1 is destroy")
                #sys.exit("PowerOFF")












            
            
            if receivedList[0]=='$loop':
                MainStatusLine.set(receivedList[1])
                pass
                
            if receivedList[0]=='$batV':
                tk_batVoltage.set('{:04.2f}'.format(float(receivedList[1])))
                
            if receivedList[0]=='$GNRMC':
                gpsData = parse_GPRMC(str(line)) # Turn a GPRMC sentence into a Python dictionary called gpsData
                if gpsData['validity'] == "A": # If the sentence shows that there's a fix, then we can log the line
                    if firstFixFlag is False: # If we haven't found a fix before, then set the filename prefix with GPS date & time.
                        firstFixDate = gpsData['fix_date'] + "-" + gpsData['fix_time']
                        firstFixFlag = True
                
                    else: # write the data to a simple log file and then the raw data as well:
                        with open(cwd+"/gpsdata/" + firstFixDate +".txt", "a") as myfile:
                            myfile.write(gpsData['fix_date'] + "," + gpsData['fix_time'] + "," + str(gpsData['decimal_latitude']) + "," + str(gpsData['decimal_longitude']) +"\n")
                            #with open("/home/pi/gps_experimentation/" + firstFixDate +"-gprmc-raw-log.txt", "a") as myfile:
                            #myfile.write(line +"\n")
                   
 
    fen1.after(10,MainLoop)  # 10 here is the main loop each 10ms

    
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

    txtConsoleRecu.insert('1.0', 'All Console file are saved')
    

    
def ButtonForward_click():
    ManualSpeedL=manualSpeedSlider.get()
    ManualSpeedR=manualSpeedSlider.get()
    send_terminal_message('pc.cm,500,500,'+str(ManualSpeedL)+','+str(ManualSpeedR)+'')
def ButtonRight_click():
    ManualSpeedR=20
    ManualSpeedL=manualSpeedSlider.get()
    #to brake the inner wheel faster only 5 CM to drive
    send_terminal_message('pc.cm,500,5,'+str(ManualSpeedL)+','+str(ManualSpeedR)+'')
def ButtonLeft_click():
    ManualSpeedL=20
    ManualSpeedR=manualSpeedSlider.get()
    #to brake the inner wheel faster only 5 CM to drive
    send_terminal_message('pc.cm,5,500,'+str(ManualSpeedL)+','+str(ManualSpeedR)+'')
def ButtonReverse_click():
    ManualSpeedL=manualSpeedSlider.get()
    ManualSpeedR=manualSpeedSlider.get()
    send_terminal_message('pc.cm,-500,-500,-'+str(ManualSpeedL)+',-'+str(ManualSpeedR)+'')

def ButtonStop_click():
    send_terminal_message('pc.s')
     

def button_home_click():
    send_terminal_message('gohome')
    
def button_track_click():
    send_terminal_message('tpt')

def button_stop_all_click():
    send_terminal_message('M')


    
def ButtonCamera_click():
    StreamVideoPage.tkraise()
    
def ButtonGps_click():
    GpsPage.tkraise()

def ButtonSchedule_click():
    TabTimer.tkraise()

    
def ButtonPowerOff_click():
    returnval=messagebox.askyesno('Info',"Are you sure you want to shutdown all the PCB ?")
    if returnval :
        send_terminal_message('poweroff')

    
def ButtonStartMow_click():
    send_terminal_message('A')
   
def buttonBlade_stop_click():
    send_terminal_message('t')
    
def buttonBlade_start_click():
    send_terminal_message('z')

def ButtonManual_click():
    send_terminal_message('M')
    ManualPage.tkraise()
        
def ButtonPlot_click():
    TabPlot.tkraise()

def ButtonSetting_click():
    TabSetting.tkraise()
   
def ButtonBackToMain_click():
    MainPage.tkraise()

def ButtonBackHome_click():
    MainPage.tkraise()
                    
def ButtonConsole_click():
    ConsolePage.tkraise()

def ButtonTest_click():
    TestPage.tkraise()
    
def ButtonAuto_click():
    AutoPage.tkraise()
    
   

def ButtonOdo1TurnFw_click():
    send_terminal_message('h')
def ButtonOdo5TurnFw_click():
    send_terminal_message('M')
def ButtonOdo1TurnRev_click():
    send_terminal_message('A')
def ButtonOdo5TurnRev_click():
    send_terminal_message('reset')
def ButtonOdo3MlFw_click():
    send_terminal_message('Batery.show') 
def ButtonOdoRot180_click():
    send_terminal_message('per.show')
def ButtonOdoRot360_click():
    send_terminal_message('temp.show')
def ButtonOdoRotNonStop_click():
    send_terminal_message('bht.tri')
def BtnReset_click():
    send_terminal_message('reset')
def BtnHelp_click():
    send_terminal_message('H')
def BtnManual_click():
    send_terminal_message('M')
def Btn111_click():
    send_terminal_message('temp.show')
def Btn112_click():
    send_terminal_message('set.cco,2,1,65000')
    send_terminal_message('set.cco,3,1,65000')
    
    
   
        
def send_terminal_message(val1):
    message = val1
    message=str(message)
    message=message + '\r' +'\n'
    try:
            MainLoop()
            ser.flushOutput() #clear the output buffer
            ser.write(bytes(message,'utf-8'))
            #print("Send Message :" , message)
            txtSend.insert('1.0',  message) 
            
    except :
            #pass
            print("ERREUR while transfert")
            #time.sleep(2)


#Initialise the serial line response is normaly 'b'
try:
        if myOS == "Linux":
            ser = serial.Serial('/dev/ttyACM0',115200,timeout=0)
        else:
            ser = serial.Serial('COM9',115200,timeout=0)
            
        byteResponse=ser.readline()
        print(str(byteResponse))
        
        
except:
        print("ERREUR DE CONNECTION")
        time.sleep(2)
        #sys.exit("Impossible de continuer")




#*************************************************************************************************
#******************************* THE AUTO PAGE ***************************************************

imgHome=tk.PhotoImage(file=cwd + "/icons/home.png")
imgTrack=tk.PhotoImage(file=cwd + "/icons/track.png")
imgStopAll=tk.PhotoImage(file=cwd + "/icons/stop all.png")
imgstartMow=tk.PhotoImage(file=cwd + "/icons/startmow.png")
imgBack=tk.PhotoImage(file=cwd + "/icons/back1.png")

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



ButtonStartMow = tk.Button(AutoPage, image=imgstartMow, command=ButtonStartMow_click())
ButtonStartMow.place(x=130,y=0,width=100, height=130)
Buttonhome = tk.Button(AutoPage, image=imgHome, command =  button_home_click())
Buttonhome.place(x=250,y=0,width=100, height=130)
Buttontrack = tk.Button(AutoPage, image=imgTrack, command =  button_track_click())
Buttontrack.place(x=380,y=0,width=100, height=130)
ButtonStopAllAuto = tk.Button(AutoPage, image=imgStopAll, command = button_stop_all_click())
ButtonStopAllAuto.place(x=500,y=0,width=100, height=130)
ButtonBackHome = tk.Button(AutoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=310, height=120, width=120)


#*****************************************************************************************
#**************************THE MANUAL PAGE  **********************************************
ManualPage = tk.Frame(fen1)
ManualPage.place(x=0, y=0, height=430, width=800)
Frame1 = tk.Frame(ManualPage)
Frame1.place(x=0, y=0, height=300, width=300)

imgBladeStop = tk.PhotoImage(file=cwd + "/icons/bladeoff.png")
imgBladeStart = tk.PhotoImage(file=cwd + "/icons/bladeon.png")
imgForward=tk.PhotoImage(file=cwd + "/icons/forward.png")
imgReverse=tk.PhotoImage(file=cwd + "/icons/reverse.png")
imgLeft=tk.PhotoImage(file=cwd + "/icons/left.png")
imgRight=tk.PhotoImage(file=cwd + "/icons/right.png")
imgStop=tk.PhotoImage(file=cwd + "/icons/stop.png")



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


manualSpeedSlider = tk.Scale(ManualPage,orient='horizontal', from_=50, to=255)
manualSpeedSlider.place(x=0,y=300,width=300, height=80)
tk.Label(ManualPage,text='speed', fg='green').place(x=120,y=340)

ButtonBackHome = tk.Button(ManualPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=310, height=120, width=120)



#*********************************************************************************************
#************************************** The Console page  ************************************

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
BtnReset= tk.Button(Frame2,command = BtnReset_click,text="Reset")
BtnReset.place(x=0,y=0, height=25, width=60)
BtnHelp= tk.Button(Frame2,command = BtnHelp_click,text="Help")
BtnHelp.place(x=65,y=0, height=25, width=60)
BtnManual= tk.Button(Frame2,command = BtnManual_click,text="Manual")
BtnManual.place(x=130,y=0, height=25, width=60)
Btn111= tk.Button(Frame2,command = Btn111_click,text="111")
Btn111.place(x=195,y=0, height=25, width=60)
Btn112= tk.Button(Frame2,command = Btn112_click,text="112")
Btn112.place(x=260,y=0, height=25, width=60)

ButtonSaveReceived = tk.Button(ConsolePage)
ButtonSaveReceived .place(x=600,y=5, height=25, width=130)
ButtonSaveReceived .configure(command = ButtonSaveReceived_click,text="Save To File")

ButtonBackHome = tk.Button(ConsolePage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=310, height=120, width=120)


#**********************************************************************************************************
#****************************************** THE GPS PAGE **************************************************


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
    send_terminal_message('set.cco,4,0,65000') #gps due factory rate for 65000 data

        
def BtnGpsRecordStop_click():
    BtnGpsRecordStart.configure(state='normal')
    BtnGpsRecordStop.configure(state='disabled')
    send_terminal_message('set.cco,4,0,0') #gps due factory rate for 0 data so stop
        
GpsPage =tk.Frame(fen1)
GpsPage.place(x=0, y=0, height=430, width=800)
FrameGps = tk.Frame(GpsPage,borderwidth="1",relief=tk.SOLID)
FrameGps.place(x=20, y=40, width=600, height=300)
tk.Label(GpsPage, text='The gps run as service and write into directory /gpsdata/').place(x=10,y=15)
BtnGpsRecordStart= tk.Button(FrameGps,command = BtnGpsRecordStart_click,text="Start")
BtnGpsRecordStart.place(x=80,y=50, height=25, width=60)
BtnGpsRecordStop= tk.Button(FrameGps,command = BtnGpsRecordStop_click,text="Stop")
BtnGpsRecordStop.place(x=10,y=50, height=25, width=60)

ButtonBackHome = tk.Button(GpsPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=310, height=120, width=120)



#*******************************************************************************************************************
#*********************************************** THE CAMERA PAGE ***************************************************
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

ButtonBackHome = tk.Button(StreamVideoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=310, height=120, width=120)
        
#**********************************************************************************************************************        
#**************************************************** THE TEST PAGE ***************************************************

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




ButtonBackHome = tk.Button(TabTimer, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=310, height=120, width=120)











#****************************************************************************************************
#********************************** THE MAIN PAGE ***************************************************
       
imgArdumower = tk.PhotoImage(file=cwd + "/icons/ardumower.png")
imgManual=tk.PhotoImage(file=cwd + "/icons/manual.png")
imgAuto=tk.PhotoImage(file=cwd + "/icons/auto.png")
#imgTest=tk.PhotoImage(file=cwd + "/icons/test.png")
imgConsole=tk.PhotoImage(file=cwd + "/icons/console.png")
#imgSetting=tk.PhotoImage(file=cwd + "/icons/setting.png")
imgPowerOff=tk.PhotoImage(file=cwd + "/icons/off.png")
#imgPlot=tk.PhotoImage(file=cwd + "/icons/plot.png")
imgSchedule=tk.PhotoImage(file=cwd + "/icons/schedule.png")
imgCamera=tk.PhotoImage(file=cwd + "/icons/camera.png")
imgGps=tk.PhotoImage(file=cwd + "/icons/gps.png")


MainPage = tk.Frame(fen1)
MainPage.place(x=0, y=0, height=480, width=800)

ButtonAuto = tk.Button(MainPage,image=imgAuto,command = ButtonAuto_click)
ButtonAuto.place(x=10,y=10, height=130, width=100)

ButtonManual = tk.Button(MainPage,image=imgManual)
ButtonManual.place(x=145,y=10, height=130, width=100)
ButtonManual.configure(command = ButtonManual_click)

ButtonCamera = tk.Button(MainPage, image=imgCamera, command = ButtonCamera_click)
ButtonCamera.place(x=280,y=10,width=100, height=130)

ButtonConsole = tk.Button(MainPage,image=imgConsole)
ButtonConsole.place(x=415,y=10, height=130, width=100)
ButtonConsole.configure(command = ButtonConsole_click)

ButtonGps = tk.Button(MainPage, image=imgGps, command = ButtonGps_click)
ButtonGps.place(x=550,y=10,width=100, height=130)

ButtonSchedule = tk.Button(MainPage, image=imgSchedule, command = ButtonSchedule_click)
ButtonSchedule.place(x=145,y=145,width=100, height=130)

ButtonPowerOff = tk.Button(MainPage, image=imgPowerOff, command = ButtonPowerOff_click)
ButtonPowerOff.place(x=685,y=300,width=100, height=130)


MainStatusLine=tk.StringVar()

Maintext = tk.Label(MainPage, text='',textvariable=MainStatusLine,font=("Arial", 20), fg='red')
Maintext.place(x=10,y=450, height=25, width=300)



MainLoop()
if myOS == "Linux":
               print("Read the time from due to setup the PI one" )
               send_terminal_message('set.cco,2,5,65000') #battery each 5*2 second for 65000 data
               send_terminal_message('set.cco,3,10,65000') #temperature each 5*4 second for 65000 data
               send_terminal_message('set.cco,4,0,0') #gps due factory rate for 0 data so stop
               send_terminal_message('rtc.show')
#read_all_setting()
#read_time_setting()

fen1.mainloop()




            
            
        
