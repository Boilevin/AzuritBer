import os

cwd = os.getcwd()


if (os.name=='nt'):
    print('Windows Platform')
    myComPort = 'COM9'
    myFrameWidth = 800
    myFrameHeight = 430
    myBaudRate = 115200
    myOS="Windows"
    GpsConnectedOnPi=False
    GpsIsM6n=True
    NanoConnectedOnPi=False
    DueConnectedOnPi=False
    AutoRecordBatCharging=False
    useDebugConsole=True

if (os.name=='posix'):
    print('Linux Platform')
    #myComPort = '/dev/ttyACM0'
    myFrameWidth = 800
    myFrameHeight = 430
    #myBaudRate = 250000
    myOS="Linux"
    GpsConnectedOnPi=False
    GpsIsM6n=False
    NanoConnectedOnPi=False
    DueConnectedOnPi=True
    AutoRecordBatCharging=True
    useDebugConsole=True
    myCamAngle = 0 # Possible= 0, 90, 180, 270 
    

