#!/usr/bin/env python3
import shlex
import subprocess
import tkinter
from tkinter import messagebox
from tkinter import filedialog
import tkinter as tk
import os
import time
import sys




class Shell_Command_class(object):
    """class use to send command to the shell"""
    def __init__(self):
        self.Shell_Command = None

    def send(self,Command_line):
        print(Command_line)
        args = shlex.split(Command_line)
        print(args)
        self.Shell_Command=subprocess.call(args)
        
myShell= Shell_Command_class()

fileName='/home/pi/Documents/PiArdumower/Due_firmware/ardumower.ino.bin'
actualRep=os.getcwd()
    
def ButtonFlash_click():
    ButtonFlash.destroy()
    fen1.update()
    if os.path.exists('/dev/ttyACM0') == True:
        print("put the DUE into flash mode on ttyACM0")
        myShell.send('stty -F /dev/ttyACM0 1200')
        time.sleep(1)
        print("Flash the DUE")
        print()
        myShell.send('/home/pi/.arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac -i -p /ttyACM0 -R -e -w -b -v '+str(tk_fileName.get())+'')
        time.sleep(1)
        print("restart the DUE")
        myShell.send('stty -F /dev/ttyACM0 115200')
        time.sleep(3)
        os.remove(str(tk_fileName.get()))
        fen1.destroy()
    if os.path.exists('/dev/ttyACM1') == True:
        print("put the DUE into flash mode on ttyACM1")
        myShell.send('stty -F /dev/ttyACM1 1200')
        time.sleep(1)
        print("Flash the DUE")
        print()
        #/home/pi/.arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac -i -d --port=ttyACM0 -U true -e -w -v -b /tmp/arduino_build_705225/Blink.ino.bin -R
        myShell.send('/home/pi/.arduino15/packages/arduino/tools/bossac/1.6.1-arduino/bossac -i -p /ttyACM1 -R -e -w -b -v '+str(tk_fileName.get())+'')
        #myShell.send('bossac -i -p /ttyACM0 -R -e -w -b -v ardumower.ino.bin')
        time.sleep(1)
        print("restart the DUE")
        myShell.send('stty -F /dev/ttyACM1 115200')
        time.sleep(3)
        os.remove(str(tk_fileName.get()))
        fen1.destroy()
    
    
def ButtonReadFromFile_click():
    global fileName
    settingFileName = filedialog.askopenfilename(title="Open :", initialdir=actualRep,initialfile='ardumower.ino.bin', filetypes = [("All", "*"),("File Setting","*.ini")])    
    
    if len(settingFileName) > 0:
        fileName=str(settingFileName)
        tk_fileName.set(str(fileName))

    else:
        fileName='ardumower.ino.bin'
        tk_fileName.set(str(fileName))

        
          
  
fen1 =tk.Tk()
fen1.title('DUE FLASHER')
fen1.geometry("600x120+10+150")

tk_fileName=tk.StringVar()
tk_fileName.set(str(fileName))
LabelFilename=tk.Label(fen1, text='File:',fg='green').place(x=10,y=0)
TextFilename = tk.Entry(fen1,textvariable=tk_fileName)

TextFilename.place(x=10,y=15, height=20, width=500)
ButtonReadFromFile= tk.Button(fen1,text='...')
ButtonReadFromFile.place(x=510,y=15, height=20, width=40)
ButtonReadFromFile.configure(command = ButtonReadFromFile_click)

ButtonFlash = tk.Button(fen1,text='Start Flash')
ButtonFlash .place(x=240,y=60, height=40, width=100)
ButtonFlash .configure(command = ButtonFlash_click)

  
fen1.mainloop()
