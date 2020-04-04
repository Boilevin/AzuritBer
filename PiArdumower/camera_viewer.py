#!/usr/bin/env python3
import tkinter as tk
import picamera

fen1 = tk.Tk()
fen1.title('CAMERA')
fen1.geometry("400x300")

camera=picamera.PiCamera()
camera.start_preview()
def start_camera(e) :
    camera.start_preview()
def stop_camera(e) :
    camera.stop_preview()

def kbd_spaceKey(e) :
    stop_camera(e)
def kbd_enterKey(e) :
    start_camera(e) 
fen1.bind('<space>', kbd_spaceKey)
fen1.bind('<Return>', kbd_enterKey)

def ButtonStart_click():
    start_camera(0)
def ButtonStop_click():
    stop_camera(0)
frame = tk.Frame(fen1, width=360, height=260)
frame.pack()
ButtonStart = tk.Button(frame, command = ButtonStart_click,text="START")
ButtonStart.pack()
ButtonStop = tk.Button(frame, command = ButtonStop_click,text="STOP")
ButtonStop.pack()

frame.focus_set()
fen1.mainloop()
