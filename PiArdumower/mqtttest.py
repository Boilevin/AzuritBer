import tkinter as tk
from tkinter import *
import tkinter.filedialog
from tkinter import ttk
from time import sleep
################################################################################
### START OF MQTT

import paho.mqtt.client as paho
broker = "10.0.0.22"
port = 1883
Mq_C = paho.Client("RzBrain")
Mq_C.connect(broker,port)
Mq_C.loop_start()


def on_log(client, userdata, level, buf):
    print("Log: ",buf)

def on_publish(client, userdata, result):
    print("Data published \n")
    pass

def on_message(client,userdata, message):
    Topic = message.topic
    Raw_Msg = message
    try:          
        Msg = message.payload.decode("utf-8")
    except:
        print("Msg error on MQTT Read")
    if Topic == "RSV/TEMP":
        print(Msg)
    elif Topic == "RSV/HUMIDITY":
        print(Msg)
    elif Topic == "RSV/PRESSURE":
        print(Msg)
    elif Topic == "RSV/PHRASE":
        print("\n",Msg)


Mq_C.on_publish = on_publish
Mq_C.on_message = on_message

Mq_C.subscribe("RSV/TEMP")              #This data is being published on my network from another RPI
Mq_C.subscribe("RSV/HUMIDITY")      #This data is being published on my network from another RPI
Mq_C.subscribe("RSV/PRESSURE")    #This data is being published on my network from another RPI
Mq_C.subscribe("RSV/PHRASE")         # This is the return trip of a published topic / message


def Call_The_Function():
    print("Function called")
    Pay_Load = "Wuz Up"
    Mq_C.publish("RSV/PHRASE",Pay_Load)
    print("Message sent and moving on \n")

def Call_The_Print():
    print("Print Function called")
    print("Print moving on \n")


WinRoot = tk.Tk()
WinRoot.geometry('300x200+0+0')
WinRoot.title("TEST")
canvas = tk.Canvas(WinRoot, width=300, height=200, bg="gray30")
canvas.place(x=0,y=0)
MQTT_B = tk.Button(WinRoot,text="FUNC",command=Call_The_Function,bg="gray30",fg="red",height=2, width=7,highlightbackground="gray60",relief="raised",borderwidth=3)
MQTT_B.place(x=170,y=100)

Print_B = tk.Button(WinRoot,text="Print",command=Call_The_Print,bg="gray30",fg="red",height=2, width=7,highlightbackground="gray60",relief="raised",borderwidth=3)
Print_B.place(x=50,y=100)



WinRoot.mainloop()

