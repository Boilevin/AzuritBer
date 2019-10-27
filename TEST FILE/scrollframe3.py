from tkinter import *

fen2 = Tk()
fen2.geometry("480x320")
"""
MainFrame=Frame(fen2)
MainFrame.place(x=0,y=0, height=320, width=300)
"""
def retour():
    MainFrame2.tkraise()
def retour1():
    MainFrame.tkraise()
    

MainFrame2 = Frame(fen2)
MainFrame2.place(x=0,y=0, height=320, width=480)
bt20= Button(MainFrame2, text='Bt20' ,width=10,height=1,command=retour1)
bt20.grid(row=0, column=0)


MainFrame = Frame(fen2)
MainFrame.place(x=0,y=0, height=320, width=480)

MainCanvas = Canvas(MainFrame, height=280,width=430)

MainCanvas.grid(row=0, column=0,sticky="nsew")
MainCanvasFrame = Frame(MainCanvas)
MainCanvas.create_window(0, 0, window=MainCanvasFrame, anchor='nw')
bt1= Button(MainCanvasFrame, text='Bt1' ,width=10,height=1,command=retour)
bt1.grid(row=0, column=0)
bt2= Button(MainCanvasFrame, text='Bt2' ,width=20,height=5)
bt2.grid(row=0, column=1)
bt3= Button(MainCanvasFrame, text='Bt3' ,width=20,height=5)
bt3.grid(row=1, column=0)
bt4= Button(MainCanvasFrame, text='Bt4' ,width=20,height=5)
bt4.grid(row=1, column=1)

bt1= Button(MainCanvasFrame, text='Bt5' ,width=20,height=5)
bt1.grid(row=2, column=0)
bt2= Button(MainCanvasFrame, text='Bt6' ,width=20,height=5)
bt2.grid(row=2, column=1)
bt3= Button(MainCanvasFrame, text='Bt7' ,width=20,height=5)
bt3.grid(row=3, column=0)
bt4= Button(MainCanvasFrame, text='Bt8' ,width=20,height=5)
bt4.grid(row=3, column=1)

bt1= Button(MainCanvasFrame, text='Bt9' ,width=20,height=5)
bt1.grid(row=4, column=0)
bt2= Button(MainCanvasFrame, text='Bt10' ,width=20,height=5)
bt2.grid(row=4, column=1)
bt3= Button(MainCanvasFrame, text='Bt11' ,width=20,height=5)
bt3.grid(row=5, column=0)
bt4= Button(MainCanvasFrame, text='Bt12' ,width=20,height=5,command=retour)
bt4.grid(row=5, column=1)


yscrollbar = Scrollbar(MainFrame, orient=VERTICAL,width=40)
yscrollbar.config(command=MainCanvas.yview)
MainCanvas.config(yscrollcommand=yscrollbar.set)
yscrollbar.grid(row=0, column=2, sticky="ns")

#MainCanvas.yview_moveto(0.5)

MainCanvasFrame.bind("<Configure>", lambda event: MainCanvas.configure(scrollregion=MainCanvas.bbox("all")))

fen2.mainloop()
