import Tkinter as tk
import ttk
import matplotlib

import pandas as pd
import urllib
import json
import numpy as np
import matplotlib.animation as animation

matplotlib.use("TkAgg")

from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2TkAgg
from matplotlib.figure import Figure




LARGE_FONT = ("Verdana",12)
style.use("ggplot")
class SeaofBTCapp(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        tk.Tk.wm_title(self,"Sea of BTC client")

        container = tk.Frame(self)

        container.pack(side="top",fill="both", expand = True)
        container.grid_rowconfigure(0,weight=1)
        container.grid_columnconfigure(0,weight=1)

        menubar =tk.Menu(container)
        fileemenu = tk.Menu(menubar,tearoff=0)

        self.frames = {}

        for F in (StartPage,PageOne,Pagetwo):

            frame = F(container,self)
            self.frames[F] = frame

            frame.grid(row=0,column= 0, sticky="nsew")
        self.show_frame(StartPage)

    def show_frame(self,count):
        frame = self.frames[count]
        frame.tkraise()

def qf(parm):
    print(parm)

class StartPage(tk.Frame):

    def __init__(self,parent,controler):
        tk.Frame.__init__(self,parent)
        label = ttk.Label(self,text="Start Page", font=LARGE_FONT)
        label.pack(pady = 10 , padx=10)
        button1 = ttk.Button(self,text="Disagree",
                            command=lambda :controler.show_frame(PageOne))
        button1.pack()

        button2 = ttk.Button(self, text="Agree",
                             command=lambda: controler.show_frame(Pagetwo))

        button2.pack()


class PageOne(tk.Frame):
    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)
        label = ttk.Label(self, text="second_page", font=LARGE_FONT)
        label.pack(pady=10, padx=10)
        button1 = ttk.Button(self, text="Go Home",
                            command=lambda: controller.show_frame(StartPage))
        button1.pack()

class Pagetwo(tk.Frame):

    def __init__(self,parent,controller):
        tk.Frame.__init__(self,parent)
        label = ttk.Label(self, text="Grap Page ", font=LARGE_FONT)
        label.pack(pady=10, padx=10)
        button1 = ttk.Button(self, text="Go Home",
                            command=lambda: controller.show_frame(StartPage))
        button1.pack()




        canvas = FigureCanvasTkAgg(f,self)
        canvas.show()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2TkAgg(canvas,self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)


f = Figure(figsize=(5, 5), dpi=100)
a = f.add_subplot(111)

def animation1(i):
    dataLink = 'https://btc-e.com/api/3/trades/btc_usd'

    pullData = open("sampleData.txt") .read()
    dataList = pullData.split('\n')
    xList = []
    yList = []
    for eachLine in dataList:
        x,y = eachLine.split(',')
        xList.append(int(x))
        yList.append(int(y))

    a.clear()
    a.plot(xList,yList)

app = SeaofBTCapp()
ani = animation.FuncAnimation(f,animation1,interval = 1000)
app.mainloop()

