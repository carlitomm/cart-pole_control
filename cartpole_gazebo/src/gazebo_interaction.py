#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32, Float64
import tkinter
from tkinter import *
import tkinter.ttk

import threading
import geometry_msgs
import numpy as np
from functools import partial

from scipy import signal

class interact2:
    def __init__(self):
        self.hit_on_cart = rospy.Publisher('/stand_cart_force_controller/command', Float64, queue_size=10)

        self.position_pub = rospy.Publisher("desired_pose", Float64, queue_size=10)
        self.k1_pub = rospy.Publisher("k1", Float64, queue_size=10)
        self.k2_pub = rospy.Publisher("k2", Float64, queue_size=10)
        self.k3_pub = rospy.Publisher("k3", Float64, queue_size=10)
        self.k4_pub = rospy.Publisher("k4", Float64, queue_size=10)

        self.mp = 0.23243 #pendulum mass
        self.mc = 0.11377 #pendulum mass
        self.M = self.mp + self.mc
        self.l = 0.25 #half pendulum lengh
        self.g = 9.8 #gravity

        self.k3 = 15.3
        self.k4 = 0.56
        self.k2 = -0.8
        self.k1 = -0.5
        
    def hit_cart_right(self):
        u = Float64()
        u.data = 10
        self.hit_on_cart.publish(u)
    
    def hit_cart_left(self):
        u = Float64()
        u.data = -10
        self.hit_on_cart.publish(u)
    
    def hit_cart(self, stregnt):
        num1 = stregnt.get()
        u = Float64()
        try:
            u.data = float(num1)
            if float(num1) > 30 or float(num1) < -30:
                raise ValueError
        except:
            u.data = 0
        self.hit_on_cart.publish(u)
    
    def podition_step(self,n1):
        num1 = (n1.get())
        u = Float64()
        try:
            u.data = float(num1)
            if float(num1) > 0.3 or float(num1) < -0.3:
                raise ValueError
        except:
            u.data = 0
        self.position_pub.publish(u)
        
    
    def gain_step(self, k1, k2, k3, k4):
        num1 = (k1.get())
        num2 = (k2.get())
        num3 = (k3.get())
        num4 = (k4.get())
        u1 = Float64()
        u2 = Float64()
        u3 = Float64()
        u4 = Float64()
        try:
            u1.data = float(num1)
            u2.data = float(num2)
            u3.data = float(num3)
            u4.data = float(num4)
        except:
            u1.data = -15.3
            u2.data = -0.56
            u3.data = 0.5
            u4.data = 0.15
        self.k1_pub.publish(u1)
        self.k2_pub.publish(u2)
        self.k3_pub.publish(u3)
        self.k4_pub.publish(u4)
    
    def K_compute(self, d1, d2, d3, d4):

        num1 = (d1.get())
        num2 = (d2.get())
        num3 = (d3.get())
        num4 = (d4.get())
        try:
            num1 = complex(num1)
            num2 = complex(num2)
            num3 = complex(num3)
            num4 = complex(num4)
        except Exception as e:
            print(e)
            return
        
        M = self.M
        mp = self.mp
        mc = self.mc
        l = self.l
        g = self.g
        I=0.00438

        # A = np.array([ [ 0, 1, 0, 0], 
        #                 [0, 0, -(g*mp)/(mc), 0],
        #                 [0, 0, 0, 1],
        #                 [0, 0, g*M/(l*mp), 0] ])

        # B = np.array([  [0], 
        #                 [1/(mc)],
        #                 [0],
        #                 [-1/l*mc] ])
        
        p = I*(mc+mp)+mc*mp*l*l

        A = np.array([ [ 0, 1, 0, 0], 
                        [0, 0, (g*mp*mp*l*l)/( p ), 0],
                        [0, 0, 0, 1],
                        [0, 0, g*mp*l*(mp+mc)/( p ), 0] ])

        B = np.array([  [0], 
                        [I+mp*l*l/( p )],
                        [0],
                        [mp*l/( p )] ])

        # polos de lazo abierto
        # 0 + 178.3620i
        # 0 - 178.3620i
        # 0 +      0i
        # 0 +      0i

        desired_Poles = np.array([num1, num2, num3, num4])
        fs = signal.place_poles(A,B,desired_Poles)

        self.k1 = fs.gain_matrix[0,0]
        self.k2 = fs.gain_matrix[0,1]
        self.k3 = fs.gain_matrix[0,2]
        self.k4 = fs.gain_matrix[0,3]
        rospy.logwarn("pole Placement excecuted")

if __name__=='__main__':

    rospy.init_node('interacter')
    
    root = Tk()

    rate = rospy.Rate(1)
    inter = interact2()

    position = StringVar()
    k1 = StringVar()
    k2 = StringVar()
    k3 = StringVar()
    k4 = StringVar()
    d1 = StringVar()
    d2 = StringVar()
    d3 = StringVar()
    d4 = StringVar()
    hitStrenght = StringVar()

    gain = partial(inter.gain_step, k1, k2, k3, k4)
    hit = partial(inter.hit_cart, hitStrenght)
    pole_placement = partial(inter.K_compute, d1, d2, d3, d4)

    # row0
    labek1 = Label(root, text="Hit the cart").grid(row=0, column=0)
    entryStr = Entry(root, textvariable=hitStrenght)
    entryStr.grid(row=0, column=1)
    entryStr.insert(END, 'Newtons')
    bon = Button(root, text = 'hit it!!', command = hit).grid(column = 2, row = 0)
    
    # row 3
    pose = partial(inter.podition_step, position)
    bostep = Button(root, text = 'step', command = pose).grid(column = 2, row = 3)
    labePose = Label(root, text="Go to Pose").grid(row=3, column=0)
    entryPose = Entry(root, textvariable=position).grid(row=3, column=1)

    # row 4
    labek1 = Label(root, text="k1").grid(row=4, column=0)
    entryk1 = Entry(root, textvariable=k1)
    entryk1.grid(row=5, column=0)
    entryk1.insert(END, inter.k1)
    
    labek2 = Label(root, text="k2").grid(row=4, column=1)
    entryk2 = Entry(root, textvariable=k2)
    entryk2.grid(row=5, column=1)
    entryk2.insert(END, inter.k2)

    labek3 = Label(root, text="k3").grid(row=4, column=2)
    entryk3 = Entry(root, textvariable=k3)
    entryk3.grid(row=5, column=2)
    entryk3.insert(END, inter.k3)

    labek4 = Label(root, text="k4").grid(row=4, column=3)
    entryk4 = Entry(root, textvariable=k4)
    entryk4.grid(row=5, column=3)
    entryk4.insert(END, inter.k4)
    
    bogain = Button(root, text = 'submit', command = gain).grid(column = 4, row = 5)

    # row 6
    labek1 = Label(root, text="desired Poles").grid(row=6, column=4)
    ds1 = Entry(root, textvariable=d1)
    ds1.insert(END, -1.5-1.65j)
    ds1.grid(row=6, column=0)
    
    ds2 = Entry(root, textvariable=d2)
    ds2.insert(END, -1.5+1.65j)
    ds2.grid(row=6, column=1)

    ds3 = Entry(root, textvariable=d3)
    ds3.insert(END, -1.75)
    ds3.grid(row=6, column=2)

    ds4 = Entry(root, textvariable=d4)
    ds4.insert(END, -8.0)
    ds4.grid(row=6, column=3)


    k1lbl = Label(root, text=str(inter.k1), font=("Helvetica", 18))
    k1lbl.grid(column = 0, row = 7)
    k2lbl = Label(root, text=str(inter.k2), font=("Helvetica", 18))
    k2lbl.grid(column = 1, row = 7)
    k3lbl = Label(root, text=str(inter.k3), font=("Helvetica", 18))
    k3lbl.grid(column = 2, row = 7)
    k4lbl = Label(root, text=str(inter.k4), font=("Helvetica", 18))
    k4lbl.grid(column = 3, row = 7)

    boFSF = Button(root, text = 'Pole_placement', command = pole_placement)
    
    boFSF.grid(column = 4, row = 7)

    def iniciar():
        while(True):
            k1lbl.configure(text=str(round(inter.k1,3)))
            k2lbl.configure(text=str(round(inter.k2,3)))
            k3lbl.configure(text=str(round(inter.k3,3)))
            k4lbl.configure(text=str(round(inter.k4,3)))
            time.sleep(1)

    thread1 = threading.Thread(target=iniciar)
    thread1.start()

    while not rospy.is_shutdown():
        root.mainloop()