#!/usr/bin/env python

import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64
from tf2_msgs.msg import TFMessage
import tf
import tkinter
from tkinter import *

from gazebo_msgs.srv import SetPhysicsProperties
import gazebo_msgs.msg 
import geometry_msgs

from functools import partial

class interact:
    def __init__(self):
        
        self.set_gravity = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.set_gravity.wait_for_service()

        self.time_step = Float64(0.001)
        self.max_update_rate = Float64(1000.0)
        
        self.ode_config = gazebo_msgs.msg.ODEPhysics()
        self.ode_config.auto_disable_bodies = False
        self.ode_config.sor_pgs_precon_iters = 0
        self.ode_config.sor_pgs_iters = 50
        self.ode_config.sor_pgs_w = 1.3
        self.ode_config.sor_pgs_rms_error_tol = 0.0
        self.ode_config.contact_surface_layer = 0.001
        self.ode_config.contact_max_correcting_vel = 0.0
        self.ode_config.cfm = 0.0
        self.ode_config.erp = 0.2
        self.ode_config.contact_max_correcting_vel = 100 #100
        self.ode_config.max_contacts = 20


        self.gravity = geometry_msgs.msg.Vector3()
        self.gravity.x = 0.000
        self.gravity.y = 0.0
        self.gravity.z = -9.8
        
        # self.set_gravity(self.time_step.data, self.max_update_rate.data, self.gravity, self.ode_config)
    
    def gravity_up(self):
        self.gravity.x += 0.001
        self.set_gravity(self.time_step.data, self.max_update_rate.data, self.gravity, self.ode_config)
    

    def gravity_down(self):
        self.gravity.x -= 0.001
        self.set_gravity(self.time_step.data, self.max_update_rate.data, self.gravity, self.ode_config)

    def gravity_step(self):
        self.gravity.x += 0.1
        self.set_gravity(self.time_step.data, self.max_update_rate.data, self.gravity, self.ode_config)
        
        self.gravity.x -= 0.1
        self.set_gravity(self.time_step.data, self.max_update_rate.data, self.gravity, self.ode_config)

class interact2:
    def __init__(self):
        self.hit_on_cart = rospy.Publisher('/stand_cart_force_controller/command', Float64, queue_size=10)

        self.position_pub = rospy.Publisher("desired_pose", Float64, queue_size=10)
        self.k1_pub = rospy.Publisher("k1", Float64, queue_size=10)
        self.k2_pub = rospy.Publisher("k2", Float64, queue_size=10)
        self.k3_pub = rospy.Publisher("k3", Float64, queue_size=10)
        self.k4_pub = rospy.Publisher("k4", Float64, queue_size=10)
        
    def hit_cart_right(self):
        u = Float64()
        u.data = 10
        self.hit_on_cart.publish(u)
    
    def hit_cart_left(self):
        u = Float64()
        u.data = -10
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

if __name__=='__main__':

    rospy.init_node('interacter')
    
    root = Tk()

    rate = rospy.Rate(100)
    inter = interact2()

    position = StringVar()
    k1 = StringVar()
    k2 = StringVar()
    k3 = StringVar()
    k4 = StringVar()

    #row 1
    bon = Button(root, text = '>>', command = inter.hit_cart_right).grid(column = 2, row = 1)
    labelHit = Label(root, text="hit the cart 10N").grid(row=1, column=1)
    boff = Button(root, text = '<<', command = inter.hit_cart_left).grid(column = 0, row = 1)

    #row 2
    bon = Button(root, text = '>>', command = inter.hit_cart_right).grid(column = 2, row = 2)
    labelHit = Label(root, text="hit the cart 20N").grid(row=2, column=1)
    boff = Button(root, text = '<<', command = inter.hit_cart_left).grid(column = 0, row = 2)

    # row 3
    pose = partial(inter.podition_step, position)
    bostep = Button(root, text = 'step', command = pose).grid(column = 2, row = 3)
    labePose = Label(root, text="Pose").grid(row=3, column=0)
    entryPose = Entry(root, textvariable=position).grid(row=3, column=1)

    # row 4
    gain = partial(inter.gain_step, k1, k2, k3, k4)   
    labek1 = Label(root, text="k1").grid(row=4, column=0)
    entryk1 = Entry(root, textvariable=k1).grid(row=4, column=1)
    labek2 = Label(root, text="k2").grid(row=4, column=2)
    entryk2 = Entry(root, textvariable=k2).grid(row=4, column=3)
    labek3 = Label(root, text="k3").grid(row=4, column=4)
    entryk3 = Entry(root, textvariable=k3).grid(row=4, column=5)
    labek4 = Label(root, text="k4").grid(row=4, column=6)
    entryk4 = Entry(root, textvariable=k4).grid(row=4, column=7)
    bogain = Button(root, text = 'submit', command = gain).grid(column = 8, row = 3)

    while not rospy.is_shutdown():
        root.mainloop()