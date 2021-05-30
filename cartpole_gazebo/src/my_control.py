#!/usr/bin/env python

import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64
from tf2_msgs.msg import TFMessage
import tf

import numpy as np
import symbol

from gazebo_msgs.srv import SetPhysicsProperties
import gazebo_msgs.msg 
import geometry_msgs

from rospy.exceptions import ROSTimeMovedBackwardsException

import scipy

"""
dot_theta  = [     0       1 ]theta     + [      0     ] dot_p
ddot_theta   [ g/(L-mpl/M) 0 ]dot_theta   [(1/(M*(L-mp*l/M)))*((Km*Km*Kg*Kg)/(R*r*r))]
"""

class states:
    def __init__(self):
        self.angle = 0
        self.angle_vel = 0
        self.cart_position = 0
        self.cart_velocity = 0
        
        self.joint_subscriber = rospy.Subscriber("joint_states", JointState, self.joint_state_cb)
        self.desired_pose_subscriber = rospy.Subscriber("desired_pose", Float64, self.pose_cb)

        self.k1_subscriber = rospy.Subscriber("k1", Float64, self.k1_cb)
        self.k2_subscriber = rospy.Subscriber("k2", Float64, self.k2_cb)
        self.k3_subscriber = rospy.Subscriber("k3", Float64, self.k3_cb)
        self.k4_subscriber = rospy.Subscriber("k4", Float64, self.k4_cb)

        self.angle_pub = rospy.Publisher('angle', Float32, queue_size=10)
        self.angle_vel_pub = rospy.Publisher('angle_vel', Float32, queue_size=10)
        self.cart_pub = rospy.Publisher('position',Float32, queue_size=10)
        self.cart_vel_pub = rospy.Publisher('position_vel',Float32, queue_size=10)

        self.u_pub = rospy.Publisher("/stand_cart_force_controller/command", Float64, queue_size=10)

        self.mp = 0.23243 #pendulum mass
        self.mc = 0.11377 #pendulum mass
        self.M = self.mp + self.mc
        self.l = 0.25 #half pendulum lengh
        self.g = 9.8 #gravity
        
        self.I = 0.00651 #Rotational Inertia
        self.L= (self.I + self.mp*self.l*self.l)/self.mp*self.l
        self.R = 2.6 #Motor Armature Resistance
        self.r = 0.00635 #Motor Pinion Radius
        self.Km = 0.00767 #Motor Torque Constant
        self.Kg = 1 #Gear-box Ratio

        self.desired_pose = 0

        self.k1 = -15.3
        self.k2 = -0.56
        self.k3 = 0.5
        self.k4 = 0.5
        # self.g/(self.L-self.mp*self.l/self.M)
        # 1/self.M*(self.L-self.mp*self.l/self.M)*(self.Km*self.Km*self.Kg*self.Kg)/self.R*self.r*self.r
    
    
    def pose_cb(self, msg):
        self.desired_pose = msg.data
    
    def k1_cb(self, msg):
        self.k1 = msg.data
    
    def k2_cb(self, msg):
        self.k2 = msg.data
    
    def k3_cb(self, msg):
        self.k3 = msg.data
    
    def k4_cb(self, msg):
        self.k4 = msg.data
    
    
    def joint_state_cb(self, msg):
        if msg.position[1] != 0:
            self.cart_position = msg.position[1]

        if msg.position[0] != 0:
            self.angle = msg.position[0]
            
        if len(msg.velocity) > 0:
            self.angle_vel = msg.velocity[0]
            self.cart_velocity = msg.velocity[1]
            
    
    def K_compute(self):

        M = self.M
        Km = self.Km
        Kg = self.Kg
        r = self.r
        L = self.L

        #matrix A and B 
        A = np.array([  [ 0, 1 ], 
                        [self.g/( L- (self.mp*self.l/M) ), 0]])

        B = np.array([  [0], 
                        [(1/( M*(L-(self.mp*self.l/M) ) )) * ( (Km*Km*Kg*Kg)/(self.R*r*r) )]])

        desired_Poles = np.array([-0.2, -0.5])

        # print(scipy.signal.place_poles(A,B,desired_Poles))
        #ecuacion caracteristica deseada

    def states_publisher(self):
        self.angle_pub.publish(self.angle)
        self.angle_vel_pub.publish(self.angle_vel)
        self.cart_pub.publish(self.cart_position)
        self.cart_vel_pub.publish(self.cart_velocity)
    
    def control_action(self):
        # -15.3 -0.56
        # u = -15.3*self.angle - 0.56*self.angle_vel + 0.5*self.cart_velocity + 0.5*(self.cart_position-self.desired_pose)
        k1 = self.k1
        k2 = self.k2
        k3 = self.k3
        k4 = self.k4
        u = k1*self.angle + k2*self.angle_vel + k3*self.cart_velocity + k4*(self.cart_position-self.desired_pose)
        self.u_pub.publish(u)
        

if __name__=='__main__':

    rospy.init_node('controller')
    
    states = states()

    rate = rospy.Rate(100)
    states.K_compute()
    
    while not rospy.is_shutdown():
        try:
            states.states_publisher()
            states.control_action()
            rate.sleep()
        except ROSTimeMovedBackwardsException:
            print('reset simulation')
            
