#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Copyright (c) 2023 Aniket Datar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rospy
import tf
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
import sys
import os
import time
from threading import Lock
import math
import numpy as np

class twistControl(object):
    def __init__(self):
        #Safety Arming Variables:
        self.armed = False
        self.automode = False 
        self.joytime = time.time()
        self.joytimeout = 3
        self.navtime = time.time()
        self.navTimeout = 2
        
        #throtle and steering multiplier
        self.thr_mul = 0.95
        self.str_mul = 1

        #servo variables
        self.throttleInitialized = False
        self.steeringAngle = 0.5
        self.throttle = 0.5
        self.trim = 0.0
        self.gear = 1.0
        self.fTLock = 1.0
        self.rTLock = 0.0
        self.camangle = 0.0

        #Control Command to Arduino (format dataindex: fuction)
        #[0:steering 1:throttle 2:arm-Disarm 3:gear 4:frontDiffLock 5:RearDiffLock 6:cameraServo]
        self.control_cmd = Float32MultiArray( data=[ 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.28 ] )  
        
        #Camera Pose variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.set_pitch = 0.0
        self.def_pitch = 0.0
        
        #camera stabilization PID values
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        self.prv_ae = 0
        self.ang_integral_error = 0
        self.out_ang = 0
        
        print("cmd_control")

        rospy.Subscriber("/cmd_vel", Twist, self.navStackCB, queue_size=1)
        rospy.Subscriber("/campose", Float32MultiArray, self.camCB, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joyControlCB, queue_size=1)
        self.vel_pub1 = rospy.Publisher("/cmd_vel1", Float32MultiArray, queue_size=1)
        rospy.init_node("cmd_control", anonymous=True)
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            self.cmdRouter()
            rate.sleep()

    def navStackCB(self, navCmd):
        try:
            #self.nav_cmd = navCmd.twist
            if self.automode and self.armed:
                #Throttle command from navigation stack
                self.control_cmd.data[1] = navCmd.linear.x * self.thr_mul
                #Steering command from navigation stack
                self.control_cmd.data[0] = navCmd.angular.z * self.str_mul
                self.navtime = time.time()
        except (Exception, b):
            print(b)
	
    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

    def camCB(self, cam):
        try:
            self.roll = cam.data[0]
            self.pitch = cam.data[1]
            self.yaw = cam.data[2]
            self.p = cam.data[3]
            self.i = cam.data[4]
            self.d = cam.data[5]
            k = cam.data[6]

            ang_error = self.set_pitch - self.pitch
            if(abs(ang_error) < 0.2):
                ang_error = 0

            self.ang_integral_error += ang_error
            self.ang_integral_error = min(100, max(-15, self.ang_integral_error ))
            ang_derivative_error = self.prv_ae - ang_error

            self.prv_ae = ang_error

            self.out_ang = self.out_ang + (self.p * ang_error ) + (self.i * (self.ang_integral_error)) - (self.d * ang_derivative_error)
            self.out_ang = min(28, max(-5, self.out_ang ))

            out = self.translate(self.out_ang, -5, 28, 0, 1)
            print(out)
            #ae_inc = self.translate(ae_inc, -74, 74, 0, 1)
            self.control_cmd.data[6] = out

            if self.control_cmd.data[6] > 1.0:
                self.control_cmd.data[6]= 1.0
                #pass
            if self.control_cmd.data[6] < 0.0:
                self.control_cmd.data[6] = 0.0
                #pass
        except (Exception, b):
            print(b)

    def joyControlCB(self, joy):
        try:
            #Arming Disarming
            self.joystick = joy
            #print(self.joystick)
            
            if not self.throttleInitialized:
                if self.joystick.buttons[9] == 1 and self.joystick.axes[5] == 1.0  and self.joystick.axes[2] == -1:
                    self.throttleInitialized = True   
                    print("Vehicle Armed")

            else:
                if self.joystick.buttons[8] == 1 and self.joystick.axes[5] == 1.0  and self.joystick.axes[2] == -1:
                    
                    self.throttleInitialized = False
                    self.armed = False
                    self.automode = False 
                    self.control_cmd.data = [ 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.28 ]
                    print("Vehicle Disarmed")

            #Joy control loop
            if self.throttleInitialized:

                if not self.armed:
                    #Steering Control
                    self.steeringAngle = self.trim + (-self.joystick.axes[0])
                    self.control_cmd.data[0] = self.steeringAngle
                    #Steering Trims
                    if self.joystick.axes[6] > 0:
                        self.trim = self.trim + 0.001
                    if self.joystick.axes[6] < 0:
                        self.trim = self.trim - 0.001
                
                
                    #Throttle Control forward
                    if self.joystick.axes[4] == 0:
                        self.throttle = ((1 - self.joystick.axes[5])/2)*((1 - self.joystick.axes[5])/2)
                        self.control_cmd.data[1] = self.throttle
                
                    #throttle Control reverse    
                    if self.joystick.axes[4] == -1:
                        self.throttle = -((1 - self.joystick.axes[5])/2)*((1 - self.joystick.axes[5])/2)
                        self.control_cmd.data[1] = self.throttle  
                

                #Gear Hi-Low
                if self.joystick.buttons[4] == 1:
                    self.gear = 1.0
                if self.joystick.buttons[5] == 1:
                    self.gear = 0.0
                
                #Front and Rear Diff Lock
                if self.joystick.buttons[0] == 1:
                    self.fTLock = 1.0
                if self.joystick.buttons[3] == 1:
                    self.fTLock = 0.0
                if self.joystick.buttons[2] == 1:
                    self.rTLock = 1.0
                if self.joystick.buttons[1] == 1:
                    self.rTLock = 0.0

                #Mode Control and Deadman switch
                if self.joystick.buttons[11] == 1 and (self.joystick.axes[5] == 1.0 or self.automode):
                    self.automode = not self.automode
                    if self.automode:
                        self.control_cmd.data[2] = 1.0
                        self.navtime = time.time()
                        print("Auto Mode")
                    else:
                        self.control_cmd.data[2] = 0.0
                        print("Joy Mode")
                        self.armed = False
                
                if self.automode or self.armed:
                    if self.joystick.axes[2] == -1:
                        self.armed = True
                        #print("Armed")
                    else:
                        self.armed = False
                        #print("disarmed")
                
                if not self.automode:
                    self.armed = False

                #Camera angle control
                self.camangle = self.camangle + self.joystick.axes[7]
                if self.joystick.buttons[10] == 1:
                    self.camangle = self.def_pitch
                    print("Camera angle: " + str(self.camangle) )
                    
                
                self.control_cmd.data[3] = self.gear
                self.control_cmd.data[4] = self.fTLock
                self.control_cmd.data[5] = self.rTLock
                self.set_pitch = self.camangle

            else:
                pass

            self.joytime = time.time() 
        except (Exception, f):
            print(f)

    def cmdRouter(self):


        #if Armed then pass the current control
        if self.throttleInitialized:
            #if not in automode pass the Joystick message 
            if not self.automode:
                if time.time() - self.joytime < self.joytimeout:
                    self.vel_pub1.publish(self.control_cmd)
                else:
                    self.control_cmd.data[0] = 0.0
                    self.control_cmd.data[1] = 0.0
                    self.vel_pub1.publish(self.control_cmd)

            #if in auto mode and deadman switch is active pass the command value
            if self.automode and self.armed:
                if time.time() - self.navtime < self.navTimeout:
                    self.vel_pub1.publish(self.control_cmd)
                else:
                    self.automode = False
                    self.armed = False
                    self.control_cmd.data[2] == 0.0
            
            #keep the throttle to recent joy input or zero 
            else:
                if time.time() - self.joytime < self.joytimeout:
                    self.vel_pub1.publish(self.control_cmd)
                else:
                    if self.automode:
                        self.control_cmd.data[0] = 0.0
                        self.control_cmd.data[1] = 0.0
                        self.vel_pub1.publish(self.control_cmd)
         
        else:
            #if disArmed then pass default values
            self.control_cmd.data = [ -1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.28]
            self.automode = False
            self.armed = False


if __name__ == "__main__":
    try:
        twistControl()
    except rospy.ROSInterruptException:
        pass

