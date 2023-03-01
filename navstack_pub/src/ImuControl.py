#!/usr/bin/python3

import numpy as np
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import sys
import os
import time
import compfilter
import math

class imuControl(object):

    def __init__(self):
        self.orientation = compfilter.imu6Dof()
        self.orientation.update(0,0,-9.8,0,0,0, 0.005)
        self.ax = 0
        self.ay = 0
        self.az = -9.8
        self.gx = 0
        self.gy = 0
        self.gz = 0
        self.alpha = 0.05
        self.camangle = 0.0
        self.cnt_msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.27, 0.001, 0.38, 60.0,])
        
        
        rospy.init_node('Imu_control', anonymous=True)
        rate = rospy.Rate(80)
        self.ori_pub = rospy.Publisher('/campose', Float32MultiArray , queue_size=1)
        rospy.Subscriber('/imu', Imu, self.imuCB, queue_size=1)
        
        while not rospy.is_shutdown():

            self.cnt_msg.data[0] = self.orientation.roll
            self.cnt_msg.data[1] = -((self.orientation.pitch) + 5)

            self.ori_pub.publish(self.cnt_msg)
            rate.sleep()
 
    def imuCB(self, imu):
        try:
            
            self.ax = self.ax - (self.alpha * (self.ax - imu.linear_acceleration.x))
            self.ay = self.ay - (self.alpha * (self.ay - imu.linear_acceleration.y))
            self.az = self.az - (self.alpha * (self.az - imu.linear_acceleration.z))
            
            self.gx = imu.angular_velocity.x
            self.gy = imu.angular_velocity.y
            self.gz = imu.angular_velocity.z
            self.orientation.update(self.ax, self.ay, self.az, self.gx, self.gy, self.gz, 0.005)
     
        except (Exception, b):
            print (b)
	
if __name__ == '__main__':
    try:
        imuControl()
    except rospy.ROSInterruptException:
        pass0
