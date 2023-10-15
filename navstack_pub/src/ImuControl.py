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
        self.orientation.update(0,0,-9.8,0,0,0, 0.000625)
        self.ax = 0
        self.ay = 0
        self.az = -9.8
        self.gx = 0
        self.gy = 0
        self.gz = 0
        self.alpha = 0.010
        self.camangle = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        #                                      rl,  ph,  yw,   kp,   ki,   kd,   k
        self.cnt_msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.22, 0.0, 0.1, 5.0,])
        
        
        rospy.init_node('Imu_control', anonymous=True)
        rate = rospy.Rate(50)
        self.ori_pub = rospy.Publisher('/campose', Float32MultiArray , queue_size=1)
        rospy.Subscriber('/imu_filter', Imu, self.imuCB, queue_size=1)
        
        while not rospy.is_shutdown():
            
            self.cnt_msg.data[0] = self.roll
            self.cnt_msg.data[1] = -((self.pitch))
            self.cnt_msg.data[2] = -((self.yaw))

            '''
            self.cnt_msg.data[0] = self.orientation.roll
            self.cnt_msg.data[1] = -((self.orientation.pitch))
            '''
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
            self.orientation.update(self.ax, self.ay, self.az, self.gx, self.gy, self.gz, 0.0025)

            self.roll, self.pitch, self.yaw = np.degrees(self.quaternion_to_rpy(imu.orientation))

        except (Exception, b):
            print (b)

    def quaternion_to_rpy(self, quaternion):
        # Convert quaternion to roll, pitch, and yaw angles
        qw = quaternion.w
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
	
if __name__ == '__main__':
    try:
        imuControl()
    except rospy.ROSInterruptException:
        pass
