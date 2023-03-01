#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int16MultiArray


class RuleBased:
    def __init__(self):

        self.x_disp = 0
        self.y_disp = 0
        self.z_height = 0
        self.s_timeout = 2
        self.flow_threshold = 7
        self.flow_time = time.time()
        
        self.rule1_active = False
        self.rule1_step_time = time.time()
        self.rule1_rev_timeout = 3
        self.rule1_fw_timeout = 4
        self.rule1_rev_speed = 0.35
        self.rule1_rev_str_ang = 0.0
        self.rule1_fwd_speed_max = 0.28
        self.rule1_fwd_speed = 0.28
        self.rule1_fw_str_ang = 0.4
        self.rule1_step = 0
        self.rule1_acc_timeout = 3

        self.vel = Twist()
        self.normal_speed = 0.2
        # rospy.loginfo("Rule Based Control node initialized")
        # init publisher

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.optiflow_sub = rospy.Subscriber("/optiFlow_data", Int16MultiArray, self.optiFlowCB, queue_size=1)
        self.rate = rospy.Rate(50)
        while True:
            self.drive()
            self.vel_pub.publish(self.vel)
            # rospy.sleep(0.05)

    def optiFlowCB(self, optiflow):
        self.x_disp = optiflow.data[0]
        self.y_disp = optiflow.data[1]
        self.z_height = optiflow.data[2]
        if abs(self.x_disp) > self.flow_threshold :
            self.flow_time = time.time()

    def drive(self):

        #check if the movement timeout occured
        if ((time.time() - self.flow_time) > self.s_timeout) and not self.rule1_active:
            self.rule1_active = True
            print("rule 1 active")
            self.rule1_step_time = time.time()
        
        #Drive as per the rule1 routine if the rule is active
        if self.rule1_active:
            self.rule1_drv()

        #Drive normal routine if rule1 is inactive
        else:
            self.vel.linear.x = self.normal_speed 
            self.vel.angular.z = self.rule1_rev_str_ang
     
            
    def rule1_drv(self):

        #drive back for set time
        if self.rule1_step == 0:
            if time.time() - self.rule1_step_time >= self.rule1_acc_timeout:
                self.rule1_step_time = time.time()
                if ((time.time() - self.flow_time) > self.s_timeout):
                    self.rule1_step = 1
                    print("rule1 step 1")
                else:
                    self.rule1_step = 0
                    self.rule1_active = False
                    self.flow_time = time.time()
                    print("rule 1 complete")
            else:
                self.vel.linear.x = self.vel.linear.x +0.0000025
                if self.vel.linear.x > self.rule1_fwd_speed_max:
                    self.vel.linear.x = self.rule1_fwd_speed_max
                self.vel.angular.z = self.rule1_rev_str_ang 
        
        if self.rule1_step == 1:
            if time.time() - self.rule1_step_time >= self.rule1_rev_timeout:
                self.rule1_step_time = time.time()
                self.rule1_step = 2
                print("rule1 step 2")
            else:
                self.vel.linear.x =  -(self.rule1_rev_speed)
                self.vel.angular.z = self.rule1_rev_str_ang
        
        #drive forward for set time and set steering angle
        if self.rule1_step == 2:
            if time.time() - self.rule1_step_time >= self.rule1_rev_timeout/2:
                self.rule1_step = 3
                print("rule1 step3")
            else:
                self.vel.linear.x =  self.rule1_fwd_speed
                self.vel.angular.z = self.rule1_fw_str_ang
        
        #drive forward for set time and opposite of set steering angle 
        if self.rule1_step == 3:
            if time.time() - self.rule1_step_time >= self.rule1_rev_timeout:
                self.rule1_step = 0
                self.rule1_active = False
                self.flow_time = time.time()
                print("rule 1 complete")
            else:
                self.vel.linear.x =  self.rule1_fwd_speed
                self.vel.angular.z = -(self.rule1_fw_str_ang)


if __name__ == "__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Rule_Based_Node")

    # Init the node
    RuleBased()

    # Don't let this script exit while ROS is still running
    rospy.spin()

    try:
        RuleBased()
    except rospy.ROSInterruptException:
        pass

