#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class OpenloopCircle():
	
    def __init__(self):
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.2
        self.vel_cmd.angular.z = 0

        rospy.loginfo("Open Loop Control node initialized")
	#init publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(50)
        while True:
            self.vel_pub.publish(self.vel_cmd)
            self.rate.sleep()
            #rospy.sleep(0.05)            

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Openloop_Circle_Node")

    # Init the node
    OpenloopCircle()

    # Don't let this script exit while ROS is still running
    rospy.spin()
    
    try:
        OpenloopCircle()
    except rospy.ROSInterruptException:
        pass
