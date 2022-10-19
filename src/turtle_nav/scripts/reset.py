#!/bin/python3
import rospy
from std_srvs.srv import Empty

# initialise node
rospy.init_node('reset_node', anonymous=True)

# reset the turtle simulation
rospy.ServiceProxy('/reset', Empty)()
