#!/bin/python3
import rospy
from math import pi
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

# initialise node
rospy.init_node('circle_node', anonymous=True)

# reset the turtle simulation
rospy.ServiceProxy('/reset', Empty)()

# setup publisher
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

# setup rate
rate = rospy.Rate(10)

# run until kill
while not rospy.is_shutdown():
    move_msg = Twist()
    move_msg.linear.x = 1

    for i in range(0, 20):
        velocity_publisher.publish(move_msg)
        rate.sleep()

    rotate_msg = Twist()
    rotate_msg.angular.z = pi / 2
    velocity_publisher.publish(rotate_msg)

    for i in range(0, 15):
        rate.sleep()
