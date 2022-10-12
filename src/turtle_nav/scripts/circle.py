#!/bin/python3
import rospy
from math import pi
from geometry_msgs.msg import Twist

rospy.init_node('circle_node', anonymous=True)

velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(10)

msg = Twist()
msg.linear.x = 2
msg.angular.z = pi / 4

while not rospy.is_shutdown():
    velocity_publisher.publish(msg)

    rate.sleep()
