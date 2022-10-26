#!/bin/python3
import rospy
from math import pi
from geometry_msgs.msg import Twist
from dynamic_square_service.srv import SetSize, SetSizeRequest, SetSizeResponse

# constants
TURN_TIME_WAIT = 12
RATE = 10

# variables
size = 1

# callback for service
def set_size(request: SetSizeRequest):
    global size
    size = request.size
    return SetSizeResponse()

# run until kill
if __name__ == '__main__':
    # initialise node
    rospy.init_node('dynamic_square_node')

    # register service
    srv = rospy.Service('set_square_size', SetSize, set_size)
    print(srv)

    # setup publisher
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # setup rate
    rate = rospy.Rate(RATE)

    # setup messages
    move_msg = Twist()
    move_msg.linear.x = 1

    rotate_msg = Twist()
    rotate_msg.angular.z = pi / 2

    while not rospy.is_shutdown():
        # move forwards by size
        for i in range(0, int(size * RATE)):
            velocity_publisher.publish(move_msg)
            rate.sleep()

        # turn 90 deg
        velocity_publisher.publish(rotate_msg)
        for i in range(0, TURN_TIME_WAIT):
            rate.sleep()
