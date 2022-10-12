#!/bin/python3
import math
import rospy
import angles
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

cmd_vel: rospy.Publisher
goal = (1, 10)

angle_threshold = 0.007
distance_threshold = 0.3


def callback(pose: Pose):
    # create an empty command
    twist = Twist()

    # determine angle between x-axis and line from our position to goal
    difference = (goal[0] - pose.x, goal[1] - pose.y)
    goal_orientation = math.atan2(difference[1], difference[0])

    # determine difference between turtle's current angle and the angle we want to be facing
    angle_diff = angles.shortest_angular_distance(pose.theta, goal_orientation)

    # if within threshold, turn towards the goal
    if angle_diff < -angle_threshold:
        twist.angular.z = -0.5
    elif angle_diff > angle_threshold:
        twist.angular.z = 0.5
    # if above threshold, move towards the goal
    # math.hypot gives us the length of the difference vector
    elif math.hypot(*difference) > distance_threshold:
        twist.linear.x = 1

    # publish command
    cmd_vel.publish(twist)


def listener():
    global cmd_vel

    # initialise node
    rospy.init_node('point_nav', anonymous=True)

    # setup publishers and subscribers
    cmd_vel = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, callback)

    # run indefinitely
    rospy.spin()


if __name__ == '__main__':
    listener()
