#!/bin/python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


def callback_Twist(data):
    rospy.loginfo(rospy.get_caller_id() + " received Twist message.\n%s", data)


def callback_Pose(data):
    rospy.loginfo(rospy.get_caller_id() + " received Pose message.\n%s", data)


def listener():
    rospy.init_node('debug_node', anonymous=True)
    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback_Twist)
    rospy.Subscriber("/turtle1/pose", Pose, callback_Pose)
    rospy.spin()


if __name__ == '__main__':
    listener()
