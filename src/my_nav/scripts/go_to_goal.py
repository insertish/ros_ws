#!/usr/bin/env python3

import rospy
import actionlib
from math import sin, cos, pi
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node('go_to_point')

move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
move_base_client.wait_for_server()

goals = [
    (1.95, 8.65),
    (5.7, 2.3)
]

while True:
    for target_pos in goals:
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = 'map'

        goal.target_pose.pose.position.x = target_pos[0]
        goal.target_pose.pose.position.y = target_pos[1]
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = sin(- pi / 4)
        goal.target_pose.pose.orientation.w = cos(- pi / 4)

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()

        status = move_base_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            print("Successfully reached point.")
        else:
            print("Failed to reach point.")
