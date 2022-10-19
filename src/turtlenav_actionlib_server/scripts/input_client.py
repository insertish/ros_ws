#!/bin/python3
import rospy
import actionlib

from turtlenav_actionlib_common.msg import GoToPosAction, GoToPosGoal, GoToPosActionFeedback


# log action feedback
def callback(msg: GoToPosActionFeedback):
    rospy.loginfo("Robot is currently %d units away from goal",
                  msg.feedback.distance_left)


if __name__ == '__main__':
    # initialise node
    rospy.init_node('point_nav', anonymous=True)

    # listen to feedback from the action server
    # we are only concerened with logging so this can be separate
    rospy.Subscriber('/go_to_pos/feedback', GoToPosActionFeedback, callback)

    # create an actionlib client
    client = actionlib.SimpleActionClient('go_to_pos', GoToPosAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()

    while True:
        # ask for a new goal
        print("Ready to go to new goal.")
        x = int(input("Enter X: "))
        y = int(input("Enter Y: "))

        # construct the goal
        goal = GoToPosGoal(x=x, y=y)

        # send goal to the server
        client.send_goal(goal)
        client.wait_for_result()
