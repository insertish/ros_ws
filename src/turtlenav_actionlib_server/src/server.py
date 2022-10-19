#!/bin/python3
import math
import rospy
import angles
import actionlib

# message imports
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlenav_actionlib_common.msg import GoToPosAction, GoToPosFeedback, GoToPosResult

# constants
angle_threshold = 0.005
distance_threshold = 0.1
topic_cmd_vel = "/turtle1/cmd_vel"
topic_pose = "/turtle1/pose"


class GoToPosActionServer():
    _server: actionlib.SimpleActionServer

    # current action variables
    _pub: rospy.Publisher
    _sub: rospy.Subscriber
    _goal: (int, int)
    _complete: bool

    # messages to publish
    _feedback = GoToPosFeedback()
    _result = GoToPosResult()

    def __init__(self, name: str):
        rospy.loginfo('Starting Action server...')

        # create the action server
        self._server = actionlib.SimpleActionServer(
            name, GoToPosAction, execute_cb=self.execute, auto_start=False)

        # start the server
        self._server.start()

    def execute(self, goal: GoToPosAction):
        # configure goal
        self._goal = (goal.x, goal.y)
        self._complete = False

        # log goal information
        rospy.loginfo('Now proceeding to move to goal (%d, %d)',
                      goal.x, goal.y)

        # initialise the velocity publisher
        self._pub = rospy.Publisher(topic_cmd_vel, Twist, queue_size=10)

        # start a subscriber
        self._sub = rospy.Subscriber(topic_pose, Pose, self.callback)

        # wait until goal is complete
        rate = rospy.Rate(1)
        while not self._complete:
            rate.sleep()

        # close publishers and subscribers
        self._sub.unregister()
        self._pub.unregister()

        # finish the action
        rospy.loginfo('Reached the desired goal, letting the client know.')
        self._server.set_succeeded(self._result)

    # handle incoming Pose messages
    def callback(self, pose: Pose):
        # create an empty command
        twist = Twist()

        # determine angle between x-axis and line from our position to goal
        difference = (self._goal[0] - pose.x, self._goal[1] - pose.y)
        goal_orientation = math.atan2(difference[1], difference[0])

        # determine difference between turtle's current
        # angle and the angle we want to be facing
        angle_diff = angles.shortest_angular_distance(
            pose.theta, goal_orientation)

        # if within threshold, turn towards the goal
        if angle_diff < -angle_threshold:
            twist.angular.z = -0.5
        elif angle_diff > angle_threshold:
            twist.angular.z = 0.5
        # if above threshold, move towards the goal
        # math.hypot gives us the length of the difference vector
        elif math.hypot(*difference) > distance_threshold:
            twist.linear.x = 2
        # otherwise, we've reached the target
        else:
            self._complete = True
            return

        # send feedback
        self._feedback.distance_left = math.hypot(*difference)
        self._server.publish_feedback(self._feedback)

        # publish command
        self._pub.publish(twist)


if __name__ == '__main__':
    # initialise node
    rospy.init_node('turtlenav_actionlib_server', anonymous=True)

    # start the server
    GoToPosActionServer('go_to_pos')

    # spin indefinitely
    rospy.spin()
