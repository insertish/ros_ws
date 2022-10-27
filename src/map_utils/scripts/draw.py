#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

# initialise node
rospy.init_node('draw_node', anonymous=True)

# create publisher
pub = rospy.Publisher("/map_utils/marker", Marker, queue_size=10)

# start publishing markers
r = rospy.Rate(20)
x = 0
while True:
    msg = Marker()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"
    msg.ns = "map_utils"
    msg.id = 0
    # msg.type = Marker.SPHERE
    msg.type = Marker.TEXT_VIEW_FACING
    msg.text = "get real"
    msg.action = Marker.ADD
    msg.pose.position.x = 3.11 + x
    msg.pose.position.y = 7.43
    msg.pose.position.z = 0.2
    msg.scale.x = 0.3
    msg.scale.y = 0.3
    msg.scale.z = 0.3
    msg.color.r = 0
    msg.color.b = 1
    msg.color.g = 0
    msg.color.a = 1

    pub.publish(msg)
    r.sleep()
    x += 0.05
