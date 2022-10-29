#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point

# initialise node
rospy.init_node('draw_node', anonymous=True)

# create publisher
pub = rospy.Publisher("/map_utils/marker", Marker, queue_size=10)

index = 0
def create_point(point: Point):
    global index

    msg = Marker()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"
    msg.ns = "map_utils"
    msg.id = 0
    msg.type = Marker.SPHERE
    msg.action = Marker.ADD
    msg.pose.position = point
    msg.scale.x = 0.3
    msg.scale.y = 0.3
    msg.scale.z = 0.3
    msg.color.r = 0
    msg.color.b = 1
    msg.color.g = 0
    msg.color.a = 1

    pub.publish(msg)
    index += 1

def create_line(point_a: Point, point_b: Point):
    global index

    msg = Marker()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"
    msg.ns = "map_lines"
    msg.id = 0
    msg.type = Marker.LINE_STRIP
    msg.action = Marker.ADD
    msg.points = [
        point_a,
        point_b
    ]
    msg.colors = [
        0, 1, 0, 1,
        0, 1, 0, 1
    ]
    msg.scale.x = 1
    msg.scale.y = 1
    msg.scale.z = 1

    pub.publish(msg)
    index += 1

def callback_point(msg: PointStamped):
    create_point(msg.point)

# create subscriber
rospy.Subscriber('/clicked_point', PointStamped, callback_point)

create_line(
    Point(0, 0, 0),
    Point(5, 5, 0)
)

rospy.spin()

# start publishing markers
""" r = rospy.Rate(1)
x = 5
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
    msg.pose.position.x = 0 + x
    msg.pose.position.y = 2
    msg.pose.position.z = 0.2
    msg.scale.x = 1
    msg.scale.y = 1
    msg.scale.z = 1
    msg.color.r = 0
    msg.color.b = 1
    msg.color.g = 0
    msg.color.a = 1

    if x > 10:
        x = 0

    pub.publish(msg)
    r.sleep()
    x += 1
 """