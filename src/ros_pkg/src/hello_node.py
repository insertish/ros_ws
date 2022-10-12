#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def run():
    pub = rospy.Publisher('hello', String, queue_size=10)
    rospy.init_node('hello_node', anonymous=True)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        hello_str = "hello!"
        pub.publish(hello_str)
        rate.sleep()


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
