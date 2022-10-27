#!/usr/bin/env python3

import rospy
from dynamic_square_service.srv import SetSize, SetSizeRequest

# initialise node
rospy.init_node('dynamic_square_client', anonymous=True)

# wait for service to be online
rospy.wait_for_service('set_square_size')

# setup service proxy
service_proxy = rospy.ServiceProxy('set_square_size', SetSize)

# ask user for size forever
while True:
    size = float(input('Enter target square size: '))
    service_proxy(SetSizeRequest(size=size))
