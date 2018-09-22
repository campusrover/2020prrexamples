#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

# Callback does nothing yet
def image_callback(msg):
    print("gotimage!")

rospy.init_node('follower')

# Subscribe to raw image topic
image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)
rospy.spin()