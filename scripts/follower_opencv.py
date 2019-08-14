#!/usr/bin/env python

from __future__ import print_function
import rospy
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Follower callback
class Follower:
    def __init__(self):
        self.cv_img = None
        # Open CV Bridge, and create a window
        self.bridge = CvBridge()
        # subscribe to the topic, and give each image to the callback
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)

    # Called once per image received
    def image_callback(self, frame):
        # Convert image to cv2 and display it
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")
        except:
            print("Unexpected error:", sys.exc_info()[0])
            raise
        
def main():
    cv2.namedWindow('Original')
    follower = Follower()
    rospy.init_node('follower', anonymous=True)

    while(not rospy.is_shutdown()):
        if follower.cv_img is not None:
            cv2.imshow('Original', follower.cv_img)
            
        # Wait for ESC to be clicked on opencv window
        key = cv2.waitKey(3) & 0xFF
        if key == 27:
            break

if __name__ == '__main__':
    main()
