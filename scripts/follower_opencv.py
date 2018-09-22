#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Follower callback
class Follower:
    def __init__(self):
        self.cv_img = None
        # Open CV Bridge, and create a window
        self.bridge = CvBridge()
        # subscribe to the topic, and give each image to the callback
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

    # Called once per image received
    def image_callback(self, frame):
        # Convert image to cv2 and display it
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError as exc:
            print(traceback.format_exc())

def main():
    cv2.namedWindow('Original')
    follower = Follower()
    rospy.init_node('follower', anonymous=True)

    while(not rospy.is_shutdown()):
        if follower.cv_img is not None:
            cv2.imshow('Original', follower.cv_img)
        key = cv2.waitKey(3) & 0xFF
        if key == 27:
            break
        rospy.spin()

if __name__ == '__main__':
    main()
