#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
import math

class MarkerArrayUtils:
    def __init__(self):
        self.marker_publisher = rospy.Publisher('/kalman/marker', MarkerArray, queue_size=10)
        self.marker_message = MarkerArray()

    def publish(self):
        self.marker_publisher.publish(self.marker_message)
        self.marker_message = MarkerArray()

    def add_marker(self, id, color, bearing, distance):
        m = Marker()
        m.header.frame_id = "base_scan"
        m.id = id
        m.header.stamp = rospy.get_rostime()
        m.type = m.ARROW
        m.action = m.ADD
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.scale.x = distance
        m.color = color
        quaternion = quaternion_from_euler(0, 0, bearing)
        m.pose.orientation.x = quaternion[0]
        m.pose.orientation.y = quaternion[1]
        m.pose.orientation.z = quaternion[2]
        m.pose.orientation.w = quaternion[3]
        self.marker_message.markers.append(m)

if __name__ == "__main__":
    rospy.init_node('marker_test')
    rate = rospy.Rate(3)
    bear = 10
    while not rospy.is_shutdown():
        mu = MarkerArrayUtils()
        b1 = math.radians(bear)
        b2 = math.radians(bear+10)

        mu.add_marker(1, ColorRGBA(1,0,0,1), b1, 1 )
        mu.add_marker(2, ColorRGBA(0,1,0,1), b2, 2)
        mu.publish()
        bear = bear + 10
        rate.sleep()
