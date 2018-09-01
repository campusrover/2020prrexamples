#!/usr/bin/env python
# Don't forget to chmod +x topic_publisher.py

import rospy
from std_msgs.msg import Int32

# Make this into a ROS node.
rospy.init_node('topic_publisher')

# Prepare to publish topic `counter`
pub = rospy.Publisher('counter', Int32, queue_size=10)

# sleep at 2 loops per second
rate = rospy.Rate(2)
count = 0

# loop until ^c
while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()