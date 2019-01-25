#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

# define function is called each time the message is published (by some other node)
def callback(msg):
   print "The square is " + str(msg.data*msg.data)

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('counter', Int32, callback)

# Wait for published topics, exit on ^c
rospy.spin()
