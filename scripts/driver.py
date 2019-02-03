#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# Create a Publisher object. queue_size=1 means that messages that are
# published but not handled by received are lost beyond queue size.
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Initialize this program as a node
rospy.init_node('pid_demo')

move = Twist()
move.linear.x = 1.0
print (rospy.Time.now())

# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    print (rospy.Time.now())
    cmd_vel_pub.publish(move)
    rate.sleep()