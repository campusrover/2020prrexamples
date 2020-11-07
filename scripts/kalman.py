#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from prrexamples.msg import Kalman
import math 


def scan_callback(msg):
    global g_range_ahead
    # print msg.ranges
    r1 = [dist for dist in msg.ranges[300:359] if dist != 0 and dist != float('inf')]
    r2 = [dist for dist in msg.ranges[0:10] if dist != 0 and dist != float('inf')]
    combo = r1 + r2
    if (len(combo) != 0):
        mean = sum(combo) / len(combo)
    else:
        mean = float('nan')
    g_range_ahead = mean

def cmd_vel_callback(msg):
    global g_rotation, g_forward
    g_forward = msg.linear.x
    g_rotation = msg.angular.z

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
monitor = rospy.Publisher('/kalman', Kalman, queue_size=10)

rospy.init_node('kalman')
rate = rospy.Rate(0.5)
g_range_ahead = 0
g_forward = 0 
g_rotation = 0
g_time_now = rospy.Time.now()

while not rospy.is_shutdown():
    elapsed = rospy.Time.now().to_sec() - g_time_now.to_sec()
    str = "Current delta-t: %.3f fwd: %.3f rot: %.3f range: %.3f" % (elapsed, g_forward, g_rotation, g_range_ahead)
    rospy.loginfo(str)
    k = Kalman(elapsed, g_forward, g_rotation, g_range_ahead)
    monitor.publish(k)
    g_time_now = rospy.Time.now()
    rate.sleep()