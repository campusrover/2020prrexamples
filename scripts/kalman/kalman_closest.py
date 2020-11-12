#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from prrexamples.msg import Kalman
from tf.transformations import quaternion_from_euler
import numpy as np
import math

def filter(a, i):
    x = np.take(a, [i, i-1, i+1], mode='wrap')
    y = np.logical_and(x>0.05, x!=np.inf)
    slice = x[y]
    if slice.size == 0:
        return 20
    else:
        return np.average(x[y])

def kalman_update(index, elapsed, m_dist, e_dist, m_bear, e_bear):
    newe_dist = 0.6*e_dist + 0.4*m_dist
    return (newe_dist, e_bear)

def scan_callback(msg):
    global m_dist, m_bear
    ar = np.array(msg.ranges)
    filter_and_average = [filter(ar,x) for x in range(0, ar.size)]
    m_bear = np.argmin(filter_and_average)
    m_dist = filter_and_average[m_bear]

def cmd_vel_callback(msg):
    global g_turn_cmd, g_forward_cmd
    g_forward_cmd = msg.linear.x
    g_turn_cmd = msg.angular.z

def pub_marker(marker, dist, bearing):
    m = Marker()
    m.header.frame_id = "base_scan"
    m.header.seq = 1
    m.header.stamp = rospy.get_rostime()
    m.type = m.ARROW
    m.scale.y = 0.01
    m.scale.z = 0.01
    m.scale.x = dist
    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    quaternion = quaternion_from_euler(0, 0, math.radians(bearing))
    m.pose.orientation.x = quaternion[0]
    m.pose.orientation.y = quaternion[1]
    m.pose.orientation.z = quaternion[2]
    m.pose.orientation.w = quaternion[3]
    marker.publish(m)

def pub_kalman(monitor):
    k = Kalman(elapsed, g_forward_cmd, g_turn_cmd, e_dist, e_bear)
    monitor.publish(k)

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
monitor = rospy.Publisher('/kalman/info', Kalman, queue_size=10)
marker = rospy.Publisher('/kalman/marker', Marker, queue_size=10)

rospy.init_node('kalman')
rate = rospy.Rate(10)

# Wait for the simulator to be ready
while rospy.Time.now().to_sec() == 0:
    rate.sleep()

m_dist = 0
e_dist = 1
m_bear = 0
e_bear = 0
g_forward_cmd = 0 
g_turn_cmd = 0
g_time_now = rospy.Time()
index = 1

while not rospy.is_shutdown():
    elapsed = rospy.Time.now().to_sec() - g_time_now.to_sec()
    # str = "Current delta-t: %.3f fwd: %.3f rot: %.3f bearing: %.3f range: %.3f" % (elapsed, g_forward_cmd, g_turn_cmd, g_shortest_bearing, g_shortest)
    # rospy.loginfo(str)
    str = "%.3f, %.3f, %.3f, %.3f, %.3f" % (elapsed, g_forward_cmd, g_turn_cmd, m_dist, e_dist)
    print(str)
    e_dist, e_bear = kalman_update(index, elapsed, m_dist, e_dist, m_bear, e_bear)
    pub_marker(marker, e_dist, m_bear)
    # pub_kalman(monitor)
    index = index + 1
    g_time_now = rospy.Time.now()
    rate.sleep()