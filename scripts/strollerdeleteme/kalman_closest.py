#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from prrkalman import kalman_predict, kalman_update
from math import pi
from marker_array_utils import MarkerArrayUtils
from std_msgs.msg import ColorRGBA

GAZEBO=False

def filter(a, i):
    x = np.take(a, [i, i-1, i+1], mode='wrap')
    y = np.logical_and(x>0.05, x!=np.inf)
    slice = x[y]
    if slice.size == 0:
        return 20
    else:
        return np.average(x[y])

# minirover's lidear (ydlidar X4) sends back 720 numbers per rotation hence the divide by two.
def scan_callback(msg):
    global meas_dist, meas_bear
    ar = np.round(np.array(msg.ranges),2)
    filter_and_average = [filter(ar,x) for x in range(0, ar.size)]
    meas_bear = np.argmin(filter_and_average)
    meas_dist = filter_and_average[meas_bear]
    if (not GAZEBO):
# minirover's lidear (ydlidar X4) sends back 720 numbers per rotation hence the divide by two.
        meas_bear = math.radians(meas_bear/2)
    else:
# gazebo sends back 360
        meas_bear = math.radians(meas_bear)

def cmd_vel_callback(msg):
    global g_turn_cmd, g_forward_cmd
    g_forward_cmd = msg.linear.x
    g_turn_cmd = msg.angular.z

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

rospy.init_node('kalman')
rate = rospy.Rate(10)

# Wait for the simulator to be ready
while rospy.Time.now().to_sec() == 0:
    rate.sleep()

g_time_now = rospy.Time()
g_forward_cmd = 0
g_turn_cmd = 0
state_dist = 0.5
meas_dist = 0.5
meas_bear = math.radians(0)
state_bear = math.radians(0)
mu = MarkerArrayUtils()
red = ColorRGBA(0.9,0.0,0,1)
green = ColorRGBA(0,0.9,0.0,1)

def invert_angle(angle):
    """To make the arrow markers work correctly we need to flip the angle because angles go the other way there"""
    if (not GAZEBO):
        return (angle + math.pi) % (2 * math.pi)
    else:
        return angle

while not rospy.is_shutdown():
    elapsed = rospy.Time.now().to_sec() - g_time_now.to_sec()
    control_motion = g_forward_cmd * elapsed
    state_dist_temp, state_bear_temp = kalman_predict(state_dist, state_bear, control_motion)
    state_dist, state_bear = kalman_update(state_dist_temp, state_bear_temp, meas_dist, meas_bear)
    #print("%2f,%2f,%2f,%2f,%2f " % (elapsed, g_forward_cmd , g_turn_cmd, math.degrees(meas_bear), meas_dist))
    print("move: %2f, m b: %2f, d: %2f, temp b: %2f, d: %2f, state b: %2f, d: %2f " % 
        (control_motion, meas_bear, meas_dist, state_bear_temp, state_dist_temp, state_bear, state_dist))
    mu.add_marker(1, red, invert_angle(state_bear), state_dist)
    # mu.add_marker(2, green, invert_angle(meas_bear), meas_dist)
    mu.publish()
    g_time_now = rospy.Time.now()
    rate.sleep()