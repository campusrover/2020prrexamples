#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

from marker_array_utils import MarkerArrayUtils
from std_msgs.msg import ColorRGBA

def filter(a, i):
    x = np.take(a, [i, i-1, i+1], mode='wrap')
    y = np.logical_and(x>0.05, x!=np.inf)
    slice = x[y]
    if slice.size == 0:
        return 20
    else:
        return np.average(x[y])

# d = previous state distance to target
# b = previous state bearing to target
# d_meas = new measurement of distance to target
# b_meas = new measurement of bearing to target

def kalman_update(d, b, d_meas, d_bear):
    K = 0.6
    d_prime = K * d + (1-K) * d_meas
    b_prime = K * b + (1-K) * d_bear
    return (d_prime, b_prime)

def kalman_predict(d, b, m):
    d_prime = d
    b_prime = b
    return d_prime, b_prime

# d = previous state distance to target
# b = previous state bearing to target
# m = amount moved forward

def trig_kalman_predict(d, b, m):
    d_prime = law_of_cosines_b(d, m, b)
    b_prime = math.pi - law_of_sines_A(d, b, d_prime)
    return d_prime, b_prime

def law_of_sines_A(a, B, b):
    return (math.asin((a * math.sin(B)) / b)) 

# Capital letter B is an angle. Small a, b and c are edges. 
# a is the edge across from A (I think that covers it.)
def law_of_cosines_b(a, c, B):
    return(math.sqrt(a**2 + c**2 - 2*a*c*math.cos(B)))

def scan_callback(msg):
    global meas_dist, meas_bear
    ar = np.array(msg.ranges)
    filter_and_average = [filter(ar,x) for x in range(0, ar.size)]
    meas_bear = np.argmin(filter_and_average)
    meas_dist = filter_and_average[meas_bear]
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

while not rospy.is_shutdown():
    elapsed = rospy.Time.now().to_sec() - g_time_now.to_sec()
    control_motion = g_forward_cmd * elapsed
    state_dist_temp, state_bear_temp = kalman_predict(state_dist, state_bear, control_motion)
    state_dist, state_bear = kalman_update(state_dist_temp, state_bear_temp, meas_dist, meas_bear)
    print("%2f,%2f,%2f,%2f,%2f " % (elapsed, g_forward_cmd , g_turn_cmd, math.degrees(meas_bear), meas_dist))
    # print("m b: %2f, d: %2f, temp b: %2f, d: %2f, state b: %2f, d: %2f " % (meas_bear, meas_dist, state_bear_temp, state_dist_temp, state_bear, state_dist))
    mu.add_marker(1, ColorRGBA(0.9,0.0,0,1), state_bear, state_dist)
    mu.add_marker(2, ColorRGBA(0,0.9,0.0,1), meas_bear, meas_dist)
    mu.publish()
    g_time_now = rospy.Time.now()
    rate.sleep()