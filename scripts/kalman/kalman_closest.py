#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import trianglesolver as ts
from math import pi

from marker_array_utils import MarkerArrayUtils
from std_msgs.msg import ColorRGBA

# d = previous state distance to target
# b = previous state bearing to target
# d_meas = new measurement of distance to target
# b_meas = new measurement of bearing to target

def kalman_update(d, b, d_meas, d_bear):
    K = 00.3
    d_prime = K * d + (1-K) * d_meas
    b_prime = K * b + (1-K) * d_bear
    return (d_prime, b_prime)

# a = previous state distance to target
# B = previous state bearing to target
# c = amount moved forward
# Returns (b = new distance to target, A=new bearing to target)
# Note that if no motion, bearing and distance dont change

def kalman_predict(a, B, c):
    #print(f"KP {a} {B} {c}")
    if (c == 0):
        #print("c == 0")
        return(a, B)
    elif (B == pi):
        #print("B == pi")
        return(a, B)
    elif (B > pi):
        B_prime = (2*pi - B)
        if not all(x > 0 for x in (a,c,B_prime)):
            print("+++++++++", a,c,B_prime)
            return(a, B)
        else:
            #print (f"B > pi: {B} {B_prime}")
            (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(a=a, B=B_prime, c=c)
        return (tsb, 2*pi - tsA)
    else:
        if not all(x > 0 for x in (a,c,B)):
            print("***********", a,c,B)
            return(a, B)
        else:
            (tsa,tsb,tsc,tsA,tsB,tsC) = ts.solve(a=a, B=B, c=c)
        return(tsb, tsA)

def null_kalman_predict(d, b, m):
    d_prime = d
    b_prime = b
    return d_prime, b_prime

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
    ar = np.array(msg.ranges)
    filter_and_average = [filter(ar,x) for x in range(0, ar.size)]
    meas_bear = np.argmin(filter_and_average)
    meas_dist = filter_and_average[meas_bear]
# minirover's lidear (ydlidar X4) sends back 720 numbers per rotation hence the divide by two.
    meas_bear = math.radians(meas_bear/2)

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
    return (angle + math.pi) % (2 * math.pi)

while not rospy.is_shutdown():
    elapsed = rospy.Time.now().to_sec() - g_time_now.to_sec()
    control_motion = g_forward_cmd * elapsed
    state_dist_temp, state_bear_temp = kalman_predict(state_dist, state_bear, control_motion)
    state_dist, state_bear = kalman_update(state_dist_temp, state_bear_temp, meas_dist, meas_bear)
    #print("%2f,%2f,%2f,%2f,%2f " % (elapsed, g_forward_cmd , g_turn_cmd, math.degrees(meas_bear), meas_dist))
    print("move: %2f, m b: %2f, d: %2f, temp b: %2f, d: %2f, state b: %2f, d: %2f " % 
        (control_motion, meas_bear, meas_dist, state_bear_temp, state_dist_temp, state_bear, state_dist))
    mu.add_marker(1, red, invert_angle(state_bear), state_dist)
    mu.add_marker(2, green, invert_angle(meas_bear), meas_dist)
    mu.publish()
    g_time_now = rospy.Time.now()
    rate.sleep()