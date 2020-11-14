#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from prrexamples.msg import Kalman
from tf.transformations import quaternion_from_euler
import numpy as np
import math
from marker_array_utils import MarkerArrayUtils

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
    K = 0.5
    d_prime = K * d + (1-K) * d_meas
    b_prime = K * b + (1-K) * d_bear
    return (d_prime, b_prime)

# d = previous state distance to target
# b = previous state bearing to target
# m = amount moved forward

def kalman_predict(d, b, m):
    d_prime = math.sqrt(m * m + d * d - 2 * m * d * math.cos(b))
    b_prime = math.pi - math.asin(d * (math.sin(b)/d_prime))
    return d_prime, b_prime

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

def pub_markers(markers, state_dist, state_bear, meas_dist, meas_bear):
    m = MarkerArray()
    m.markers.append(make_marker(1, state_dist, state_bear, 0, 1, 0))
    m.markers.append(make_marker(2, meas_dist, meas_bear, 1, 0, 0))
    markers.publish(m)

def make_marker(seq, d, b, red, green, blue):
    m = Marker()
    m.header.frame_id = "base_scan"
    m.header.seq = seq
    m.text = "yes"
    m.id = seq
    m.header.stamp = rospy.get_rostime()
    m.type = m.ARROW
    m.action = m.ADD
    m.scale.y = 0.01
    m.scale.z = 0.01
    m.scale.x = d
    m.color.a = 1.0
    m.color.r = red
    m.color.g = green
    m.color.b = blue
    quaternion = quaternion_from_euler(0, 0, b)
    m.pose.orientation.x = quaternion[0]
    m.pose.orientation.y = quaternion[1]
    m.pose.orientation.z = quaternion[2]
    m.pose.orientation.w = quaternion[3]
    return(m)

def pub_kalman(monitor):
    k = Kalman(elapsed, g_forward_cmd, g_turn_cmd, e_dist, e_bear)
    monitor.publish(k)

scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
monitor = rospy.Publisher('/kalman/info', Kalman, queue_size=10)
markers = rospy.Publisher('/kalman/marker', MarkerArray, queue_size=10)

rospy.init_node('kalman')
rate = rospy.Rate(10)

# Wait for the simulator to be ready
while rospy.Time.now().to_sec() == 0:
    rate.sleep()

g_time_now = rospy.Time()
g_forward_cmd = 0
g_turn_cmd = 0
state_dist = 1.5
meas_dist = 1.5
meas_bear = math.radians(0)
state_bear = math.radians(0)

while not rospy.is_shutdown():
    elapsed = rospy.Time.now().to_sec() - g_time_now.to_sec()
    control_motion = g_forward_cmd * elapsed
    state_dist_temp, state_bear_temp = kalman_predict(state_dist, state_bear, control_motion)
    state_dist, state_bear = kalman_update(state_dist_temp, state_bear_temp, meas_dist, meas_bear)
    print("%2f, %2f, %2f, %2f, %2f, %2f " % (0, elapsed, g_forward_cmd , g_turn_cmd, math.degrees(meas_bear), meas_dist))
    pub_markers(markers, state_dist, state_bear, meas_dist, meas_bear)
    g_time_now = rospy.Time.now()
    rate.sleep()