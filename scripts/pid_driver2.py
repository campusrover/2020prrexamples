#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from prrexamples.msg import Detector

def set_state(new_state):
    global state
    print("new state %s" % (new_state))
    state = new_state

def scan_callback(msg):
    global m
    m = msg
 
# Positive turns counterclockwise (left), negative turns clockwise (right) 
def handle_followwall():
    global m
    global state
    twist = Twist()
    diff = m.narrow_l1 - m.narrow_l3
    print("following: diff=%1.5f" % (diff))
    if (m.closest_dist > target_wall_dist):
        set_state("parallelize")
    elif (diff > 0.1):
        twist.angular.z = 0.2
    elif (diff < -0.1):
        twist.angular.z = -0.2
    else:
        twist.linear.x = 0.05
    return(twist)

def handle_emer_stop():
    global m
    global state
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    return(twist)

def handle_find_wall():
    global m
    global state
    twist = Twist()
    twist.linear.x = 0.1
    if (m.closest_dist <= target_wall_dist):
        set_state("parallelize")
    return(twist)

def handle_parallelize():
    global m
    global state
    twist = Twist()
    twist.angular.z = 0.2
    if (m.closest_dist > target_wall_dist):
        set_state("find_wall")
    elif ((m.closest_dir == "narrow_l1") or (m.closest_dir == "narrow_l2") or (m.closest_dir == "narrow_l3")):
        set_state("followwall")
    return(twist)

def change_state():
    global m
    global state
    global target_wall_dist
    global emer_stop_dist
    if (m.closest_dist < emer_stop_dist):
        set_state("emer_stop")

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
detect_sub = rospy.Subscriber('detector', Detector, scan_callback)

# Initialize this program as a node
rospy.init_node('pid_demo')
target_wall_dist = 0.4
emer_stop_dist = 0.2
set_state("find_wall")
m = Detector()
m.closest_dist = 5
m.closest_dir = "forward"
count_log = 0

# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    count_log += 1
    if (count_log % 10 == 0):
        print("#%s [%s] %s=%1.2f" % (count_log, state, m.closest_dir, m.closest_dist))

    change_state()
    if (state == "find_wall"):
        tw = handle_find_wall()
    elif (state == "parallelize"):
        tw = handle_parallelize()
    elif (state == "followwall"):
        tw = handle_followwall()
    elif (state == "emer_stop"):
        tw = handle_emer_stop()
    else:
        tw = handle_emer_stop()
    cmd_vel_pub.publish(tw)
    rate.sleep()
