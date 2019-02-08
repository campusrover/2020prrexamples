#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rosbook.msg import Detector

def smart_logger(thestr):
    global last_log
    global count_log
    global left_wall_change
    global delta_from_goal
    global state
    global m
    if (thestr != last_log):
        print("#%s [%s] %s=%1.2f %s" % 
            (count_log, state, m.closest_dir, m.closest_dist, thestr))
        count_log = count_log + 1
    last_log = thestr

def scan_callback(msg):
    global m
    m = msg
 
def handle_followwall():
    global m
    global state
    smart_logger("Following wall")
    twist = Twist()
    if (m.forward < m.narrow_l2):
        state = "parallelize"
    if (m.narrow_l1 + 0.2 < m.narrow_l2):
        twist.angular.z = 0.2
    elif (m.narrow_l3 + 0.2 < m.narrow_l2):
        twist.angular.z = -0.2
    else:
        twist.linear.x = 1.0
    return(twist)

def handle_emer_stop():
    global m
    global state
    smart_logger("Emergency Stop")
    twist = Twist()
    twist.linear.x = 0.0
    return(twist)

def handle_find_wall():
    global m
    global state
    smart_logger("Finding Wall")
    twist = Twist()
    twist.linear.x = 0.2
    if (m.closest_dist <= 0.4):
        state = "parallelize"
    return(twist)

def handle_parallelize():
    global m
    global state
    smart_logger("Parallize")
    twist = Twist()
    twist.angular.z = 0.2
    if (m.closest_dir == "narrow_l2"):
        state = "followwall"
    return(twist)


cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
detect_sub = rospy.Subscriber('detector', Detector, scan_callback)

# Initialize this program as a node
rospy.init_node('pid_demo')
target_wall_dist = 0.4
count_log = 0
last_log = ""
state = "find_wall"
m = Detector()
m.closest_dist = 5

# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    if (m.closest_dist < 0.2):
        state = "emer_stop"
        print("%s: %1.1f" % (state, m.closest_dist))

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
