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
    # if (thestr != last_log):
    print("#%s [%s] %s=%1.2f %s" % 
        (count_log, state, m.closest_dir, m.closest_dist, thestr))
    count_log = count_log + 1
    last_log = thestr

def 

def scan_callback(msg):
    global m
    m = msg
 
def handle_followwall():
    global m
    global state
    twist = Twist()
    if (m.closest_dir != "narrow_l2"):
        state = "find_wall"
        return(twist)
    diff1 = m.narrow_l1 - m.narrow_l2
    diff2 = m.narrow_l3 - m.narrow_l2
    print("followwall closer front: %1.3f rear: %1.3f" % (diff1, diff2))
    if (diff1 < -0.1):
        twist.angular.z = 0.2
    elif (diff2 > 0.1):
        twist.angular.z = -0.2
    else:
        twist.linear.x = 0.1
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
    twist.linear.x = 0.2
    if (m.closest_dist <= 0.5):
        state = "parallelize"
    return(twist)

def handle_parallelize():
    global m
    global state
    twist = Twist()
    twist.angular.z = 0.2
    if (m.closest_dir == "narrow_l2"):
        state = "followwall"
    return(twist)


cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
detect_sub = rospy.Subscriber('detector', Detector, scan_callback)

# Initialize this program as a node
rospy.init_node('pid_demo')
target_wall_dist = 0.5
emer_stop_dist = 0.2
count_log = 0
last_log = ""
state = "find_wall"
m = Detector()
m.closest_dist = 5
m.closest_dir = "forward"

# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    count_log += 1
    print("#%s [%s] %s=%1.2f" % (count_log, state, m.closest_dir, m.closest_dist))

    if (m.closest_dist < emer_stop_dist):
        state = "emer_stop"
        tw = handle_emer_stop();
    elif (state == "find_wall"):
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
