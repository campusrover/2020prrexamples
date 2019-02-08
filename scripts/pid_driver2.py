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
    if (thestr != last_log):
        print("#%s [%s] %s=%2.1f chgleft:%2.1f goaldif:%2.1f %s" % 
            (count_log, state, closest_dir, closest, left_wall_change, delta_from_goal, thestr))
        count_log = count_log + 1
    last_log = thestr

def scan_callback(msg):
    global closest
    global closest_dir
    closest = min(msg.leftnarrow, msg.forward, msg.left, msg.right, msg.back)
    if (msg.forward == closest):
        closest_dir = "forward"
    elif (msg.left == closest):
        closest_dir = "left"
    elif (msg.right == closest):
        closest_dir = "right"
    elif (msg.leftnarrow == closest):
        closest_dir = "lnarrow"
    else:
        closest_dir = "back"
    # print("closest: %2.1f at %s" % (closest, closest_dir))

def handle_followwall():
    global state
    global closest_dir
    global closest
    smart_logger("Following wall")
    twist = Twist()
    twist.linear.x = 0.1
    if (closest <= 0.2):
        state = "emer_stop"
    return(twist)

def handle_emer_stop():
    global state
    global closest_dir
    global closest
    smart_logger("Emergency Stop")
    twist = Twist()
    twist.linear.x = 0.0
    return(twist)

def handle_find_wall():
    global state
    global closest_dir
    global closest
    smart_logger("Finding Wall")
    twist = Twist()
    twist.linear.x = 0.2
    if (closest <= 1.0):
        state = "parallelize"
    return(twist)

def handle_parallelize():
    global state
    global closest_dir
    global closest
    smart_logger("Parallize")
    twist = Twist()
    twist.angular.z = 0.1
    if (closest_dir == "lnarrow"):
        state = "followwall"
    return(twist)



cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
detect_sub = rospy.Subscriber('detector', Detector, scan_callback)

# Initialize this program as a node
rospy.init_node('pid_demo')
closest = 5
closest_dir = "left"
delta_from_goal = 0
current_left = 0
last_left = 0
wall_dist = 0.4
last_log = ""
count_log = 0
buffer = [0]*1000
left_wall_change = 5
tw_for = Twist()
tw_for.linear.x = 0.1
tw_rot = Twist()
tw_rot.angular.z = -0.4
tw_stop = Twist()
state = "find_wall"

# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(10)

while not rospy.is_shutdown():
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
    # print(state)
    cmd_vel_pub.publish(tw)
    rate.sleep()
