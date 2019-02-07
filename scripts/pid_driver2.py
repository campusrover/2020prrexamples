#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rosbook.msg import Detector

def smart_logger(thestr):
    global last_log
    global count_log
    global left_wall_change
    global delta_from_goal
    if (thestr != last_log):
        print("#%s %s=%2.1f chgleft:%2.1f goaldif:%2.1f %s" % (count_log, closest_dir, closest, left_wall_change, delta_from_goal, thestr))
        count_log = count_log + 1
    last_log = thestr

def append_to_circular_buffer(value):
    global buffer
    buffer.pop(0)
    buffer.append(value)


def scan_callback(msg):
    global closest
    global closest_dir
    global current_left
    closest = min(msg.forward, msg.left, msg.right, msg.back)
    current_left = msg.left
    if (msg.forward == closest):
        closest_dir = "forward"
    elif (msg.left == closest):
        closest_dir = "left"
    elif (msg.right == closest):
        closest_dir = "right"
    else:
        closest_dir = "back"


def main():
    # Create a Publisher object. queue_size=1 means that messages that are
    # published but not handled by received are lost beyond queue size.
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    detect_sub = rospy.Subscriber('detector', Detector, scan_callback)

    # Initialize this program as a node
    rospy.init_node('pid_demo')
    closest = 5
    closest_dir = "left"
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
    state = "lost"

    # rate object gets a sleep() method which will sleep 1/10 seconds
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        tw = Twist()
        if (state == "lost"):
            tw = handle_lost()
        elif (state == "gotowall"):
            tw = handle_gotowall()
        elif (state == "followwall"):
            tw = handle_followwall()
        cmd_vel_pub.publish(tw)
        rate.sleep()

main()
