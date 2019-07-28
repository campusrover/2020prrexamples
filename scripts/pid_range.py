#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from prrexamples.msg import Detector

# float32 narrow_l1
# float32 narrow_l2
# float32 narrow_l3
# float32 narrow_r1
# float32 narrow_r2
# float32 narrow_r3
# float32 forward
# float32 left
# float32 right
# float32 back
# float32 closest_dist
# string closest_dir


def calc_range(range, mp1, mp2, wedge):
    combo = range[mp1-wedge:mp1] + range[mp2:mp2+wedge]
    mean = sum(combo) / len(combo)
    return mean


def scan_callback(msg):
    global counter
    forward = calc_range(msg.ranges, 359, 0, 15)
    right = calc_range(msg.ranges, 270, 271, 15)
    left = calc_range(msg.ranges, 90, 91, 15)
    back = calc_range(msg.ranges, 180, 181, 15)
    narrow_l1 = sum(msg.ranges[83:87])/5
    narrow_l2 = sum(msg.ranges[88:92])/5
    narrow_l3 = sum(msg.ranges[93:97])/5
    narrow_r1 = sum(msg.ranges[273:277])/5
    narrow_r2 = sum(msg.ranges[268:272])/5
    narrow_r3 = sum(msg.ranges[263:267])/5
    closest_dist = min(narrow_l1, narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3,
                       forward, left, right, back)
    if (closest_dist == forward):
        closest_dir = "forward"
    elif (closest_dist == left):
        closest_dir = "left"
    elif (closest_dist == right):
        closest_dir = "right"
    elif (closest_dist == back):
        closest_dir = "back"
    elif (closest_dist == narrow_l1):
        closest_dir = "narrow_l1"
    elif (closest_dist == narrow_l2):
        closest_dir = "narrow_l2"
    elif (closest_dist == narrow_l3):
        closest_dir = "narrow_l3"
    elif (closest_dist == narrow_r1):
        closest_dir = "narrow_r1"
    elif (closest_dist == narrow_r2):
        closest_dir = "narrow_r2"
    elif (closest_dist == narrow_r3):
        closest_dir = "narrow_r3"
    else:
        closest_dir = "bug"

    counter += 1
    if (counter % 2 == 0):
        d = Detector(narrow_l1, narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3,
                    forward, left, right, back, closest_dist, closest_dir)
        # rospy.loginfo("CLOSEST: %s at %1.1f flrb: %1.1f %1.1f %1.1f %1.1f "+
        #             "nl: %1.2f %1.2f %1.2f, nr: %1.2f %1.2f %1.2f",
        #             closest_dir, closest_dist, forward, left, right, back, narrow_l1, 
        #             narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3)

        log_msg = "CLOSEST: %s at %1.1f flrb: %1.1f %1.1f %1.1f %1.1f nl: %1.2f %1.2f %1.2f, nr: %1.2f %1.2f %1.2f" % \
                    (closest_dir, closest_dist, forward, left, right, back, narrow_l1, 
                    narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3)
        rospy.loginfo_throttle(5, log_msg)
        pub.publish(d)

# Create the node
counter = 0
rospy.init_node('pid_range', log_level=rospy.DEBUG)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
pub = rospy.Publisher('detector', Detector, queue_size=10)

# Loop until ^c
rospy.spin()
