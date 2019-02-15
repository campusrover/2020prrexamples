#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from rosbook.msg import Detector

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
    forward = calc_range(msg.ranges, 359, 0, 15)
    right = calc_range(msg.ranges, 270, 271, 15)
    left = calc_range(msg.ranges, 90, 91, 15)
    back = calc_range(msg.ranges, 180, 181, 15)
    narrow_l1 = sum(msg.ranges[75:84])/10
    narrow_l2 = sum(msg.ranges[85:94])/10
    narrow_l3 = sum(msg.ranges[95:104])/10
    narrow_r1 = sum(msg.ranges[275:284])/10
    narrow_r2 = sum(msg.ranges[265:274])/10
    narrow_r3 = sum(msg.ranges[255:264])/10
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

    d = Detector(narrow_l1, narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3,
                 forward, left, right, back, closest_dist, closest_dir)
    print ("   [%s=%1.1f] [flrb %1.1f %1.1f %1.1f %1.1f]" %
          (closest_dir, closest_dist, forward, left, right, back)),
    print(" nl: [%1.2f %1.2f %1.2f], nr: [%1.2f %1.2f %1.2f]" % (
        narrow_l1, narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3))
    pub.publish(d)


# Create the node
rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
pub = rospy.Publisher('detector', Detector, queue_size=10)

# Loop until ^c
rospy.spin()
