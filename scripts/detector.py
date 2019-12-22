#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from prrexamples.msg import Detector2
import numpy as np

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


def calc_range(rang, mp1, mp2, wedge):
    r1 = [dist for dist in rang[mp1-wedge:mp1] if dist != 0]
    r2 = [dist for dist in rang[mp2:mp2+wedge] if dist != 0]
    combo = r1 + r2
    if (len(combo) != 0):
        mean = sum(combo) / len(combo)
    else:
        mean = float('nan')
    return mean


def calc_range_1(range, start, length):
    r1 = [dist for dist in range[start:start+length] if dist != 0]
    if (len(r1) != 0):
        mean = sum(r1) / len(r1)
    else:
        mean = float('nan')
    return mean

def scan_callback(msg):
    global counter

    forward=calc_range(msg.ranges, 359, 0, 15)
    right=calc_range(msg.ranges, 270, 271, 15)
    left=calc_range(msg.ranges, 90, 91, 15)
    back=calc_range(msg.ranges, 180, 181, 15)
    narrow_l1=sum(msg.ranges[83:87])

    narrow_l1=calc_range_1(msg.ranges, 83, 5)
    narrow_l2=calc_range_1(msg.ranges, 88, 5)
    narrow_l3=calc_range_1(msg.ranges, 93, 5)
    narrow_r1=calc_range_1(msg.ranges, 273, 5)
    narrow_r2=calc_range_1(msg.ranges, 268, 5)
    narrow_r3=calc_range_1(msg.ranges, 263, 5)

    closest_dist=np.nanmin([narrow_l1, narrow_l2, narrow_l3,
                              narrow_r1, narrow_r2, narrow_r3, forward, left, right, back])

    if (closest_dist == forward):
        closest_dir="forward"
    elif (closest_dist == left):
        closest_dir="left"
    elif (closest_dist == right):
        closest_dir="right"
    elif (closest_dist == back):
        closest_dir="back"
    elif (closest_dist == narrow_l1):
        closest_dir="narrow_l1"
    elif (closest_dist == narrow_l2):
        closest_dir="narrow_l2"
    elif (closest_dist == narrow_l3):
        closest_dir="narrow_l3"
    elif (closest_dist == narrow_r1):
        closest_dir="narrow_r1"
    elif (closest_dist == narrow_r2):
        closest_dir="narrow_r2"
    elif (closest_dist == narrow_r3):
        closest_dir="narrow_r3"
    else:
        closest_dir="bug"

# New way to represent obstacles.
# norm_bearing: -1 > +1 float. Negative means left, positive means right, 0 means forward, -1 or +1 mean reverse
# norm_dist: distance in meters in stated direction

    w=5
    norm_dist=5
    for norm in np.linspace(-1, 1, 21):
        mid=(360 + norm * 180) % 360
        p1=(mid - w + 360) % 360
        p2=(mid + w + 360) % 360
        p1i=int(p1)
        p2i=int(p2-1)
        midi=int(mid)
        dist=calc_range(msg.ranges, midi-1, midi, w)
        if (dist < norm_dist):
            norm_dist=dist
            norm_bearing=norm

    counter += 1
    if (counter % 2 == 0):
        d=Detector2(narrow_l1, narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3, forward, left, right, back, closest_dist, closest_dir,
        norm_bearing, norm_dist)
        log_msg="""\
\n---\nCLOSEST: %s at %1.1f\n
flrb: %1.1f %1.1f %1.1f %1.1f\n
nl: %1.2f %1.2f %1.2f, nr: %1.2f %1.2f %1.2f\n
NORM (bearing,dist): %1.1f %1.1f""" % (closest_dir, closest_dist, forward, left, right, back, narrow_l1,
             narrow_l2, narrow_l3, narrow_r1, narrow_r2, narrow_r3, norm_bearing, norm_dist)
        rospy.loginfo_throttle(2, log_msg)
        pub.publish(d)

# Create the node
counter=0
rospy.init_node('pid_range', log_level = rospy.DEBUG)
scan_sub=rospy.Subscriber('scan', LaserScan, scan_callback)
pub=rospy.Publisher('detector', Detector2, queue_size = 10)

# Loop until ^c
rospy.spin()
