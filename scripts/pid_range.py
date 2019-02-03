#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from rosbook.msg import Detector


def calc_range(range, mp1, mp2, wedge):
  combo = range[mp1-wedge:mp1] + range[mp2:mp2+wedge]
  mean = sum(combo) / len(combo)
  return mean

# Function is called every time the lidar has a scan
# Once for every rotation of the scan
# Notice msg is the one parameter
def scan_callback(msg):
  ahead_r = calc_range(msg.ranges, 359, 0, 15)
  right_r = calc_range(msg.ranges, 270, 271, 15)
  left_r = calc_range(msg.ranges, 90, 91, 15)
  back_r = calc_range(msg.ranges, 180, 181, 15)
  print "range (f,l,r,b) %0.1f %0.1f %0.1f %0.1f " % (ahead_r, left_r, right_r, back_r)
  d = Detector(ahead_r, left_r, right_r, back_r)
  pub.publish(d)

# Create the node
rospy.init_node('range_ahead')

# declare the subsceriber
# Python class of msg is LaserScan
# Function to call on each scan is scan_callback
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
pub = rospy.Publisher('detector', Detector, queue_size=10)


# Loop until ^c
rospy.spin()
