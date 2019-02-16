#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

# Function is called every time the lidar has a scan
# Once for every rotation of the scan
# Notice msg is the one parameter
def scan_callback(msg):
  range_ahead = msg.ranges[len(msg.ranges)/2]
  print "range ahead: %0.1f" % range_ahead


# Create the node
rospy.init_node('range_ahead', log_level=rospy.DEBUG)

# declare the subsceriber
# Python class of msg is LaserScan
# Function to call on each scan is scan_callback
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Loop until ^c
rospy.spin()
