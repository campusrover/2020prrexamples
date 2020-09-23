#!/usr/bin/env python

import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# fill in scan callback
def scan_cb(msg):
   return

# it is not necessary to add more code here but it could be useful
def key_cb(msg):
   global state; global last_key_press_time
   state = msg.data
   last_key_press_time = rospy.Time.now()

# odom is also not necessary but very useful
def odom_cb(msg):
   return


# print the state of the robot
def print_state():
   print("---")
   print("STATE: " + state)

   # calculate time since last key stroke
   time_since = rospy.Time.now() - last_key_press_time
   print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

# init node
rospy.init_node('dancer')

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# start in state halted and grab the current time
state = "H"
last_key_press_time = rospy.Time.now()

# set rate
rate = rospy.Rate(10)

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():

   # print out the current state and time since last key press
   print_state()

   # publish cmd_vel from here 
   t = Twist()
   # one idea: use vector-2 to represent linear and angular velocity
   # velocity_vector = [linear_component, angular_component]
   # then represent:
   # twist.linear.x = LINEAR_SPEED * linear_component
   # twist.angular.z = ANGULAR_SPEED * angular_component 
   # where for example:
   # LINEAR_SPEED = 0.2, ANGULAR_SPEED = pi/4
   # velocity_vector = [1, 0] for positive linear and no angular movement
   # velocity_vector = [-1, 1] for negative linear and positive angular movement
   # we can then create a dictionary state: movement_vector to hash the current position to get the movement_vector
   # in order to get the zig zag and spiral motion you could you something like this:
   # twist.linear.x = LINEAR_SPEED * linear_component * linear_transform
   # twist.angular.z = ANGULAR_SPEED * angular_component * angular_transform
   # where the [linear_transform, angular_transform] is derived from another source that is based on the clock
   # now you can change the velocity of the robot at every step of the program based on the state and the time
   cmd_vel_pub.publish(t)

   # run at 10hz
   rate.sleep()