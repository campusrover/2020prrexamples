#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image

# scan callback
def scan_callback(msg):
    global ranges
    ranges = msg.ranges

# img callback
def img_callback(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg)

# update state 
def update_state():
    global current_state

    # once we have received an image and scan start change state to forward
    if img is not None and ranges is not None:
        current_state = "FORWARD"

# pub/subs
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# init node 
rospy.init_node('racer')
rate = rospy.Rate(10)

# some variables
angular_vel = math.pi /2    # radians/second
linear_vel  = 0.2           # meters/second
states = {"STOP"    : (0, 0), 
          "FORWARD" : (1, 0),
          "LEFT"    : (1, 1),
          "RIGHT"   : (1, -1)}
current_state = "STOP"
ranges = None; img = None
bridge = cv_bridge.CvBridge()

# idea: keep control loop simple and change current_state
#       based on LiDAR and img data. Also I would suggest that 
#       your team uses the update state function so you can 
#       update state at every step of the program based on 
#       the LiDAR and camera data.

# control loop
while not rospy.is_shutdown():

    # print current state
    print(current_state)

    # change cmd_vel based on motion associated with the current state
    t = Twist()
    update_state()
    motion = states.get(current_state, (0, 0))
    t.linear.x = linear_vel * motion[0]
    t.angular.z = angular_vel * motion[1]

    # publish velocity command
    cmd_vel_pub.publish(t)

    # runs at 10hz
    rate.sleep()