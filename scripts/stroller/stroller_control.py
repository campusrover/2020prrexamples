#!/usr/bin/env python3

import rospy
from prrexamples.msg import Sensor
from std_msgs.msg import ColorRGBA
from math import pi, radians
from marker_array_utils import MarkerArrayUtils
from geometry_msgs.msg import Twist
import py_trees.blackboard as bb
import py_trees.logging as log
import stroller_behaviors

GAZEBO=True
FRONT_BEAR=0
RIGHT_BEAR=90
REAR_BEAR=180
LEFT_BEAR=270

green = ColorRGBA(0, 1, 0, 1)
red = ColorRGBA(1, 0, 0, 1)
blue1 = ColorRGBA(0, 0, 1, 1)
blue2 = ColorRGBA(0, 0, 1, 0.75)
blue3 = ColorRGBA(0, 0, 1, 0.5)
blue4 = ColorRGBA(0, 0, 1, 0.25)

def invert_angle(angle):
    """To make the arrow markers work correctly we need to flip the angle because angles go the other way there"""
    if (not GAZEBO):
        return (angle + pi) % (2 * pi)
    else:
        return angle

def draw_markers(forward, left, right, rear, shortest_bearing, shortest):
    mu = MarkerArrayUtils()
    mu.add_marker(1, blue1, invert_angle(radians(FRONT_BEAR)), forward)
    mu.add_marker(2, blue4, invert_angle(radians(LEFT_BEAR)), left)
    mu.add_marker(3, blue2, invert_angle(radians(RIGHT_BEAR)), right)
    mu.add_marker(4, blue3, invert_angle(radians(REAR_BEAR)), rear)
    mu.add_marker(5, red, invert_angle(shortest_bearing), shortest)
    mu.publish()

def sensor_callback(msg):
    global target_bearing, target_distance
    draw_markers(msg.forward, msg.left, msg.right, msg.rear, msg.shortest_bearing, msg.shortest)
    target_bearing = msg.shortest_bearing
    target_distance = msg.shortest

def stop():
    global command_vel_pub
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    command_vel_pub.publish(twist)
    print("done")

def create_bt_root():
    my_bb = bb.Client(name="Configuration")
    my_bb.register_key(key="angular_z", access=py_trees.common.Access.WRITE)
    my_bb.register_key(key="linear_x", access=py_trees.common.Access.WRITE)
    my_bb.angular_z = 0
    my_bb.linear_x = 0
    root = py_trees.composites.Sequence("Find Wall")
    root.add_child(Turn2Target())
    root.add_child(ApproachTarget())
    root.add_child(TrackFixedDistance())
    return root

rospy.init_node('stroller_control')
sensor_sub = rospy.Subscriber('/sensor', Sensor,  sensor_callback)
command_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
target_distance = target_bearing = 0
rate = rospy.Rate(1)
twist = Twist()
log.level = log.Level.DEBUG
bt_root = create_bt_root()
bt.root.setup_with_descendants()


# Wait for the simulator to be ready
while rospy.Time.now().to_sec() == 0:
    rate.sleep()

while not rospy.is_shutdown():
    try:
        bt_root.tick_once()
    except Exception as e:
        print(e.message)
        stop()
        break



