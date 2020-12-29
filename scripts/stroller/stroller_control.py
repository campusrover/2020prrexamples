#!/usr/bin/env python
import rospy
from prrexamples.msg import Sensor
from std_msgs.msg import ColorRGBA
from math import pi, radians
from marker_array_utils import MarkerArrayUtils
from geometry_msgs.msg import Twist
import stroller_bt

GAZEBO=True
FRONT_BEAR=0
RIGHT_BEAR=90
REAR_BEAR=180
LEFT_BEAR=270

green = ColorRGBA(0, 1, 0, 1)
red = ColorRGBA(1, 0, 0, 1)
grey = ColorRGBA(0.6, 0.6, 0.6, 1)

def invert_angle(angle):
    """To make the arrow markers work correctly we need to flip the angle because angles go the other way there"""
    if (not GAZEBO):
        return (angle + pi) % (2 * pi)
    else:
        return angle

def draw_markers(forward, left, right, rear, shortest_bearing, shortest):
    mu = MarkerArrayUtils()
    # mu.add_marker(1, grey, invert_angle(radians(FRONT_BEAR)), forward)
    # mu.add_marker(2, grey, invert_angle(radians(LEFT_BEAR)), left)
    # mu.add_marker(3, grey, invert_angle(radians(RIGHT_BEAR)), right)
    # mu.add_marker(4, grey, invert_angle(radians(REAR_BEAR)), rear)
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

def shutdown_hook():
    stop()
    print("\n*** Shutdown Requested ***")

rospy.init_node('stroller_control')
sensor_sub = rospy.Subscriber('/sensor', Sensor,  sensor_callback)
command_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
target_distance = target_bearing = 0
rate = rospy.Rate(2)
rospy.on_shutdown(shutdown_hook)
twist = Twist()
sbc = stroller_bt.StrollerBt()
sbc.create_tree()


# Wait for the simulator to be ready
while rospy.Time.now().to_sec() == 0:
    rate.sleep()

while not (rospy.is_shutdown()):
    try:
        sbc.print_status()
        sbc.set_sensor_data(target_distance, target_bearing)
        sbc.tick_once()
        move, turn = sbc.get_desired_motion()
        if (move != twist.linear.x or turn != twist.angular.z):
            twist.linear.x = move
            twist.angular.z = turn
            command_vel_pub.publish(twist)
            #print("fwd: %.2f turn: %.2f " % (twist.linear.x, twist.angular.z))
        rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        break
