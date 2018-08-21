#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.init_node('red_light_green_light')

red_light_twist = Twist()
green_light_twist = Twist()
green_light_twist.linear.x = 2.5

driving_forward = False
light_change_time = rospy.Time.now()
print (rospy.Time.now())
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    print(light_change_time, rospy.Time.now(), light_change_time < rospy.Time.now())
    if driving_forward:
        print("green", green_light_twist.linear.x)
        cmd_vel_pub.publish(green_light_twist)
    else:
        print("red")
        cmd_vel_pub.publish(red_light_twist)
    if light_change_time < rospy.Time.now():
        print("flip")
        driving_forward = not driving_forward
        light_change_time = rospy.Time.now() + rospy.Duration(3)
    rate.sleep()