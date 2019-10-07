#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

# Create a node and allocate a tf buffer and a tf listener.

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

# Advanced trick to launch a second copy of the turtlesim named after
# the param turtle (default to turtle2)
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    turtle_name = rospy.get_param('turtle', 'turtle2')
    spawner(4, 2, 0, turtle_name)

# We are going to steer turtle2. We start by creating a turtle2/cmd_vel publisher
    turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

# And we loop, 10x per second
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
# This is the most important line. Requests the transform between turtle1 and turtle_name
            trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()

# Some trig to compute the desired motion of turtle2
        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

# And publish it to drive turtle2
        turtle_vel.publish(msg)

        rate.sleep()
