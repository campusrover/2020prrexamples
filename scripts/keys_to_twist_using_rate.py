#!/usr/bin/env python
import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Twist

# Map from letter input to x linear and z angular
key_mapping = { 'w': [ 0, 1], 'x': [0, -1],
                'a': [-1, 0], 'd': [1,  0],
                's':[0,0]}
g_last_twist = None

# Callback whenever a "keys" topic is published
def keys_cb(msg, twist_pub):
    # Python 2.7 feature to allow this method to access a variable in the global context
    global g_last_twist

    # Check if there's no message data or if it's not one of our keys
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return # unknown key
    
    # Fill in the cmd_vel set up on in global context below, based on the info in key_mapping.
    vels = key_mapping[msg.data[0]]
    g_last_twist.angular.z = vels[0]
    g_last_twist.linear.x = vels[1] 
    twist_pub.publish(g_last_twist)

# Main program starts executing here:

if __name__ == '__main__':
    # declare node
    rospy.init_node('keys_to_twist')

    # declare intention to publish topic `cmd_vel`
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # declare subscription to topic `keys`
    rospy.Subscriber('keys', String, keys_cb, twist_pub)

    # create a rate object for a rate of 10 per second
    rate = rospy.Rate(10)

    # Initialize with an all zeroes Twist message
    g_last_twist = Twist()
    
    # loop forever until a ^c
    while not rospy.is_shutdown():

        # Publish whatever is in g_last_twist, over and over again. Rely on the callback
        # to change the value of g_last_twist.
        twist_pub.publish(g_last_twist)

        # Sleep for a period that would result in 10 loops per second.
        rate.sleep()
