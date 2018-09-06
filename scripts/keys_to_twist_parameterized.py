#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = { 'w': [ 0, 1], 'x': [0, -1],
                'a': [-1, 0], 'd': [1,  0],
            's':[0,0]}

g_last_twist = None

# Default values for linear and angular. These will be multiplied by 1, 0 or -1
g_vel_scales = [0.1, 0.1] # default to very slow

def keys_cb(msg, twist_pub):
    global g_last_twist, g_vel_scales

    # Allow for blank data from keys topic, or a key that has no mapping
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return # unknown key
    
    # Based on key grab 0, 1 or -1 for linear and angular
    vels = key_mapping[msg.data[0]]

    # Multiply multiplier by absolute value in g_vel_scales
    g_last_twist.angular.z = vels[0] * g_vel_scales[0]
    g_last_twist.linear.x = vels[1] * g_vel_scales[1]
    
    # And publish cmd_vel
    twist_pub.publish(g_last_twist)

# Main program
if __name__ == '__main__':

    # Declare node-hood
    rospy.init_node('keys_to_twist')

    # Declare that want to publish cmd_vel
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Declare subsription to keys topic
    rospy.Subscriber('keys', String, keys_cb, twist_pub)
    
    # Create a blank twist
    g_last_twist = Twist() # initializes to zero

    # Based on param either on command line, grab the absolute values for linear and angular
    if rospy.has_param('~linear_scale'):
        g_vel_scales[1] = rospy.get_param('~linear_scale')
    else:
        rospy.logwarn("linear scale not provided; using %.1f" % g_vel_scales[1])
    
    # Based on param either on command line, grab the absolute values for linear and angular
    if rospy.has_param('~angular_scale'): 
        g_vel_scales[0] = rospy.get_param('~angular_scale')
    else:
        rospy.logwarn("angular scale not provided; using %.1f" % g_vel_scales[0])
    
    rate = rospy.Rate(10)

    # Publish whatever the current desired g_last_twist is
    while not rospy.is_shutdown():
        twist_pub.publish(g_last_twist)
        rate.sleep()