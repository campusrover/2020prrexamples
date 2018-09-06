#!/usr/bin/env python
# import String and Twist
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Map from a letter to a the change in x for linear, and the change in z for angular
key_mapping = { 'w': [ 0, 1], 'x': [0, -1],
                'a': [-1, 0], 'd': [1,  0],
                's':[0,0]}

# Callback whenever a `keys` message is seen
def keys_cb(msg, twist_pub):
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return # unknown key

    vels = key_mapping[msg.data[0]]
    t = Twist()
    t.angular.z = vels[0]
    t.linear.x = vels[1]
    twist_pub.publish(t)

# Main program when this .py file is called directly (this is a standard python idiom)
if __name__ == '__main__':

# Declare node
    rospy.init_node('keys_to_twist')

# let me publish `cmd_vel` topic    
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# let me know when someone else publishes `keys` topic, then call keys_cb
    rospy.Subscriber('keys', String, keys_cb, twist_pub)

# Wait until ^c
    rospy.spin()