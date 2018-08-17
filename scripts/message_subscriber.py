#!/usr/bin/env python
import rospy
from rosbook.msg import Complex

def callback(msg):
    print 'Real:', msg.real
    print 'Imaginary:', msg.imaginary
    print
    
print "starting message subscriber"
rospy.init_node('message_subscriber')
sub = rospy.Subscriber('complex', Complex, callback)
rospy.spin()