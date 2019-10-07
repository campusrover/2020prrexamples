#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class FixedTFBroadcaster:

# Class has just a constructor.

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

# Loop forever, 10 times per second.
        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

# Create a transform from turtle1 to carrot1
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "turtle1"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "carrot1"
# Carrot is t meters left of turtle1
            t.transform.translation.x = 0.0
            t.transform.translation.y = 2.0
            t.transform.translation.z = 0.0
# And no rotational transformation
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
# Convert the TFStampted into a message and publish it.
# 10 times per second but the same. over and over again.
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

# Here's the main program. Define a node and instantiate FixedTFBroadcaster
if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()
