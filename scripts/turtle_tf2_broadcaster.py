#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg

# Called each time the turtle publishes a new pose. We inform tf2 that there's a new
# transform between this turtle and the world coordinate system by creating a TransformBroadcaster
# and calling sendTransform.
#
def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

# Fill in the Transform with the current time, the from and to frame_ids and
# the 7 components of the transform, three for translation and 4 (!) for rotation.

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

# Publish the trandsform.
    br.sendTransform(t)

if __name__ == '__main__':
# Create a node, grab a parameter indicating the name of the turtle, and subscribe that
# turtle's pose, /turtleX/pose. By using a parameter we can launch this node and supply 
# the turtle name later.
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = rospy.get_param('~turtle', 'turtle1')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
