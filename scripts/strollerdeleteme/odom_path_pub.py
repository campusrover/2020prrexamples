#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def shutdown_hook():
    print("\n*** Shutdown Requested ***")

def odom_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)

if __name__ == "__main__":

    rospy.init_node('path_node')
    rospy.on_shutdown(shutdown_hook)
    path = Path()
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        try:
            path_pub.publish(path)
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break
