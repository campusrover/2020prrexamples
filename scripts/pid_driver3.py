#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rosbook.msg import Detector


class FollowWall:

    def __init__(self):
        self.goal_wall_distance = 0.5
        self.emer_stop_dist = 0.2
        self.m = None

    def set_state(self, new_state):
        print("new state %s" % (new_state))
        self.state = new_state

    def scan_callback(self, msg):
        self.m = msg

    def handle_find_wall(self):
        rightish = ["right", "narrow_r1", "narrow_r2", "narrow_r3"]
        leftish = ["left", "narrow_l1", "narrow_l2",
                    "narrow_l3"]
        twist = Twist()
        if (self.m.closest_dist < self.goal_wall_distance * 1.25):
            self.set_state("follow_wall")
        elif (self.m.closest_dir in leftish):
            twist.angular.z = 0.3
        elif (self.m.closest_dir in rightish or m.closest_dir == "back"):
            twist.angular.z = -0.3
        else:
            twist.linear.x = 0.1
        self.cv_pub .publish(twist)

# Positive turns counterclockwise (left), negative turns clockwise (right)
    def handle_follow_wall(self):
        twist = Twist()
        if (self.m.closest_dist < self.emer_stop_dist):
            print("1")
            self.set_state("emer_stop")
        elif (self.m.closest_dist < self.goal_wall_distance):
            print("2")
            twist.angular.z = -0.1
        elif (self.m.closest_dist > self.goal_wall_distance * 1.25):
            print("3")
            self.set_state("find_wall")
        elif (self.m.closest_dist > self.goal_wall_distance):
            print("4")
            twist.linear.x = 0.02
        elif (abs(self.m.closest_dist, self.goal_wall_distance) < 0.05):
            print("5")
            twist.linear.x = 0.05
        self.cv_pub.publish(twist)
    
    def handle_emer_stop(self):
        twist = Twist()
        self.cv_pub.publish(twist)

    def run(self):
        rospy.init_node('wallfollow', anonymous=True)
        self.cv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        mysub = rospy.Subscriber('detector', Detector, self.scan_callback)
        self.set_state("find_wall")
        rate = rospy.Rate(10)
        count_log = 0

        while not rospy.is_shutdown():
            count_log += 1
            if (count_log % 10 == 0):
                print("#%s [%s] %s=%1.2f" % (count_log, self.state,
                                             self.m.closest_dir, self.m.closest_dist))
            if (self.m is None):
                pass
            elif (self.state == "find_wall"):
                self.handle_find_wall()
            elif (self.state == "follow_wall"):
                self.handle_follow_wall()
            elif (self.state == "emer_stop"):
                self.handle_emer_stop()
            rate.sleep()


if __name__ == '__main__':
    fw = FollowWall()
    fw.run()
