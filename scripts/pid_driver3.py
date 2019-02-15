#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rosbook.msg import Detector


class FollowWall:

    def __init__(self):
        self.goal_wall_distance = 0.5
        self.emer_stop_dist = 0.2
        self.m = None
        self.rightish = ["right", "narrow_r1", "narrow_r2", "narrow_r3"]
        self.leftish = ["left", "narrow_l1", "narrow_l2", "narrow_l3"]

    def set_state(self, new_state):
        print("new state %s" % (new_state))
        self.state = new_state

    def scan_callback(self, msg):
        self.m = msg

    def is_too_close(self):
        return (self.m.closest_dist < self.emer_stop_dist)

    def is_too_far(self):
        return self.m.closest_dist > self.goal_wall_distance

    def inside_wall_follow_zone(self):
        return self.m.closest_dist < self.goal_wall_distance

    def outside_wall_follow_zone(self):
        return self.m.closest_dist > self.goal_wall_distance

    def onleft(self):
        return self.m.closest_dir in self.leftish

    def onnarrowleft(self):
        return self.onleft() and not self.closest_dir == "left"

    def onright(self):
        return self.m.closest_dir in self.rightish

    def onfront(self):
        return self.m.closest_dir == "forward"

    def onback(self):
        return self.m.closest_dir == "back"

    def handle_find_wall(self):
        twist = Twist()
        if (self.inside_wall_follow_zone()):
            self.set_state("follow_wall")
        elif (self.onleft()):
            print("find:leftish")
            twist.linear.x = 0.05
            twist.angular.z = 0.3
        elif (self.onright() or self.onback()):
            print("find:rightish")
            twist.linear.x = 0.05
            twist.angular.z = -0.3
        else:
            twist.linear.x = 0.1
        self.cv_pub .publish(twist)


# Positive turns counterclockwise (left), negative turns clockwise (right)

    def handle_follow_wall(self):
        twist = Twist()
        if (self.is_too_close()):
            print("\nfw:tooclose")
            self.set_state("emer_stop")
        elif (self.is_too_far()):
            print("\nfw:toofar")
            self.set_state("find_wall")
        elif (self.onleft()):
            twist.linear.x = 0.05
            pid_p = self.goal_wall_distance - self.m.closest_dist
            pid_p = pid_p * -3
            if self.inside_wall_follow_zone():
                print("\nfw:onleft-inside %1.3f" % pid_p)
                twist.angular.z = pid_p
            elif self.outside_wall_follow_zone():
                print("\nfw:onleft-outside %1.3f" % pid_p)
                twist.angular.z =pid_p
        else:
            print("\nfw:wrongway")
            twist.angular.z = 0.5
        self.cv_pub.publish(twist)

    def handle_emer_stop(self):
        twist = Twist()
        self.cv_pub.publish(twist)

    def run(self):
        rospy.init_node('wallfollow', anonymous=True)
        self.cv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        mysub = rospy.Subscriber('detector', Detector, self.scan_callback)
        self.set_state("find_wall")
        rate = rospy.Rate(5)
        count_log = 0

        while not rospy.is_shutdown():
            count_log += 1
            if (count_log % 10 == 0):
                print("\n#%s [%s] %s=%1.2f" % (count_log, self.state,
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
