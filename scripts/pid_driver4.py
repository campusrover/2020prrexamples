#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from prrexamples.msg import Detector


class FollowWall:

    def __init__(self):
        self.goal_wall_distance = 0.7
        self.emer_stop_dist = 0.2
        self.m = None
        self.rightish = ["right", "narrow_r1", "narrow_r2", "narrow_r3"]
        self.leftish = ["left", "narrow_l1", "narrow_l2", "narrow_l3"]

    def log(self, message):
        rospy.loginfo(message)

    def set_state(self, new_state):
        self.log("NEW STATE: %s" % (new_state))
        self.state = new_state

    def scan_callback(self, msg):
        self.log("CB")
        self.m = msg
    
    def is_too_close(self):
        return (self.m.closest_dist < self.emer_stop_dist)

    def is_too_far(self):
        return self.m.closest_dist > self.goal_wall_distance*2

    def inside_wall_follow_zone(self):
        return self.m.closest_dist < self.goal_wall_distance

    def outside_wall_follow_zone(self):
        return self.m.closest_dist > self.goal_wall_distance

    def wall_onleft(self):
        return self.m.closest_dir in self.leftish

    def wall_onnarrowleft(self):
        return self.wall_onleft() and not self.m.closest_dir == "left"

    def wall_onright(self):
        return self.m.closest_dir in self.rightish

    def wall_onforward(self):
        return self.m.closest_dir == "forward"

    def wall_onback(self):
        return self.m.closest_dir == "back"

    def at_right_inside_corner(self):
        return self.m.forward < self.m.narrow_l2

# Positive turns counterclockwise (left), negative turns clockwise (right)

    def handle_follow_wall(self):
        self.log("follow_wall")
        twist = Twist()
        if (self.is_too_close()):
            self.log("FOLLOW WALL: tooclose")
            self.set_state("emer_stop")
        elif (self.is_too_far()):
            self.log("FOLLOW WALL: toofar")
            self.set_state("find_wall")
        elif (self.at_right_inside_corner()):
            self.log("FOLLOW WALL: corner")
            self.set_state("decision_point")
        elif (self.wall_onnarrowleft()):
            g = self.goal_wall_distance
            w1 = self.m.narrow_l1
            w2 = self.m.narrow_l2
            w3 = self.m.narrow_l3
            pid_p = (w1 - w3) + (w2 - g) * 0.15
            twist.linear.x = 0.2
            twist.angular.z = pid_p * 7
            self.log("FOLLOW WALL: following %1.3f" % pid_p)
        else:
            self.log("FOLLOW WALL: wrongway")
            twist.angular.z = -0.5
        self.cv_pub.publish(twist)

    def handle_decision_point(self):
        twist = Twist()
        if (self.is_too_close()):
            self.log("DECISION POINT: tooclose")
            self.set_state("emer_stop")
        if self.wall_onnarrowleft():
            self.log("DECISION POINT: back to following")
            self.set_state("follow_wall")
        else:
            self.log("DECISION POINT: spin in place")
            twist.angular.z = -0.3
        self.cv_pub.publish(twist)
    
    def handle_find_wall(self):
        self.log("handle_find_wall")
        twist = Twist()
        if (not self.is_too_far()):
            self.set_state("follow_wall")
        else:
            twist.linear.x = self.m.closest_dist / 10.0
            self.log("FINDWALL: go fast %1.2f %1.2f" %
                     (twist.linear.x, twist.angular.z))
        self.cv_pub.publish(twist)

    def handle_emer_stop(self):
        twist = Twist()
        if (self.m.closest_dist > self.goal_wall_distance):
            self.set_state("find_wall")
        self.cv_pub.publish(twist)

    def run(self):
        rospy.init_node('wallfollow', log_level=rospy.DEBUG)
        self.cv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        mysub = rospy.Subscriber('detector', Detector, self.scan_callback)
        self.set_state("find_wall")
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            if (self.m is None):
                self.log("no state")
            elif (self.state == "find_wall"):
                self.handle_find_wall()
            elif (self.state == "follow_wall"):
                self.handle_follow_wall()
            elif (self.state == "decision_point"):
                self.handle_decision_point()
            elif (self.state == "emer_stop"):
                self.handle_emer_stop()
            else:
                self.log("LOOP: Invalid state")
            rate.sleep()


if __name__ == '__main__':
    fw = FollowWall()
    fw.run()
