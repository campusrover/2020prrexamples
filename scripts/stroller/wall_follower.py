#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import sqrt, cos, sin, pi, atan2, isnan
import numpy
import sys
from prrexamples.msg import Sensor
from std_msgs.msg import ColorRGBA
from math import pi, radians
from marker_array_utils import MarkerArrayUtils

GAZEBO=False
FRONT_BEAR=0
RIGHT_BEAR=90
REAR_BEAR=180
LEFT_BEAR=270

green = ColorRGBA(0, 1, 0, 1)
red = ColorRGBA(1, 0, 0, 1)
grey = ColorRGBA(0, 0, 0, 1)


from dynamic_reconfigure.server import Server
from prrexamples.cfg import FollowerConfig

def invert_angle(angle):
    """To make the arrow markers work correctly we need to flip the angle because angles go the other way there"""
    if (not GAZEBO):
        return (angle + pi) % (2 * pi)
    else:
        return angle

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_control(self, current_error, reset_prev=False):
        
        self.prev_error = self.curr_error
        self.curr_error = current_error
        
        #Calculating the integral error
        self.sum_error = self.sum_error + self.curr_error*self.dt

        #Calculating the derivative error
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        #Calculating the PID Control
        self.control = self.Kp * self.curr_error + self.Ti * self.sum_error + self.Td * self.curr_error_deriv
        print "pid (%.1f,%.1f,%.1f) curr_err: %2f, control: %.2f" % (self.Kp, self.Ti, self.Td, self.curr_error, self.control)

    def get_control(self):
        return self.control
        
class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.hz = 20
        self.distance_from_left_wall = 0
        self.shutdown_requested = False

        #Setting the PID
        self.controller=PID(-1,-1.2, 0.00001,0.1)
        self.update_params()

        #Creating a cte_pub Publisher that will publish the cross track error
        self.cte_pub = rospy.Publisher("/cte", String, queue_size=50)

        #Creating cmd_pub Publisher that will publish a Twist msg to cmd_vel
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
        
        self.msg = Twist()
        self.msg.linear.x = self.forward_speed

        #Publish the msg
        self.cmd_pub.publish(self, self.msg)
        
        #Creating a Subscriber that will call laser_scan_callback every Laser Scan
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)

        self.sensor_sub = rospy.Subscriber('/sensor', Sensor,  self.sensor_callback)

    def draw_markers(self, forward, left, right, rear, shortest_bearing, shortest):
        mu = MarkerArrayUtils()
        mu.add_marker(5, red, invert_angle(shortest_bearing), shortest)
        mu.add_marker(2, grey, invert_angle(radians(LEFT_BEAR)), left)
        mu.add_marker(3, green, invert_angle(radians(RIGHT_BEAR)), right)

        mu.publish()

    def sensor_callback(self, msg):
        global target_bearing, target_distance
        self.draw_markers(msg.forward, msg.left, msg.right, msg.rear, msg.shortest_bearing, msg.shortest)
        if isnan(msg.left):
            return
        self.distance_from_left_wall = msg.left
        print("left: %.2f dfl: %.2f ddfw: %.2f" % (msg.left, self.distance_from_left_wall, self.desired_distance_from_wall))

    def laser_scan_callback(self, msg):
        cross_track_error=0
        self.update_params()

       	#Calculating the cross track error
       	cross_track_error = self.desired_distance_from_wall - self.distance_from_left_wall
        
       	#Updating the controller and publishing the cross track error
       	self.controller.update_control(cross_track_error)
       	self.cte_pub.publish(str(cross_track_error))

       	cmd = Twist()
        cmd.linear.x = self.forward_speed

  		#Getting the new cmd.angular.z
       	cmd.angular.z = self.controller.get_control()
       	
       	#Publishing the cmd.linear.x
        print "lin: %.2f ang: %.2f" % (cmd.linear.x, cmd.angular.z)
       	self.cmd_pub.publish(cmd)
   
            
    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown() and not self.shutdown_requested:
            rate.sleep()
        self.stop()
        
    # Update latest params
    def update_params(self):
        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
    	self.controller.Kp = rospy.get_param("~Kp")
    	self.controller.Td = rospy.get_param("~Td")
    	self.controller.Ti = rospy.get_param("~Ti")
        self.controller.dt = rospy.get_param("~dt")
    
    def stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

    def shutdown_hook(self):
        self.shutdown_requested = True
        print("\n**** Shutdown Requested ****")
        self.stop()


if __name__ == '__main__':
    wfh = WallFollower()
    rospy.on_shutdown(wfh.shutdown_hook)
    wfh.run()

