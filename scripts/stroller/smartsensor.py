#!/usr/bin/env python

import rospy
import basenode
from prrexamples.msg import Sensor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from stroller_kalman import kalman_predict, kalman_update
from math import pi
from marker_array_utils import MarkerArrayUtils
from std_msgs.msg import ColorRGBA

class SmartSensor(BaseNode):

    GAZEBO=False
    FRONT_BEAR=0
    RIGHT_BEAR=90
    REAR_BEAR=180
    LEFT_BEAR=270

    def filter(self, a, i):
        """Given an array a and an index i, replace each entry in the array with the average
        of three adjacent cells, while filtering out values that are less than 0.05 or np.inf"""
        x = np.take(a, [i, i-1, i+1], mode='wrap')
        y = np.logical_and(x>0.05, x!=np.inf)
        slice = x[y]
        if slice.size == 0:
            return np.NAN
        else:
            return np.average(x[y])

    def scan_callback(self, msg):
        """Return the bearing and distance to the nearest obstacle as well as the distance
        to obstacle front, rear, left and right, in radians"""
        ar = np.array(msg.ranges)
        filter_and_average = [filter(ar,x) for x in range(0, ar.size)]
        self.near_bear = np.argmin(np.around(filter_and_average, decimals=2))
        self.near_dist = filter_and_average[near_bear]
    
    # minirover's lidear (ydlidar X4) sends back 720 numbers per rotation hence the divide by two, 
    # gazebo sends back 360 numbers so it doesn't have to be divided by two
        lidar_div = ar.size/360
        self.near_bear = math.radians(near_bear/lidar_div)
        self.front_dist = filter_and_average[self.FRONT_BEAR*lidar_div]
        self.right_dist = filter_and_average[self.RIGHT_BEAR*lidar_div]
        self.left_dist = filter_and_average[self.LEFT_BEAR*lidar_div]
        self.rear_dist = filter_and_average[self.REAR_BEAR*lidar_div]
    #    print("stroller_sensor nearest: %.1f bearing: %.3f front: %.3f left: %.3f rear: %.3f right: %.3f " % (near_dist, near_bear, front_dist, left_dist, rear_dist, right_dist))

    def cmd_vel_callback(msg):
        """Whenever there's a new command for motion this will be incorporated, using the Kalman FIlter
        into the estimated nearest bearing and distance"""
        self.g_forward_cmd = msg.linear.x
        self.g_turn_cmd = msg.angular.z

    
    def pre_loop(self):
        rospy.init_node('prrsensor', log_level=rospy.DEBUG)
        self.sensor_pub = rospy.Publisher('sensor', Sensor, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
        self.rate = rospy.Rate(10)

        self.wait_for_simulator()
        self.g_time_now = rospy.Time()
        self.g_forward_cmd = 0
        self.g_turn_cmd = 0
        self.state_dist = near_dist = front_dist = rear_dist = left_dist = right_dist = 0.0
        self.near_bear = math.radians(0)
        self.state_bear = math.radians(0)

    def loop(self):
        elapsed = rospy.Time.now().to_sec() - g_time_now.to_sec()
        control_motion = g_forward_cmd * elapsed
        state_dist_temp, state_bear_temp = kalman_predict(state_dist, state_bear, control_motion)
        state_dist, state_bear = kalman_update(0.4, state_dist_temp, state_bear_temp, near_dist, near_bear)
        sensor_msg = Sensor(front_dist, left_dist, right_dist, rear_dist, state_dist, state_bear)
        sensor_pub.publish(sensor_msg)
        g_time_now = rospy.Time.now()
        rate.sleep()
    