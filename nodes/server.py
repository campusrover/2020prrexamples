#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from prrexamples.cfg import FollowerConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Kp}, {Td},{Ti}, {dt}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(FollowerConfig, callback)
    rospy.spin()

