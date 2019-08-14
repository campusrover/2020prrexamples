#! /usr/bin/env python
import rospy
import time
import actionlib
from prrexamples.msg import TimerAction, TimerGoal, TimerResult

# Action Request Comes in
def do_timer(goal):
    start_time = time.time()
    time.sleep(goal.time_to_wait.to_sec())
    result = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = 0
    server.set_succeeded(result)

# Declare that we are a node
rospy.init_node('timer_action_server')

# Declare that this node will handle actions
# When action requests come in, call do_timer method
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)

# Start it up
server.start()

# Wait until ^c
rospy.spin()