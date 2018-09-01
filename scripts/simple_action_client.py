#! /usr/bin/env python

import rospy
import actionlib
from rosbook.msg import TimerAction, TimerGoal, TimerResult

# Declare the node
rospy.init_node('timer_action_client')

# Get the method to talk to the action
client = actionlib.SimpleActionClient('timer', TimerAction)

# Now just wait for it  to come up.
client.wait_for_server()

# Create the TimerGoal objet
goal = TimerGoal()

# Set it up
goal.time_to_wait = rospy.Duration.from_sec(5.0)

# And now tell the action to begin working on the goal
client.send_goal(goal)

# Block until the action says the job is done
client.wait_for_result()

# Print the result.
print('Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))