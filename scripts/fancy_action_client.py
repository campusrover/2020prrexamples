#! /usr/bin/env python
import rospy
import time
import actionlib
from prrexamples.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

def feedback_cb(feedback):
    rospy.loginfo('[Feedback] Time elapsed: %f'%(feedback.time_elapsed.to_sec()))
    rospy.loginfo('[Feedback] Time remaining: %f'%(feedback.time_remaining.to_sec()))

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', TimerAction)

rospy.loginfo("Waiting for server...")
client.wait_for_server()
rospy.loginfo("Server running...")

goal = TimerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5.0)

# Uncomment this line to test server-side abort:
# goal.time_to_wait = rospy.Duration.from_sec(500.0)
client.send_goal(goal, feedback_cb=feedback_cb)

# Uncomment these lines to test goal preemption:
# time.sleep(3.0)
# client.cancel_goal()

client.wait_for_result()
rospy.loginfo('[Result] State: %d'%(client.get_state()))
rospy.loginfo('[Result] Status: %s'%(client.get_goal_status_text()))
rospy.loginfo('[Result] Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))
rospy.loginfo('[Result] Updates sent: %d'%(client.get_result().updates_sent))