#! /usr/bin/env python
import rospy
import time
import actionlib
from prrexamples.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

# Notice there is a single callback which uses conditionals to determine what kind of 
# request is being sent. It is called just once, when the action is requested!
# (This is not so obvious)
def do_timer(goal):
    start_time = time.time()
    update_count = 0

    # Check for an invalid request and just abort the action with an error message
    # if there is a problem
    if goal.time_to_wait.to_sec() > 60.0:
        result = TimerResult()
        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        result.updates_sent = update_count
        server.set_aborted(result, "Timer aborted due to too-long wait")
        return

    # Here we actually do the action. We know that this can take a while but we just 
    # loop here. In this example the action is simply to delay "time_to_wait" seconds.
    # We loop here until that amount of time has passed.
    while (time.time() - start_time) < goal.time_to_wait.to_sec():

        # While we are in the loop, the requester can change their mind and ask for the action to just be aborted.
        if server.is_preempt_requested():
            result = TimerResult()
            result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
            result.updates_sent = update_count
            server.set_preempted(result, "Timer preempted")
            return

        # Otherwise we compute how close to the goal we are, create a "TimerFeedback" message to report progress
        feedback = TimerFeedback()
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed

        # Publish the feedback
        server.publish_feedback(feedback)
        update_count += 1

        # And wait one second before we check again
        time.sleep(1.0)

    # Here we have succeeded on the requested action and so report.
    result = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = update_count
    server.set_succeeded(result, "Timer completed successfully")

# Main program begins here:
rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
server.start()
rospy.spin()
