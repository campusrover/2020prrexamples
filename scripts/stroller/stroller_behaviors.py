import random
import py_trees
from geometry_msgs.msg import Twist
from math import pi, radians

class Turn2Target(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.twist = Twist()
        self.countdown = 5
        super(Turn2Target, self).__init__(name)

    def setup(self, timeout):
        self.logger.debug("  %s [Turn2Target::setup()]" % self.name)
        return True

    def initialise(self):
        self.logger.debug("  %s [Turn2Target::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [TurnToTarget::update()]" % self.name)
        # self.twist = Twist()
        # delta = self.angle_diff(self.target, 0)
        # print(abs(delta))
        # if abs(delta) > (pi/30):
        #     self.twist.angular.z = delta/3.0
        #     self.twist.linear.x = 0.02
        #     return py_trees.common.Status.RUNNING
        # else:
        #     return py_trees.common.Status.SUCCESS
        self.countdown = self.countdown-1
        return (py_trees.Status.SUCCESS if self.countdown == 0 else py_trees.Status.RUNNING)
    
    def angle_diff(self, target, actual):
        if (target > pi):
            target = target - 2 * pi
        if actual > pi:
            actual = actual - 2 * pi
        return target-actual


class ApproachTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        """
        Minimal one-time initialisation. A fd rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(ApproachTarget, self).__init__(name)

    def setup(self, timeout):
        """
        When is this called?
          This function should be either manually called by your program
          or indirectly called by a parent behaviour when it's own setup
          method has been called.

          If you have vital initialisation here, a useful design pattern
          is to put a guard in your initialise() function to barf the
          first time your behaviour is ticked if setup has not been
          called/succeeded.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph.
          Good examples include:
          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
        """
        self.logger.debug("  %s [Turn2Target::setup()]" % self.name)
        return True

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug("  %s [ApproachTarget::initialise()]" % self.name)
    
    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [ApproachTarget::update()]" % self.name)
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])
        if not ready_to_make_a_decision:
            return py_trees.Status.RUNNING
        elif decision:
            self.feedback_message = "We are not bar!"
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "Uh oh"
            return py_trees.Status.FAILURE


class TrackFixedDistance(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        """
        Minimal one-time initialisation. A fd rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(TrackFixedDistance, self).__init__(name)

    def setup(self, timeout):
        """
        When is this called?
          This function should be either manually called by your program
          or indirectly called by a parent behaviour when it's own setup
          method has been called.

          If you have vital initialisation here, a useful design pattern
          is to put a guard in your initialise() function to barf the
          first time your behaviour is ticked if setup has not been
          called/succeeded.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph.
          Good examples include:
          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
        """
        self.logger.debug("  %s [TrackFixedDistance::setup()]" % self.name)
        return True

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug("  %s [TrackFixedDistance::initialise()]" % self.name)
    
    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [TrackFixedDistance::update()]" % self.name)
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])
        if not ready_to_make_a_decision:
            return py_trees.Status.RUNNING
        elif decision:
            self.feedback_message = "We are not bar!"
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "Uh oh"
            return py_trees.Status.FAILURE

