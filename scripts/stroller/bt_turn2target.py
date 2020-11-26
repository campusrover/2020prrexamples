from geometry_msgs.msg import Twist
import py_trees
from math import pi, radians

class TurnToTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Foo, self).__init__(name)
    
    def setup(self):
        self.logger.debug("  %s [TurnToTarget::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [TurnToTarget::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [TurnToTarget::update()]" % self.name)
        self.twist = Twist()
        delta = self.angle_diff(self.target, 0)
        print(abs(delta))
        if abs(delta) > (pi/30):
            self.twist.angular.z = delta/3.0
            self.twist.linear.x = 0.02
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
        
    def angle_diff(self, target, actual):
        if (target > pi):
            target = target - 2 * pi
        if actual > pi:
            actual = actual - 2 * pi
        return target-actual

    def terminate(self, new_status):
        self.logger.debug("  %s [TurnToTarget::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
