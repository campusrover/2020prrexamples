import py_trees
from math import pi

class BaseBehavior(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = type(self).__name__
        super(BaseBehavior, self).__init__(name)
        #self.log("__init__")

    def setup(self, timeout=15):
        super(BaseBehavior, self).setup(timeout)
        #self.log("setup")
        return True
    
    def mirror_radians(self, angle):
        """Given an angle compute the corresponding mirror angle as if
        positive was clockwise and negative by counterclockwise"""
        if angle < pi:
            return angle
        else:
            return angle-2*pi

    def angle_dif(self, actual, desired):
        """Given an actual and desired angle in radians, return a positive or 
        negative radian result indicating the angle to turn to bring the angle
        to the desired"""

        diff = actual - desired
        if diff > 180:
            diff = 360-diff
        elif diff < -180:
            diff = 360+diff
        return diff

    def log(self, str, vals = None):
        """ Log with an optional parameter. A central place to turn it on and off."""
        if True:
            if (vals is  None):
                print(("%s: " + str) % self.name)
            else:
                print(("%s: "+str+" %s") % (self.name, vals))


    
    
