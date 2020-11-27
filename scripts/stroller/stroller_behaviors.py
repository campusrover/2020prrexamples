import random as r
import py_trees

class Turn2Target(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        print ("init ", name)
        super(Turn2Target, self).__init__(name)

    def setup(self):
        print ("setup turn2target")

    def initialise(self):
        print ("initialise turn2target")

    def update(self):
        print ("update turn2target")
        return py_trees.common.Status.SUCCESS

class ApproachTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        print ("init ", name)
        super(ApproachTarget, self).__init__(name)

    def setup(self):
        print ("setup ApproachTarget")

    def initialise(self):
        print ("initialise ApproachTarget")

    def update(self):
        print ("update ApproachTarget")
        return py_trees.common.Status.SUCCESS

class TrackFixedDistance(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        print ("init ", name)
        super(TrackFixedDistance, self).__init__(name)

    def setup(self):
        print ("setup TrackFixedDistance")

    def initialise(self):
        print ("initialise TrackFixedDistance")

    def update(self):
        print ("update TrackFixedDistance")
        return py_trees.common.Status.SUCCESS if r.random > 0.5 else py_trees.common.Status.CONTINUE





