import py_trees
import random
import math
import time
from py_trees import Blackboard
import bt_utils

class StrollerBt():
    def create_tree(self):
        self.tree = py_trees.composites.Selector(children=[
            py_trees.composites.Sequence(children=[
                DistanceLessThan(0.2),
                TurnAwayFromObstacle(),
                CreepForward(5)
            ]),
            py_trees.composites.Sequence(children=[
                py_trees.composites.Selector(children=[
                    CheckFacingObstacle(),
                    TurnTowardsNearestObstacle()
                ]),
                py_trees.composites.Sequence(children=[
                    py_trees.decorators.Inverter(DistanceLessThan(0.3)),
                    DriveForward(0.1)
                ])
            ]),
            py_trees.composites.Sequence(children=[
                TurnSoObstacleAt(90),
                DriveForward(0.1)
            ])
        ])

        self.tree.setup(timeout=15)
        bb = Blackboard()
        bb.desired_move = 0.0
        bb.desired_turn = 0.0
        bb.target_distance = 0
        bb.target_bearing = 0
        self.counter = 0
        py_trees.display.render_dot_tree(self.tree)
        return self.tree

    def set_sensor_data(self, target_distance, target_bearing):
        bb = Blackboard()
        bb.target_distance = target_distance
        bb.target_bearing = target_bearing

    def get_desired_motion(self):
        bb = Blackboard()
        return (bb.desired_move, bb.desired_turn)
        
    def is_shutdown(self):
        return (self.tree.status == py_trees.Status.SUCCESS)
    
    def print_status(self):
        bb = Blackboard()
        print("--------- Tick {0} ---------".format(self.counter))
        print("des mov: {0:.2f} turn: {1:.2f} target dist: {2:.2f} bearing {3:.2f}".format(bb.desired_move, bb.desired_turn, bb.target_distance, bb.target_bearing))
        py_trees.display.print_ascii_tree(self.tree, show_status=True)

    def tick_once(self):
        self.counter += 1
        self.tree.tick_once()

class TurnTowardsNearestObstacle(bt_utils.BaseBehavior):
    def update(self):
        bb = Blackboard()
        mirr_angle = self.mirror_radians(bb.target_bearing)
        if  abs(mirr_angle) < 0.15:
            self.log("Now pointing to wall", mirr_angle)
            bb.desired_move = 0.0
            bb.desired_turn = 0.0
            return (py_trees.Status.SUCCESS)
        else:
            self.log("Turning towards wall", mirr_angle)
            bb.desired_move = 0.0
            bb.desired_turn = mirr_angle / 5.0
            return(py_trees.Status.RUNNING)

class TurnSoObstacleAt(bt_utils.BaseBehavior):
    def __init__(self, angle):
        # Angle is in degrees
        self.desired = math.radians(angle)
        super(TurnSoObstacleAt, self).__init__()

    def update(self):
        bb = BlackBoard()
        actual = bb.target_bearing
    # actual is the angle of the nearest obstacle
    # desired is the desired angle to the nearest obstacle
        delta = self.angle_dif(actual, self.desired)
        bb.desired_move = 0.0
        bb.desired_turn = delta/180.0

class TurnAwayFromObstacle(bt_utils.BaseBehavior):
    def update(self):
        bb = Blackboard()
        mirr_angle = self.mirror_radians(bb.target_bearing)
        if  abs(mirr_angle) > math.radians(175):
            self.log("Now pointing away from wall", mirr_angle)
            bb.desired_move = 0.0
            bb.desired_turn = 0.0
            return (py_trees.Status.SUCCESS)
        else:
            self.log("Turning away from Obstacle", mirr_angle)
            bb.desired_move = 0.0
            bb.desired_turn = (mirr_angle - math.radians(175))
            return(py_trees.Status.RUNNING)

class CheckFacingObstacle(bt_utils.BaseBehavior):
    def update(self):
        bb = Blackboard()
        mirr_angle = self.mirror_radians(bb.target_bearing)
        if abs(mirr_angle) < 0.15:
            self.log("Facing Obstacle", mirr_angle)
            return(py_trees.Status.SUCCESS)
        else:
            self.log("Not Facing Obstacle", mirr_angle)
            return(py_trees.Status.FAILURE)

class DistanceLessThan(bt_utils.BaseBehavior):
    def __init__(self, min_distance):
        self.min_distance = min_distance
        super(DistanceLessThan, self).__init__()

    def update(self):
        bb = Blackboard()
        if bb.target_distance <= self.min_distance:
            self.log("Distance .LE. than: ", bb.target_distance)
            return(py_trees.Status.SUCCESS)
        else:
            self.log("Distance .GT.:", bb.target_distance)
            return(py_trees.Status.FAILURE)

class CreepForward(bt_utils.BaseBehavior):

    def __init__(self, loops):
        self.loops = loops
        super(CreepForward, self).__init__()

    def update(self):
        bb = Blackboard()
        bb.desired_move = 0
        bb.desired_turn = 0
        if self.loops==0:
            return(py_trees.Status.SUCCESS)
        else:
            self.log("Creeping")
            self.loops -= 1
            bb.desired_move = 0.1
            return(py_trees.Status.RUNNING)

class DriveForward(bt_utils.BaseBehavior):

    def __init__(self, loops):
        self.loops = loops
        super(DriveForward, self).__init__()

    def update(self):
        bb = Blackboard()
        bb.desired_turn = 0
        bb.desired_move = 0.1
        return(py_trees.Status.RUNNING)
