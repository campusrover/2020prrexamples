import py_trees
import random
import time
from py_trees import Blackboard
import bt_utils

class StrollerBt():
    def create_tree(self):
        #range_safety = py_trees.composites.Selector("rangsaf")
        #range_safety.add_child(TooClose("tooclose"))
        #range_safety.add_child(MoveAwayFromWall("awaywall"))
        self.tree = py_trees.composites.Selector(name="orient")
        # tree.add_child(range_safety)
        self.tree.add_child(CheckFacingObstacle("chkfacing"))
        self.tree.add_child(TurnTowardsNearestObstacle("turn2obs"))
        self.tree.setup(timeout=15)
        bb = Blackboard()
        bb.desired_move = 0.0
        bb.desired_turn = 0.0
        bb.target_distance = 0
        bb.target_bearing = 0
        self.counter = 0
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

class CheckFacingObstacle(bt_utils.BaseBehavior):
    def update(self):
        bb = Blackboard()
        mirr_angle = self.mirror_radians(bb.target_bearing)
        if abs(mirr_angle) < 0.15:
            return(py_trees.Status.SUCCESS)
        else:
            return(py_trees.Status.FAILURE)

    