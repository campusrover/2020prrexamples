import py_trees
import random
import math
import time
from py_trees import Blackboard
import bt_utils

class StrollerBt():
    def create_tree(self):
        self.tree = py_trees.composites.Selector(children=[
# Check if we are too close
            py_trees.composites.Sequence(children=[
                DistanceLessThan(0.1),
                TurnSoObstacleAt(180),
                CreepForward(1)
            ]),
# # Check if we are too far
#             py_trees.composites.Sequence(children=[
#                 py_trees.decorators.Inverter(DistanceLessThan(0.4)),
#                 TurnSoObstacleAt(0),
#                 CreepForward(1)
#             ]),
# Here are are in the right zone
            FollowWall(angle=1.57, distance=0.35, speed=0.2)
        ])

        self.tree.setup(timeout=15)
        bb = Blackboard()
        self.counter = 0
        self.set_desired_motion(0,0)
        self.set_sensor_data(0,0)
        py_trees.display.render_dot_tree(self.tree)
        return self.tree

    def set_sensor_data(self, target_distance, target_bearing):
        bb = Blackboard()
        bb.target_distance = target_distance
        bb.target_bearing = target_bearing

    def get_desired_motion(self):
        bb = Blackboard()
        return (bb.desired_move, bb.desired_turn)

    def set_desired_motion(self, move, turn):
        bb = Blackboard()
        bb.desired_move = move
        bb.desired_turn = turn

    def is_shutdown(self):
        return (self.tree.status == py_trees.Status.SUCCESS)
    
    def print_status(self):
        bb = Blackboard()
        print("--------- Tick {0} ---------".format(self.counter))
        print("Des mov: {0:.2f},turn: {1:.2f} Target dist: {2:.2f},bearing {3:.2f}".format(bb.desired_move, bb.desired_turn, bb.target_distance, bb.target_bearing))
        #py_trees.display.print_ascii_tree(self.tree, show_status=True)

    def tick_once(self):
        self.counter += 1
        bb = Blackboard()
        bb.desired_move = 0.0
        bb.desired_turn = 0.0
        self.tree.tick_once()

class TurnSoObstacleAt(bt_utils.BaseBehavior):
    def __init__(self, angle):
        # Angle is in degrees
        self.angle = math.radians(angle)
        super(TurnSoObstacleAt, self).__init__()

    def update(self):
        bb = Blackboard()
        actual = bb.target_bearing
    # actual is the angle of the nearest obstacle
    # desired is the desired angle to the nearest obstacle
        delta = self.angle_dif(actual, self.angle)
        if (abs(delta) < 0.2):
            bb.desired_turn = 0
            bb.desired_move = 0.0
            self.log("Goal angle: %.2f delta: %.2f -> SUCCESS" % (self.angle, delta))
            return(py_trees.Status.SUCCESS)
        else:
            bb.desired_move = 0.0
            bb.desired_turn = delta/10.0
            self.log("Goal angle: %.2f delta: %.2f -> RUNNING" % (self.angle, delta))
            return(py_trees.Status.RUNNING)

class FollowWall(bt_utils.BaseBehavior):
    def __init__(self, angle, distance, speed):
        super(FollowWall, self).__init__()
        self.goal_angle = angle
        self.goal_distance = distance
        self.goal_speed = speed

    def update(self):
        bb = Blackboard()
        actual_angle = bb.target_bearing
        angle_delta = self.angle_dif(actual_angle, self.goal_angle)
        distance_delta = bb.target_distance - self.goal_distance
        bb.desired_move = max(0.01, self.goal_speed-abs(angle_delta/4.0))
        bb.desired_turn = distance_delta * 4

    # # Is robot pointed in the right direction?
    #     if (abs(angle_delta) > 0.2):
    #         bb.desired_turn = angle_delta/5.0
    # # Is robot at the right distance?
    #     distance_delta = bb.target_distance - self.goal_distance
    #     if abs(distance_delta) > 0.05:
    #         print(".")
    #         bb.desired_turn = distance_delta/10.0
    # # If we are turning strongly then reduce speed
    #     if (abs(angle_delta) > math.radians(30)):
    #         bb.desired_move = self.goal_speed/2.0
        self.log("delta dist %.2f angle: %.2f req move: %.2f turn %.2f" % (distance_delta, angle_delta, bb.desired_move, bb.desired_turn))
        return(py_trees.Status.SUCCESS)

class DistanceLessThan(bt_utils.BaseBehavior):
    def __init__(self, min_distance):
        self.min_distance = min_distance
        super(DistanceLessThan, self).__init__()

    def update(self):
        bb = Blackboard()
        if bb.target_distance <= self.min_distance:
            self.log("Distance %.2f .LE. than %.2f (SUCCESS)" % (bb.target_distance, self.min_distance))
            return(py_trees.Status.SUCCESS)
        else:
            self.log("Distance %.2f .GT. than %.2f (FAIL)" % (bb.target_distance, self.min_distance))
            return(py_trees.Status.FAILURE)

class CreepForward(bt_utils.BaseBehavior):
    def __init__(self, loops):
        print("creep init")
        self.loops = loops
        super(CreepForward, self).__init__()

    def initialise(self):
        print("creep initialise")
        super(CreepForward, self).initialise()
        self.counter = self.loops
        return True

    def update(self):
        print("creep update",)
        bb = Blackboard()
        bb.desired_move = 0
        bb.desired_turn = 0
        if self.counter==0:
            return(py_trees.Status.SUCCESS)
        else:
            self.log("Creeping")
            self.counter -= 1
            bb.desired_move = 0.07
            return(py_trees.Status.RUNNING)
