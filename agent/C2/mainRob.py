import logging
import logging.config
import math
import sys
import xml.etree.ElementTree as ET
from typing import Dict, Tuple

import yaml

import control_action
import utils
import ternary_tree
from ternary_tree import TreeNode
from utils import get_key
from croblink import *

CELLROWS = 7
CELLCOLS = 14

# Helper variables
axis_angles: Dict[int, str] = {90: "north", 0: "east", -90: "south", -180: "west"}
sides: Dict[int, str] = {90: "left", -90: "right", 0: "front", -180: "back"}
compass_axis: Tuple[str, str, str, str] = ("west", "south", "east", "north")


class MyRob(CRobLinkAngs):
    # Logging configuration load
    with open('C2/logging.yml', 'r') as stream:
        config = yaml.load(stream, Loader=yaml.FullLoader)
    logging.config.dictConfig(config)

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # FIX self.finish exception: (AttributeError: 'MyRob' object has no attribute 'rob_name')
        self.rob_name = rob_name
        # Logger(s) instantiation
        self.logger = logging.getLogger("wander")
        # Type of movement that the robot is
        #  doing (around what axis, e.g.: up/down (y), sideways (x) or rotation (angle))
        self.axis: str = "None"
        # Relative movement action that the robot should do
        #   e.g.: turn left/right/back or go front
        self.action: str = "starting"
        self.init_pose: dict = {}  # Initial gps coordinates
        # Controllers
        self.xy_control = control_action.ControlAction(ki=.0, td=.0, kp=12.0)  # translation
        self.rot_control = control_action.ControlAction(ki=.0, td=.0, kp=0.10)  # rotation
        # Holds the goal target pose for the robot for each movement
        # This variable complements "axis" and "action"
        self.target_pose: Dict[str, float] = {"x": .0, "y": .0, "turn": .0}
        # Holds the relative gps coordinates to the spawn coordinates
        self.feedback: Dict[str, float] = {}
        # TreeNode related vars
        self.root: [TreeNode, None] = None
        self.node_pointer: [TreeNode, None] = None  # Helper pointer to nodes
        # Keeping track of where the robot came from (using the tree struct analogy)
        # -1 -> not set; 0 -> from a child node; 1, 2, 3 -> from parent node's first, second or third branch
        self.travel_to: int = -1

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    #  to know if there is a wall on top of cell(i,j) (i in 0..5),
    #  check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        # Get starting pose
        self.readSensors()
        self.init_pose: Dict[str, float] = dict(x=self.measures.x, y=self.measures.y)

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed:
                    state = 'wait'
                if self.measures.ground == 0:
                    self.setVisitingLed(True)
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.wander()

    def wander(self):

        self.logger.debug("Wander Iteration Init")

        # Update feedback value (GPS + compass)
        self.feedback = {
            "x": (self.measures.x - self.init_pose["x"]),
            "y": (self.measures.y - self.init_pose["y"]),
            "turn": self.measures.compass
        }

        self.logger.debug(f"Current position: x={round(self.feedback['x'], 3)}, y={round(self.feedback['y'], 3)},"
                          f" angle={round(self.feedback['turn'], 3)}.")
        self.logger.debug(f"Target pose: x={self.target_pose['x']}, y={self.target_pose['y']},"
                          f" angle={self.target_pose['turn']}.")

        # If close enough to the target cell, proceed
        if self.is_micro_action_complete():
            # Decide next actions (move 1 cell front or turn)
            #   based on the walls detected
            self.next_micro_action()

        # Move
        self.do_micro_action()

    def next_micro_action(self):
        """
        This function should give orders whether to:
            * Rotate (left, right or back)
            * Move (one cell forward)
        Based on:
            * The obstacle sensors
            * The moving mode (explore or go to certain position)

        :return: action order
        """

        possible_actions: tuple = ("left", "right", "back", "front")

        # Walls
        walls: Dict[str, int] = self.create_micro_map(root=False)

        # Assert the dictionary's keys and values are as expected
        assert set(compass_axis) == set(walls.keys()), "Invalid map!"

        # Check if this crossroad is the same from last micro-iteration
        # In this case, target pose can be assumed as GPS values
        rotated_m1: bool = False
        if self.node_pointer is not None:
            rotated_m1 = self.node_pointer.coordinates \
                                 == (self.target_pose["x"], self.target_pose["y"])

        if not ternary_tree.is_crossroad_or_root(node=self.node_pointer, *walls.values()):
            # Case where it's not a crossroad
            self.move_on(walls)
        else:  # If this cell is a crossroad (multiple pathways to go) or spawn position
            if not rotated_m1:
                # TODO assure that rotated_m1 variable works as intended

                # Get a mapping of the surroundings
                walls = self.create_micro_map(root=True)
                _map = tuple(walls[axis] for axis in compass_axis)

                # Keep last iteration of self.traveled_from
                traveled_from: int = self.travel_to
                if self.root is None:  # First tree iteration
                    self.root = TreeNode(ways=_map, x=0, y=0)
                    self.node_pointer = self.root
                elif traveled_from in (1, 2, 3):  # if gone down to unknown pathways:
                    # Create new node to insert
                    x, y = (int(self.target_pose["x"]), int(self.target_pose["y"]))
                    current_node: TreeNode = TreeNode(ways=_map, x=x, y=y)
                    # Log
                    self.logger.debug(f"New node: (x,y)=({x},{y}) and map is {_map}")
                    # add node as child to previous crossroad's one
                    self.node_pointer.add_child_node(which=traveled_from, node=current_node)
                    # Update node_pointer
                    self.node_pointer = current_node
                elif traveled_from == 0:
                    # Update node_pointer
                    self.node_pointer = self.node_pointer.parent
                else:
                    raise AssertionError("Unexpected behavior! Traversed up or down?")
                # Search for next pathway (next_untraveled indirectly marks the crossroads as
                #   fully explored if such is the case)
                axis, self.travel_to = self.node_pointer.next_untraveled()
                if self.travel_to == 0 and traveled_from == 0:
                    self.logger.info("Finished exploring.")
                    self.finish()
                last_angle: float = float(get_key(dic=axis_angles, value=axis))
                # Robot was ordered go straight (False if not)
                straight: bool = last_angle == self.target_pose["turn"]
                # Define the orders for the robot
                self.action = sides[int(last_angle - self.target_pose["turn"])]
                self.target_pose["turn"] = last_angle
                if not straight:
                    self.axis = "turn"
                else:
                    self.axis = "y" if last_angle in (90, -90) else "x"

                self.logger.debug(f"Next way: {self.target_pose['turn']}")
            else:
                # Case where the robot has already rotated
                # Must go front; Axis (x or y) is the one it's faced to
                self.action = "front"
                self.axis = "x" if self.target_pose["turn"] in [0, -180] else "y"

        if self.action == "front":
            self.target_pose[self.axis] += 2

        self.logger.info(f"Next action: {self.action}, axis: {self.axis}")
        self.logger.info(f"Target pose new values: x={self.target_pose['x']}, "
                         f"y={self.target_pose['y']}, "
                         f"angle={self.target_pose['turn']}")

        if self.action not in possible_actions:
            self.logger.critical(f"Action \"{self.action}\" not recognized!")

    def do_micro_action(self):

        assert self.axis != "None" or self.action == "finished", "I don't know what to do!"

        # Debug printing
        self.logger.debug(f"Current axis is {self.axis}")

        lin: float = 0
        rot: float = 0

        # Log the error value
        if not self.action == "finished":
            error: float = self.target_pose[self.axis] - self.feedback[self.axis]
            self.logger.debug(f"Controller error: {round(error, 3)} => "
                              f"target {self.axis} = {self.target_pose[self.axis]}, "
                              f"actual {self.axis} = {round(self.feedback[self.axis], 3)}")

        if self.axis in ["x", "y"]:
            # lin = self.xy_control.c_action(
            #     set_point=self.target_pose[self.axis], feedback=self.feedback[self.axis]
            # )
            lin = 0.09
            rot = self.correct_pose()
            self.logger.debug(f"Linear velocity output: {round(lin, 3)}")
            self.logger.debug(f"Rotational velocity correction: {round(rot, 3)}")
        elif self.axis == "turn":
            rot = self.control_rotation()
            self.logger.debug(f"Rotational velocity output: {round(rot, 3)}")
        elif self.action == "finished":
            pass  # nop (velocity default is already 0!)
        else:
            raise AssertionError("Unexpected behavior!")

        # Drive Motors
        self.driveMotors(
            lPow=lin - rot / 2,
            rPow=lin + rot / 2
        )

    def is_micro_action_complete(self) -> bool:
        """
        Check if the robot pose is fairly close from its goal position
        :return: True if robot is in target pose (tolerance margin defines how close it must be).
            False otherwise.
        """

        # Tolerance margins from the target pose
        margins: Dict[str, float] = dict(x=.2, y=.2, turn=.01)  # margin values can be tuned

        trign = math.sin if self.target_pose["turn"] in [90.0, -90.0] else math.cos

        error_x: float = abs(self.feedback["x"] - self.target_pose["x"])
        error_y: float = abs(self.feedback["y"] - self.target_pose["y"])
        error_angle: float = abs(
            trign(math.radians(self.feedback["turn"]))
            - trign(math.radians(self.target_pose["turn"]))
        )

        # Check closeness from the:
        # * x axis
        if error_x > margins["x"]:
            self.logger.debug(f"Not close enough (e = {error_x}) from X set point")
            return False
        # * y axis
        if error_y > margins["y"]:
            self.logger.debug(f"Not close enough (e = {error_y}) from Y set point")
            return False
        # * turn axis (rotation)
        if self.axis == "turn":
            if error_angle > margins["turn"] * .25:  # 25% less margin when rotating
                # We shrank the tolerance here since it is more important for the robot's angle to
                #   be correct in this case because it is likely starting an action of "going ahead",
                #   propagating any angle error to larger proportions
                self.logger.debug(f"Not close enough (e = {error_angle}) from Angle set point")
                return False
        else:
            if error_angle > margins["turn"]:
                # In this case, the robot is going ahead and the self.correct_pose is already adjusting
                #   the agent's orientation, so a slightly larger slack is acceptable
                self.logger.debug(f"Not close enough (e = {error_angle}) from Angle set point")
                return False

        return True

    def control_rotation(self) -> float:
        """
        This function is to be called when the robot must turn left/right/back (radius=0)
        :return:
        """
        # Calculate difference between intended angle value and actual angle value (orientation)
        error = self.target_pose[self.axis] - self.feedback[self.axis]
        # Coefficients to formula TODO tune
        a, b = 1, 1 / 8.5
        return math.atan(math.radians(error) * a) * b

    def correct_pose(self) -> float:
        """
        Based on the robot distance to the center of the cell,
            calculates the error
        :return: rotational velocity
        """
        # Get target (xt, yt) and current (x, y) coordinates
        x: float = self.feedback["x"]
        y: float = self.feedback["y"]
        xt: float = self.target_pose["x"]
        yt: float = self.target_pose["y"]
        # Coefficient
        k: float = .05  # tune instructions: above 0.1 it becomes unstable
        # Either north, west, east or south (variable used for logging purpose)
        direction: str
        # Deviation from the mid line
        error: float
        if 45 <= self.feedback["turn"] < 135:  # NORTH
            error = - (xt - x)
            direction = "NORTH"
        elif -135 >= self.feedback["turn"] or 135 < self.feedback["turn"]:  # WEST
            error = - (yt - y)
            direction = "WEST"
        elif -135 <= self.feedback["turn"] < -45:  # SOUTH
            error = (xt - x)
            direction = "SOUTH"
        else:  # EAST
            error = (yt - y)
            direction = "EAST"

        # Log
        self.logger.debug(f"Going {direction}. Deviating {round(error, 3)} units from the mid line")

        return k * error

    def create_micro_map(self, root: bool) -> Dict[str, int]:
        """
        This function assumes that the direction the agent faces to is the target pose (since the agent at this
            phase has just arrived to the goal position, therefore currentPose=targetPose, hence the assumption)
        Creates a map of the surrounding elements
        :return: A dictionary in the format: {west: ORIGIN, south, WALL, ...}
        """
        # Another name for map...
        topography: Dict[str, int] = dict()

        # Helper variables
        side_ids: Dict[str, int] = dict(front=0, left=1, right=2, back=3)
        _sides: Dict[int, str] = sides.copy()
        _axis_angles: Dict[int, str] = axis_angles.copy()
        # Map each relative axis of the robot (left, front, ...) to the actual axis (north, east, ...)
        sides_to_axis: Dict[str, str] = {}
        for key, value in _axis_angles.items():
            sides_to_axis[_sides[key]] = _axis_angles[int(self.target_pose["turn"]) + key]

        # Assign the way the robot came from as ORIGIN (see ternary_tree enums)
        # Unless the robot has just spawned
        if not root:
            back: int = int(utils.opposite_angle(angle=self.target_pose["turn"]))
            came_from: str = _axis_angles.get(back)
            topography[came_from] = ternary_tree.ORIGIN
            # Log
            self.logger.debug(f"The robot came from {came_from} ({back}).")
            # Remove the now unused sides
            _sides.pop(back)
            _axis_angles.pop(back)

        # Assign the others as walls (WALL) or clear pathway (CLEAR)
        for key, value in sides.items():
            # E.g.: topography[left -> north] = WALL or CLEAR from the evaluation of sensor[id_of[left]]
            dist: float = self.measures.irSensor[side_ids[value]]
            topography[sides_to_axis[value]] = utils.eval_distance(obstacle_sensor=dist)

        return topography

    def move_on(self, walls: dict):
        """
        Guides the robot through closed pathways (must not include dead ends)
        :return:
        """

        # Angle associated to the axis (e.g. north -> +90)
        absolute_angle: int = get_key(axis_angles, get_key(walls, ternary_tree.CLEAR))
        relative_angle: int = int(self.target_pose["turn"]) + absolute_angle
        self.action = sides[relative_angle]
        if self.action == "front":
            self.axis = "y" if self.target_pose["turn"] in (90, -90) else "x"
        else:
            self.axis = "turn"
            assert not self.action == "back", "Unexpected, car is in dead end!"

        return


class Map:
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 60.0, -60.0, 180.0], host)
    if mapc is not None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
