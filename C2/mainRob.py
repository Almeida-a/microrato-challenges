import logging
import logging.config
import math
import pdb
import sys
import yaml

from typing import Dict, Tuple, List

from croblink import *
from math import *
import xml.etree.ElementTree as ET
import control_action

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    # Logging configuration load
    with open('./logging.yml', 'r') as stream:
        config = yaml.load(stream, Loader=yaml.FullLoader)
    logging.config.dictConfig(config)

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # Logger(s) instantiation
        self.logger = logging.getLogger("wander_debug")
        # Type of movement that the robot is
        #  doing (around what axis, e.g.: up/down, sideways or rotation)
        self.axis: str = "None"
        # Relative movement action that the robot should do
        #   e.g.: turn left/right/back or go front
        self.action: str = "starting"
        self.init_pose: dict = {}  # Initial gps coordinates
        # Controllers
        self.xy_control = control_action.ControlAction(ti=10.0, td=.0, kp=0.70)  # translation
        self.rot_control = control_action.ControlAction(ti=1.0, td=.2, kp=0.10)  # rotation
        # Holds the goal target pose for the robot for each movement
        # This variable complements "axis" and "action"
        self.target_pose: Dict[str, float] = {"x": .0, "y": .0, "turn": .0}
        # Holds the relative gps coordinates to the spawn coordinates
        self.feedback: Dict[str, float] = {}
        # TODO delete test variable
        self.step: int = 0

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

        # TODO consider commenting the existing info logs (no practical use and make difficult to read log file)
        self.logger.info("Wander Iteration Init")

        # Update feedback value (GPS + compass)
        self.feedback = {
            "x": (self.measures.x - self.init_pose["x"]) / 2,
            "y": (self.measures.y - self.init_pose["y"]) / 2,
            "turn": self.measures.compass % 360
        }
        # Normalize to this program's standard
        self.target_pose["turn"] %= 360

        self.logger.debug(f"Current position: x={round(self.feedback['x'], 3)}, y={round(self.feedback['y'], 3)},"
                          f" angle={round(self.feedback['turn'], 3)}.")
        self.logger.debug(f"Target pose: x={self.target_pose['x']}, y={self.target_pose['y']},"
                          f" angle={self.target_pose['turn']}.")

        # If close enough to the target cell, proceed
        if self.is_micro_action_complete():
            # Decide next actions (move 1 cell front or turn)
            #   based on the walls detected
            self.next_micro_action()
            self.step += 1  # TODO delete (test variable)

        # Move
        self.do_micro_action()

    def next_micro_action(self):
        """
                This function should give orders whether to:
                    * Rotate (how many degrees)
                    * Move (one cell forward)
                Based on:
                    * The obstacle sensors
                    * The moving mode (explore or go to certain position)

                :return: action order
                """

        # self.logger.info("Deciding the next micro action...")

        possible_actions: tuple = ("left", "right", "back", "front", "finished")

        # Placeholders until the above algorithm is implemented
        self.action = "left"
        self.axis = "turn"
        # TODO Test code
        if self.step in [0, 1, 2]:
            self.action = "front"
            self.axis = "x"
            self.target_pose["x"] += 1
        elif self.step == 3:
            self.action = "right"
            self.axis = "turn"
            self.target_pose["turn"] -= 90
        elif self.step in [4, 5]:
            self.action = "front"
            self.axis = "y"
            self.target_pose["y"] -= 1
        elif self.step == 6:
            self.action = "finished"
            self.axis = "None"
        # TODO Decide what action to take based on the obstacle sensors
        #  and the algorithm (and update target)
        # if dead end:
        #   turn or go front (depending on the robot's pose)
        # else:
        #   let the mapping algorithm decide
        ...
        # TODO before implementing the mapping algorithm, script the robot's movements
        #   in the code for testing

        self.logger.debug(f"Next action: {self.action}, axis: {self.axis}, (debug) step {self.step}")

        if self.action not in possible_actions:
            self.logger.critical(f"Action \"{self.action}\" not recognized!")

    def do_micro_action(self):

        # self.logger.info("Preparing to send power values to actuators (wheels)...")

        assert self.axis != "None" or self.action == "finished", "I don't know what to do!"

        # Debug printing
        self.logger.debug(f"Current axis is {self.axis}")

        power: float = 0
        right: float = power
        left: float = power

        if self.axis in ["x", "y"]:
            power = self.xy_control.c_action(
                set_point=self.target_pose[self.axis], feedback=self.feedback[self.axis]
            )
            right = left = abs(power)
        elif self.axis == "turn":
            power = self.rot_control.c_action(
                set_point=self.target_pose[self.axis], feedback=self.feedback[self.axis]
            )
            if self.action == "left" or self.action == "back":
                right = power
                left = -power
            elif self.action == "right":
                left = power
                right = -power
        elif self.action == "finished":
            pass  # nop
        else:
            raise AssertionError("Unexpected behavior!")

        # Drive Motors
        self.driveMotors(lPow=left, rPow=right)

    def is_micro_action_complete(self) -> bool:
        """
        Check if the robot pose is fairly close from its goal position
        :return: True if robot is in target pose (tolerance margin defines how close it must be).
            False otherwise.
        """
        # self.logger.info("Checking if micro action is complete...")

        # Tolerance margins from the target pose
        margins: Dict[str, float] = dict(x=.2, y=.2, turn=.01)  # margin values can be tuned
        self.logger.debug("Margins are selected to be: "
                          f"{int(margins['x']*100)}% from x pos, "
                          f"{int(margins['y']*100)}% from y pos, "
                          f"{int(margins['turn']*100)}% from angle sin/cos.")
        left_margins: Dict[str, float] = {
            axis: self.target_pose[axis] - margins[axis] for axis in ["x", "y"]
        }
        right_margins: Dict[str, float] = {
            axis: self.target_pose[axis] + margins[axis] for axis in ["x", "y"]
        }
        if self.target_pose["turn"] in [180, 270]:
            left_margins["turn"] = -1
            right_margins["turn"] = -(1 - margins["turn"])
        elif self.target_pose["turn"] in [0, 90]:
            left_margins["turn"] = 1 - margins["turn"]
            right_margins["turn"] = 1
        else:
            self.logger.critical("Target pose is of unexpected value!")
        # E.g.: if the robot is faced north, then sin(angle) ~= 1
        trign = math.sin if self.target_pose["turn"] in [90.0, 270.0] else math.cos

        # Check closeness from the:
        # * x axis
        if not (left_margins["x"] <= self.feedback["x"] <= right_margins["x"]):
            return False
        # * y axis
        if not (left_margins["y"] <= self.feedback["y"] <= right_margins["y"]):
            return False
        # * turn axis (rotation)
        if not (left_margins["turn"] <= trign(math.radians(self.feedback["turn"]))
                <= right_margins["turn"]):
            return False

        return True


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
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
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
