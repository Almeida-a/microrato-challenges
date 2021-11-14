import math
import sys

from typing import Dict, Tuple, List

from croblink import *
from math import *
import xml.etree.ElementTree as ET
import control_action

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # Type of movement that the robot is
        #  doing (around what axis, e.g.: up/down, sideways or rotation)
        self.axis: str = "None"
        # Relative movement action that the robot should do
        #   e.g.: turn left/right/back or go front
        self.action: str = "starting"
        self.init_pose: dict = {}  # Initial gps coordinates
        # Controllers
        self.xy_control = control_action.ControlAction(ti=10.0, td=.0, kp=0.70)  # translation
        self.rot_control = control_action.ControlAction(ti=10.0, td=.0, kp=0.70)  # rotation
        # Holds the goal target pose for the robot for each movement
        # This variable complements "axis" and "action"
        self.target_pose: Dict[str, float] = {"x": 1.0, "y": 0.0, "orientation": 0.0}
        # Holds the relative gps coordinates to the spawn coordinates
        self.feedback: Dict[str, float] = {}

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

        # Update feedback value (GPS + compass)
        self.feedback = {
            "x": (self.measures.x - self.init_pose["x"]) / 2,
            "y": (self.measures.y - self.init_pose["y"]) / 2,
            # "orientation": self.measures.compass
        }

        # If close enough to the target cell, proceed
        if self.is_micro_action_completed():
            # Decide next actions (move 1 cell front or turn)
            #   based on the walls detected
            self.next_micro_action()

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

        possible_actions: tuple = ("left", "right", "back", "front")

        # TODO Decide what action to take based on the obstacle sensors
        #  and the algorithm (and update target)
        # if deadend:
        #   turn or go front (depending on the robot's pose)
        # else:
        #   let the mapping algorithm decide
        ...
        # Placeholders until the above algorithm is implemented
        self.action = "left"
        self.axis = "turn"
        # TODO before implementing the mapping algorithm, script the robot's movements
        #   in the code for testing

        assert self.action in possible_actions, f"Action \"{self.action}\" not recognized!"

    def do_micro_action(self):

        # Do micro-action based on order
        # if self.action == "left":
        #     self.axis = "turn"
        #     self.target_pose["orientation"] += 90
        # elif self.action == "right":
        #     self.axis = "turn"
        #     self.target_pose["orientation"] -= 90
        # elif self.action == "back":
        #     self.axis = "turn"
        #     self.target_pose["orientation"] += 180
        # elif self.action == "front":
        #     # Where is the vehicle faced towards:
        #     if -0.5 <= self.measures.compass <= 0.5:  # Face right
        #         self.axis = "x"
        #         # Front increment
        #         self.target_pose[self.axis] += 1
        #     elif 178 <= self.measures.compass <= 182:  # Face left
        #         self.axis = "x"
        #         # Front increment
        #         self.target_pose[self.axis] -= 1
        #     elif 88 <= self.measures.compass <= 92:  # Face up
        #         self.axis = "y"
        #         # Front increment
        #         self.target_pose[self.axis] += 1
        #     elif -92 <= self.measures.compass <= -88:  # Face down
        #         self.axis = "y"
        #         # Front increment
        #         self.target_pose[self.axis] -= 1

        assert self.axis != "None", "I don't know what to do!"

        # Debug printing
        print(f"Feedback: {round(self.feedback[self.axis], 3)};"
              f"\t\t Next move: {self.target_pose[self.axis]}.")

        power: float = 0
        right: float = power
        left: float = power

        if self.axis == "x" or self.axis == "y":
            power = self.xy_control.c_action(
                set_point=self.target_pose[self.axis], feedback=self.feedback[self.axis]
            )
            right = left = power
        elif self.axis == "turn":
            power = self.rot_control.c_action(
                set_point=self.target_pose[self.axis], feedback=self.feedback[self.axis]
            )
            if self.action == "left" or self.action == "back":
                right = power
                left = -right
            elif self.action == "right":
                left = power
                right = -left

        # Drive Motors
        self.driveMotors(lPow=left, rPow=right)

    def is_micro_action_completed(self) -> bool:
        """
        Check if the robot pose is fairly close from its goal position
        :return: True if robot is in target pose (tolerance margin defines how close it must be).
            False otherwise.
        """
        # Tolerance margin from the target pose
        margins: Dict[str, float] = dict(x=.1, y=.1, turn=.1)  # margin values can be tuned
        left_margins: Dict[str, float] = {
            axis: self.target_pose[axis]*(1-margins[axis]) for axis in margins.keys()
        }
        right_margins: Dict[str, float] = {
            axis: self.target_pose[axis]*(1+margins[axis]) for axis in margins.keys()
        }

        # Check closeness from the:
        # * x axis
        if not (left_margins["x"] < self.feedback["x"] < right_margins["x"]):
            return False
        # * y axis
        if not (left_margins["y"] < self.feedback["y"] < right_margins["y"]):
            return False
        # * turn axis (rotation)
        if not (left_margins["turn"] < self.measures.compass < right_margins["turn"]):
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
