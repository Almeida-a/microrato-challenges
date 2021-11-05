import math
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

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
            print(f"Connection refused or error [{self.status}]")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            last_checkpoint: int = -1
            self.readSensors()
            ground: int = self.measures.ground

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

            if ground == 0 and last_checkpoint == 2:
                # TODO Save time and score (like the teacher described)
                score: int = self.measures.score
                time: int = self.measures.time
                print(f"Score: {score}; Time: {time}")

            last_checkpoint = ground if ground > 0 else 2
            assert 3 > ground > -2, "Incorrect ground value!"

    def _deduce_linear_velocity(self):
        """
        Based on the input from the obstacle sensors, drive slower if there are close obstacles near the more important
         ones, and faster the farther it is from said obstacles
        :return: The scalar value of the linear velocity, where the unit is the robot's diameter
        """
        # Radar IDs
        center_id: int = 0
        left_id: int = 1
        right_id: int = 2
        back_id: int = 3
        # Get sensors values
        left = self.measures.irSensor[left_id]
        right = self.measures.irSensor[right_id]
        center = self.measures.irSensor[center_id]
        back = self.measures.irSensor[back_id]

        # TODO These variables are to be adjusted,
        #  based on whatever seems more adequate, theoretically or in a trial and error approach
        # The total coefficient's purpose is used to calculate the absolute coefficient of each sensor in the formula
        total_coefficient: float = 2.0
        # These are the contribution percentages of each sensor, where
        #  the closer a sensor is to the obstacle, the slower the robot should be.
        # Notes on adjusting the parameters:
        #  * I think the side values should be the same, because we ideally want the robot in the middle of the cells
        #  * The back sensor value should be the smallest (least important one to calculate the linear velocity)
        cp: float = 0.7  # center
        lp: float = 0.1  # left
        rp: float = 0.1  # right
        bp: float = 1 - (cp + lp + rp)  # back
        assert 0 <= bp < 1, "Sum of relative obstacle sensor coefficients should equal 1"

        # debug
        print(f"left = {left}, right = {right}, center = {center}, back = {back}", end=", ")
        return 1 / (total_coefficient * (left * lp
                                         + right * rp
                                         + center * cp
                                         + back * bp))

    def _deduce_rotational_velocity(self):
        """

        :return: The rotational velocity (in degrees, radians or other units?)
        """
        # Radar IDs
        left_id: int = 1
        right_id: int = 2
        # Get sensors values
        left = self.measures.irSensor[left_id]
        right = self.measures.irSensor[right_id]

        # TODO adjust this value through trial and error or/and with theoretical reasoning
        coefficient: float = 0.65

        return coefficient * (right - left)

    def wander(self):

        # Alternate movement indicators
        lin: float = self._deduce_linear_velocity()
        rot: float = self._deduce_rotational_velocity()

        # debug
        print(f"lin = {lin}, rot = {rot}")
        in_l: float = lin - rot
        in_r: float = lin + rot

        self.driveMotors(in_l, in_r)


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
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'

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
