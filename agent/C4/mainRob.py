import logging
import logging.config
import sys

import yaml
from typing import Tuple

from croblink import *
import xml.etree.ElementTree as ET
import csv
import numpy as np

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    # Logging configuration load
    with open('logging.yml', 'r') as stream:
        config = yaml.load(stream, Loader=yaml.FullLoader)
    logging.config.dictConfig(config)

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # Logger
        self.logger = logging.getLogger("mainRob_file")
        # Calibrate the real GPS values
        self.spawn_position: Tuple[float, float] = (.0, .0)
        # Keep the wheels' power values from every previous iteration
        self.last_estimated_rot: float = .0
        self.last_estimated_lin: float = .0
        self.last_eff_pow: Tuple[float, float] = (.0, .0)  # outL, outR
        # Keep the last (estimated) coordinate values
        #   TODO Note: this variable might be corrected at the code in the future: for instance,
        #       based on the obstacle detectors
        self.last_estimated_gps: Tuple[float, float, float] = (.0, .0, .0)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2]. to know if there
    # is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
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

        # Get first position
        self.readSensors()
        self.spawn_position = (self.measures.x, self.measures.y)

        with open('pos.csv', mode='w') as f:
            pos_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            pos_writer.writerow(["real_x", "real_y", "real_ang", "est_x", "est_y", "est_ang"])

            while True:
                self.readSensors()

                # GPS coordinates
                x: float = self.measures.x - self.spawn_position[0]
                y: float = self.measures.y - self.spawn_position[1]
                angle: float = self.measures.compass

                # Estimated coordinates
                self._calculate_estimated_pose()

                # Write in csv
                pos_writer.writerow([x, y, angle, *self.last_estimated_gps])

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

                    self.logger.info("Run state")

                    self.wander()
                elif state == 'wait':
                    self.setReturningLed(True)
                    if self.measures.visitingLed == True:
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
        assert 0 <= bp < 1, f"Sum of relative obstacle sensor coefficients should equal 1 ({bp})"

        return 1 / (total_coefficient * (left * lp
                                         + right * rp
                                         + center * cp
                                         + back * bp))

    def _deduce_rotational_velocity(self):
        """

        :return: The rotational velocity
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

        # TODO tune
        coefficient: float = 0.65

        return coefficient * (right - left)

    def wander(self):
        # Alternate movement indicators
        lin: float = self._deduce_linear_velocity()
        rot: float = self._deduce_rotational_velocity()

        in_l: float = lin - rot / 2
        in_r: float = lin + rot / 2

        # Save the values from this iteration for the next one
        self.last_estimated_lin = lin
        self.last_estimated_rot = rot

        # Calculate effective power estimation
        self._calculate_effective_powers(in_r=in_r, in_l=in_l)

        self.driveMotors(in_l, in_r)

    def _calculate_estimated_pose(self):
        """

        :return: Based on the:
            * estimated effective wheel power values,
            * last estimated pose,
             Estimate new pose
        """
        # Unpack variables
        out_l, out_r = self.last_eff_pow
        x_m1, y_m1, angle_m1 = self.last_estimated_gps
        lin, rot = (out_r + out_l) / 2, out_r - out_l

        # Calculate new x coordinates   
        x: float = x_m1 + lin * np.cos(angle_m1)
        # Calculate new y coordinates
        y: float = y_m1 + lin * np.sin(angle_m1)
        # Calculate new angle coordinates
        angle: float = angle_m1 + rot

        # Output
        self.last_estimated_gps = (x, y, angle)

    def _calculate_effective_powers(self, in_r: float, in_l: float, std_dev: float = 0):
        """

        :return: Estimates each wheel's effective power based on the commands given to them.

        """
        # add gaussian filter (default std_dev=0)
        g_noise: float = np.random.normal(1, std_dev)

        # Output
        self.last_eff_pow = (
            g_noise * (in_r + self.last_eff_pow[0]) / 2,
            g_noise * (in_l + self.last_eff_pow[1]) / 2
        )


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
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
