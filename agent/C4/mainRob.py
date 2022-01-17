import logging
import logging.config
import sys

import yaml
from typing import Tuple, List, Dict

from croblink import *
import xml.etree.ElementTree as ET
import csv
import numpy as np
from numpy import cos, pi

CELLROWS = 7
CELLCOLS = 14

# --Enums--
# Radar IDs
CENTER_ID: int = 0
LEFT_ID: int = 1
RIGHT_ID: int = 2
BACK_ID: int = 3
# Movement mode
X: int = 0
Y: int = 1
ROT: int = 2  # Deprecate?
# Robot movement states
START: int = 10
BACK: int = 20
FRONT: int = 30
SIDE: int = 40
BLIND: int = 50
ROTATE: int = 60
# ---------

# Max distance the robot can run in one cycle
MAX_POW: float = .15
# Detection threshold (for front and back sensors)
ANGLE: float = np.deg2rad(60)
# return the max distance such that the returning measure is referring to the front wall (not a side wall)
THRESHOLD_DIST: float = 1.0  # 1 / cos((pi - ANGLE) / 2.)  # equals 2.0


class MyRob(CRobLinkAngs):
    # Logging configuration load
    with open('logging.yml', 'r') as stream:
        config = yaml.load(stream, Loader=yaml.FullLoader)
    logging.config.dictConfig(config)

    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # Logger
        self.logger = logging.getLogger("mainRob_file")
        self.angle_m1: float = .0
        # Keep the wheels' power values from every previous iteration
        self.rot_m1: float = .0
        self.lin_m1: float = .0
        self.last_eff_pow: Tuple[float, float] = (.0, .0)  # outL, outR

        # Correction model
        # Hold position estimation with M.M. + with
        self.last_corr_pos: List[float, float] = [.0, .0]
        # Axis - can be X, Y or ROT
        self.axis: int = -1
        # Straight run's state holder variables
        self.mov_state: int = START
        # Last sensor value (either back, center or side*)
        self.sensors_m1: List[float] = []
        # Target orientation value (in degrees)
        self.target_orientation: int = 0
        # Keep the last not-corrected pose estimation (uncertain/shaky certainty)
        self.last_mm_pos: Tuple[float, float] = .0, .0

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
        spawn_position: tuple = (self.measures.x, self.measures.y)

        with open('pos.csv', mode='w') as f:
            pos_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            pos_writer.writerow(["real_x", "real_y", "est_x", "est_y"])

            while True:
                self.logger.info(f"State: {state}")

                self.readSensors()

                if state != "stop":
                    # GPS coordinates
                    x: float = self.measures.x - spawn_position[0]
                    y: float = self.measures.y - spawn_position[1]

                    self._update_axis()  # Set axis according to the robot's orientation

                    # State transition evaluation
                    shifted: bool = self._state_transition()
                    # Log state and real coordinates
                    self.logger.debug(f"Current state is {self.mov_state}.")
                    self.logger.debug(f"Current real coordinates are: (x,y) = {(round(x, 3), round(y, 3))}")
                    # Do stateful correction
                    self._stateful_correction(state_change=shifted)

                    # Write in csv
                    pos_writer.writerow([x, y, *self.last_corr_pos])

                # Get value of angle for

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
                    if self.measures.visitingLed is True:
                        self.setVisitingLed(False)
                    if self.measures.returningLed is True:
                        state = 'return'
                    self.driveMotors(0.0, 0.0)
                elif state == 'return':
                    if self.measures.visitingLed is True:
                        self.setVisitingLed(False)
                    if self.measures.returningLed is True:
                        self.setReturningLed(False)
                    self.wander()

                # Keep values of sensor as memory for the next loop iteration
                self.sensors_m1 = self.measures.irSensor
                # Keep last cycle's angle value
                self.angle_m1 = self.measures.compass
                # # TODO deprecate, as this seems redundant
                # self.last_mm_pos = self.last_corr_pos
                # ...

    # TODO Deprecate
    def _deduce_linear_velocity(self):
        """
        Based on the input from the obstacle sensors, drive slower if there are close obstacles near the more important
         ones, and faster the farther it is from said obstacles
        :return: The scalar value of the linear velocity, where the unit is the robot's diameter
        """
        # Get sensors values
        left = self.measures.irSensor[LEFT_ID]
        right = self.measures.irSensor[RIGHT_ID]
        center = self.measures.irSensor[CENTER_ID]
        back = self.measures.irSensor[BACK_ID]

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

    # TODO deprecate
    def _deduce_rotational_velocity(self):
        """

        :return: The rotational velocity
        """
        # Get sensors values
        left = self.measures.irSensor[LEFT_ID]
        right = self.measures.irSensor[RIGHT_ID]
        center = self.measures.irSensor[CENTER_ID]
        back = self.measures.irSensor[BACK_ID]

        # tune
        coefficient: float = 0.65

        return coefficient * (right - left)

    def wander(self):
        # Alternate movement indicators
        lin: float = self._deduce_linear_velocity()
        rot: float = self._deduce_rotational_velocity()

        in_l: float = lin - rot / 2
        in_r: float = lin + rot / 2

        # Save the values from this iteration for the next one
        self.lin_m1 = lin
        self.rot_m1 = rot

        # Calculate effective power estimation
        self._calculate_effective_powers(in_r=in_r, in_l=in_l)

        self.driveMotors(in_l, in_r)

    def _movement_model(self):
        """
        :return: Based on the:
            * estimated effective wheel power values,
            * last estimated pose,
             Estimate new pose
        """
        # Unpack variables
        out_l, out_r = self.last_eff_pow
        x_m1, y_m1 = self.last_corr_pos
        lin = (out_r + out_l) / 2
        self.logger.debug(f"Linear velocity: {round(lin, 3)}")

        # Calculate new x coordinates
        x: float = x_m1 + lin * np.cos(np.radians(self.angle_m1))
        # Calculate new y coordinates
        y: float = y_m1 + lin * np.sin(np.radians(self.angle_m1))

        self.logger.debug(f"New m.m. coordinates: x = {round(x, 3)}, y = {round(y, 3)},"
                          f" with angle = {round(self.measures.compass, 3)}.")
        # Output
        self.last_mm_pos = (x, y)

    def _calculate_effective_powers(self, in_r: float, in_l: float, std_dev: float = 0):
        """

        :return: Estimates each wheel's effective power based on the commands given to them.

        """
        # add gaussian filter (default std_dev=0)
        g_noise: float = np.random.normal(1, std_dev)

        # Normalize in_{l,r} to their max power if applicable
        in_l = in_l if in_l <= MAX_POW else MAX_POW
        in_r = in_r if in_r <= MAX_POW else MAX_POW

        # Output
        self.last_eff_pow = (
            g_noise * (in_r + self.last_eff_pow[0]) / 2,
            g_noise * (in_l + self.last_eff_pow[1]) / 2
        )

        self.logger.debug("Directed power: "
                          f"inL = {in_l}, "
                          f"inR = {in_r}.")
        self.logger.debug(f"Effective power: "
                          f"outL = {self.last_eff_pow[0]}, "
                          f"outR = {self.last_eff_pow[1]}.")

    def _front_info(self) -> bool:
        return self._vertical_perception(CENTER_ID)

    def _back_info(self) -> bool:
        return self._vertical_perception(BACK_ID)

    def _vertical_perception(self, obs_id: int) -> bool:
        """
        Term "True detection" means that the front/back wall is reachable to the respective sensor
        :return: Information about the robot's front/back obstacles: if it is a true detection,
            the return value is (true, measure) where measure is the distance from the robot
            to the front/back wall; otherwise the value (false, measure) is returned
            and measure is most likely the distance from the front sensor to a side wall

        """
        assert obs_id == BACK_ID or obs_id == CENTER_ID, \
            f"This function only handles front/back sensors, not {obs_id}"
        dist: float = 1 / self.measures.irSensor[obs_id]
        measure_validity: bool = dist <= THRESHOLD_DIST
        return measure_validity

    def _update_axis(self):
        # Evaluate if N/S/E/W, then update axis and other variables as needed
        orientation: str = self.get_orientation_axis()

        self.logger.debug(f"Current axis: {orientation}")

        if orientation in ("NORTH", "SOUTH"):
            self.axis = Y
        else:
            self.axis = X

    def get_orientation_axis(self) -> str:

        orientation: str

        if -45 < self.measures.compass <= 45:
            orientation = "EAST"
        elif -135 < self.measures.compass <= -45:
            orientation = "SOUTH"
        elif 45 < self.measures.compass <= 135:
            orientation = "NORTH"
        else:  # Below -135 or above 135
            orientation = "WEST"

        return orientation

    def _state_transition(self) -> bool:
        """

        :return: When next state is determined, return True if it is different from current
        """
        # Holds information about which walls of which sides are in range (True)
        #   or out of range (False)
        dists = dict(left=1 / self.measures.irSensor[LEFT_ID],
                     right=1 / self.measures.irSensor[RIGHT_ID],
                     front=1 / self.measures.irSensor[CENTER_ID],
                     back=1 / self.measures.irSensor[BACK_ID])

        self.logger.debug("Obstacle sensors distances: "
                          f"C={round(dists['front'], 3)}, "
                          f"B={round(dists['back'], 3)}, "
                          f"L={round(dists['left'], 3)}, "
                          f"R={round(dists['right'], 3)}")

        walls: Dict[str, bool] = dict(
            front=self._front_info(), back=self._back_info(),
            side=dists['right'] < .5 or dists['left'] < .5
        )  # Checks if the walls are 'within reach'
        self.logger.debug(f"Walls reachability info: {walls}")

        # Transition based on: current state and on sensors (Mealy Machine)
        # TODO If/When possible, optimize the following state transition process
        if self.mov_state == START:
            # Check eligible states: BACK -> FRONT -> SIDE -> BLIND
            if walls["back"]:
                self.mov_state = BACK
            elif walls["front"]:
                self.mov_state = FRONT
            elif walls["side"]:
                self.mov_state = SIDE
            else:
                self.mov_state = BLIND
            return True
        elif self.mov_state == BACK:
            # back wall in range
            if not walls["back"]:
                # - Going above the range of "sight" of the back wall to the back sensor
                if walls["front"]:
                    # - Detecting front walls -> FRONT
                    self.mov_state = FRONT
                elif walls["side"]:
                    # - Detecting side walls (>0.4 cells of distance) -> SIDE
                    self.mov_state = SIDE
                else:
                    # - Else -> BLIND
                    self.mov_state = BLIND
                return True
            # If back wall is still in range, no state change is needed
            return False
        elif self.mov_state == FRONT:
            # front wall in range
            if self.measures.irSensor[CENTER_ID] > 2.5:
                # - Getting too close to the front wall
                #   -> ROTATE (and complementary decide where to)
                self.mov_state = ROTATE
                return True
            #   - Else -> FRONT
            return False
        elif self.mov_state == SIDE:
            # side state: (-> ROTATE/FRONT/BLIND)
            if walls["front"]:
                # - Start detecting a front wall -> FRONT
                self.mov_state = FRONT
                return True
            elif abs(self.target_orientation - self.measures.compass) > 45:
                # Difference being more than 45 means that the
                #   algorithm has changed the orientation, and thus,
                #   decided that a rotation must be carried out
                self.mov_state = ROTATE
                return True
            elif self.measures.irSensor[LEFT_ID] > 2.3 and self.measures.irSensor[RIGHT_ID] > 2.3:
                # When side detection stagnates (to ~0.4 diameters) -> BLIND
                self.mov_state = BLIND
                return True
            # Else -> maintain SIDE state
            return False
        elif self.mov_state == BLIND:
            # blind state: (-> BLIND/SIDE/FRONT)
            if walls["side"]:
                # Start detecting any side wall -> SIDE
                self.mov_state = SIDE
                return True
            elif walls["front"]:
                # Start detecting front wall -> FRONT
                self.mov_state = FRONT
                return True
            # Else -> maintain BLIND state
            return False
        elif self.mov_state == ROTATE:
            # Rotate until angle is close enough to its target value
            # Go back to START when that condition is passed
            margin: float = 20.0  # degrees of margin
            if abs(self.measures.compass - self.target_orientation) < margin or \
                    self.lin_m1 > .25:
                self.mov_state = START
                return self._state_transition()
            return False
        else:
            self.logger.critical(f"Unexpected state: {self.mov_state}.")

    def _stateful_correction(self, state_change: bool):
        """
        Adjusts the position estimation based on the correction, itself based on
         the state of the robot in its straight run
        :return:
        """

        # TUNABLE parameters
        # Relative Contribution weight of, respectively, M.M. and sensors
        p1, p2 = .75, .25
        # Absolute contribution weight of sensors based estimation
        p2_a: float = .7

        # Run movement model
        self._movement_model()

        # Case:
        # Complementary axis oscillations are not being taken into account TODO bear this in mind
        if self.mov_state in (BACK, FRONT):
            sensor_id = BACK_ID if self.mov_state == BACK else CENTER_ID
            if state_change:
                # On first instance,
                #   only estimate from M.M. (since there is no valid "memory" sensor values)
                self.last_corr_pos = list(self.last_mm_pos)
                return
            # Get movement model estimated position change
            mm_diff = tuple(
                [self.last_mm_pos[i] - self.last_corr_pos[i] for i in (0, 1)]
            )
            self.logger.debug(f"Movement model update: (x, y) = {mm_diff}")

            # check differences from current and last back sensor value
            #   and mend
            # But first, assert that axis is either X or Y (0 or 1)
            assert self.axis in (X, Y), "self.axis value is invalid!"

            # Sensors update estimation
            mult1: int = -1 if self.get_orientation_axis() in ("WEST", "SOUTH") else 1
            mult2: int = -1 if self.mov_state == FRONT else 1

            sensor_diff: float = mult1 * mult2 * (
                    1 / self.measures.irSensor[sensor_id] - 1 / self.sensors_m1[sensor_id])

            # # Makes no sense to say that the agent ran more than .15 units in one cycle
            # if abs(sensor_diff) > MAX_POW:
            #     sensor_diff = MAX_POW if sensor_diff > 0 else -MAX_POW

            self.logger.debug(f"Sensor diff update: ({'y' if self.axis == Y else 'x'}) = ({sensor_diff})")

            self.last_corr_pos[self.axis] += mm_diff[self.axis] * p1 + sensor_diff * p2 * p2_a
            # # Complementary angle is obtained uniquely from the M.M.
            # self.last_corr_pos[1 - self.axis] += mm_diff[1 - self.axis]
        elif self.mov_state == SIDE:
            # Since this is a more complex and error-prone position estimation,
            # Reduce its contribution weight
            p1, p2 = 0.9, 0.1
            # check differences
            # and mend
            # How far away from the wall(s) the robot is
            offset: float = .4  # or calculate based on the robot's position

            # Formula for estimating a side sensor based position estimation
            # This assumes the obstacle sensors are at the exact side wing of the robot, which is wrong,
            #   it is 60 degrees deviated from the front sensor, not 90
            diff_sides = tuple(
                np.sqrt((1 / self.measures.irSensor[side]) ** 2 - offset ** 2) for side in (LEFT_ID, RIGHT_ID)
            )

            self.logger.debug("Side correction estimation: ")

            # TODO continue here the SIDE state corrections...
            # 100% M.M. for now...
            self.last_corr_pos = list(self.last_mm_pos)
        elif self.mov_state == BLIND:
            # Movement model 100% weighted estimation
            self.last_corr_pos = list(self.last_mm_pos)
            return
        elif self.mov_state == ROTATE:
            # No-op
            return
        else:
            # Error: state not covered
            self.logger.critical(f"Unexpected state: {self.mov_state}")
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
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
