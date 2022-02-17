import logging
import logging.config
import sys

import yaml
from typing import Tuple, List, Dict, Union

import utils
from croblink import *
import xml.etree.ElementTree as ET
import csv
import numpy as np
from numpy import cos, sin, pi

CELLROWS = 7
CELLCOLS = 14

# -------Enums-------
# Radar IDs
CENTER_ID: int = 0
LEFT_ID: int = 1
RIGHT_ID: int = 2
BACK_ID: int = 3
# Movement axis
X: int = 10
Y: int = 20
ROT: int = 30
# Crossroads passages
EXPLORED: int = 100
UNEXPLORED: int = 200
ORIGIN: int = 300
WALL: int = 400
# Explore modes
EXPLORE: int = 110
RETURN: int = 220
# -------------------

# POSSIBLE_MOVEMENT_TYPES: tuple = ("explore", "return")
POSSIBLE_ACTIONS: tuple = ("front", "right", "back", "left", "finished")  # Discard start state
AXIS_PRECEDENCE: tuple = ("WEST", "SOUTH", "EAST", "NORTH")

# Detection threshold (for front and back sensors)
THRESH_ANGLE: float = np.deg2rad(60)
# return the max distance such that the returning measure is referring to the front wall (not a side wall)
THRESHOLD_DIST: float = 1 / cos((pi - THRESH_ANGLE) / 2.)  # equals 2.0
THRESH_DIST_INHIBITOR: float = .5
THRESHOLD_DIST *= THRESH_DIST_INHIBITOR
# Max angle error measure value (assuming always > 0)
MAX_ANGLE_ERROR: float = 180.0

# --------------------------------- TUNABLE constants --------------------------
MAX_POW: float = .1  # Max. distance the robot can run in one cycle
ROTATION_DECELERATION_THRESH: float = 30.0  # When angle error falls below this value, velocity begins to decelerate
TRANSLATION_DECELERATION_THRESH: float = .3  # When translation error falls below, velocity begins to decelerate
SENSOR_DIFF_MIN: float = 0.7  # E.g. 0.3 -> If sensor increment is less than 30% of Mov. Model, discard it
SENSOR_DIFF_MAX: float = 1.5  # E.g. 1.5 -> If sensor increment is more than 150% of Mov. Model, discard it
# ------------------------------------------------------------------------------


def axis_precedence(axis_list: list) -> str:
    """

    :param axis_list: list of axis
    :return: First axis counting from west counter-clockwise
    """
    # Remove invalid items
    axis_list = list(filter(lambda elem: elem in AXIS_PRECEDENCE, axis_list))
    if not axis_list:  # List empty
        raise AssertionError("No valis axis in axis list!")
    return axis_list[min([AXIS_PRECEDENCE.index(axis) for axis in axis_list])]


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

        # Movement
        self.target_pose: Dict[int, int] = {X: 0, Y: 0, ROT: 0}

        # Exploration
        # Micro-action command
        self.action: str = "start"
        self.next_action: str = ""
        # Initialize explore mode
        self.explore_mode: int = EXPLORE
        # Associates a crossroad's center coordinates to the crossroads where they lead to
        # e.g.: (x, y) -> {north=(x1, y1), east=ORIGIN, west=(x2, y2), south=EXPLORED}
        self.crossroads: Dict[Tuple[int, int], Dict[str, Union[Tuple[int, int], int]]] = {}

        # Correction model
        # Hold position estimation with M.M. + with
        self.last_corr_pos: List[float, float] = [.0, .0]
        # Axis - can be X, Y or ROT
        self.axis: int = -1
        # Last sensor value (either back, center or side*)
        self.sensors_m1: List[float] = [-1., -1., -1., -1.]
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
                self.readSensors()

                if self.measures.endLed is True:
                    print(self.rob_name + " exiting")
                    quit()

                if state == 'stop' and self.measures.start:
                    state = stopped_state

                if state != 'stop' and self.measures.stop:
                    stopped_state = state
                    state = 'stop'

                self.logger.info(f"State: {state}")

                # Makes no sense to evaluate coordinates change when the robot is rotating
                if state != "stop" and self.axis in (X, Y):
                    # GPS coordinates
                    x: float = self.measures.x - spawn_position[0]
                    y: float = self.measures.y - spawn_position[1]

                    self._update_axis()  # Set axis according to the robot's orientation

                    # Log state and real coordinates
                    self.logger.debug(f"Current real coordinates are: (x,y) = {(round(x, 3), round(y, 3))}")
                    # Do stateful correction
                    self._stateless_correction()

                    # Write in csv
                    pos_writer.writerow([x, y, *self.last_corr_pos])

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
                for j, elem in enumerate(self.measures.irSensor):
                    if j in (BACK_ID, CENTER_ID) and not self._vertical_wall(j):
                        self.sensors_m1[j] = -1.
                    else:
                        self.sensors_m1[j] = elem
                # Keep last cycle's angle value
                self.angle_m1 = self.measures.compass

    def wander(self):

        # If close enough to the target cell, proceed
        if self._is_micro_action_complete():
            # Save info to the internal map
            self.write_cell_to_map()
            # Decide next actions (move 1 cell front or turn)
            #   based on the walls detected
            self._next_micro_action()
            self._translate_micro_command()
            # A second kind of position correction
            self._halted_robot_correction()

            assert self.action in POSSIBLE_ACTIONS, f"Action: {self.action} not an action!"

        # Logs
        self._log_cycle_class_vars()

        # Move
        self._do_micro_action()

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

        self.logger.debug(f"New m.m. coordinates: x = ({round(x, 3)}), y = ({round(y, 3)})")
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
        return self._vertical_wall(CENTER_ID)

    def _back_info(self) -> bool:
        return self._vertical_wall(BACK_ID)

    def _vertical_wall(self, obs_id: int) -> bool:
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

    def _stateless_correction(self):

        # TUNABLE parameters
        # Relative Contribution weight of, respectively, M.M. and sensors
        p1, p2 = .2, .8

        # Run movement model
        self._movement_model()

        # Get walls info into variable
        walls = dict(front=self._front_info(), back=self._back_info())

        # Get movement model estimated position change
        mm_diff = tuple(
            [self.last_mm_pos[i] - self.last_corr_pos[i] for i in (0, 1)]
        )
        self.logger.debug(f"Movement model update: (x, y) = ({round(mm_diff[0], 4)}, {round(mm_diff[1], 4)})")

        # check differences from current and last back sensor value
        #   and mend
        # But first, assert that axis is either X or Y (0 or 1)
        assert self.axis in (X, Y), "self.axis value shouldn't be ROT in this function!"

        # Sensors update estimation
        mult1: int = -1 if self.get_orientation_axis() in ("WEST", "SOUTH") else 1

        # Evaluate front wall
        if walls["front"] and self.sensors_m1[CENTER_ID] != -1:
            sensor_diff_front: float = mult1 * - (
                    1 / self.measures.irSensor[CENTER_ID] - 1 / self.sensors_m1[CENTER_ID])
        else:
            sensor_diff_front = 0.

        # Evaluate back wall
        if walls["back"] and self.sensors_m1[BACK_ID] != -1:
            sensor_diff_back: float = mult1 * (
                    1 / self.measures.irSensor[BACK_ID] - 1 / self.sensors_m1[BACK_ID])
        else:
            sensor_diff_back = 0.

        if 0. in (sensor_diff_front, sensor_diff_back):
            # sensor_diff = front/back/0.0
            sensor_diff = sensor_diff_front + sensor_diff_back
        else:
            # Average the result
            sensor_diff: float = (sensor_diff_front + sensor_diff_back) / 2

        self.logger.debug(f"Sensor diff update: ({'y' if self.axis == Y else 'x'}) = ({round(sensor_diff, 4)})")

        axis_index = int(self.axis / 10 - 1)

        # No back or front walls detected
        if sensor_diff == 0 or \
                sensor_diff < SENSOR_DIFF_MIN * mm_diff[axis_index] or \
                sensor_diff > SENSOR_DIFF_MAX * mm_diff[axis_index]:
            # 100% Movement model
            p1 = 1.  # Jesus, take the wheel
            p2 = 0.
        self.last_corr_pos[axis_index] += mm_diff[axis_index] * p1 + sensor_diff * p2

    def _is_micro_action_complete(self):

        if self.action == "start":
            return True

        # Tolerance margins from the target pose
        margins: Dict[str, float] = dict(x=.2, y=.2, turn=20.0)  # margin values can be tuned

        error_x: float = abs(self.last_corr_pos[0] - self.target_pose[X])
        error_y: float = abs(self.last_corr_pos[1] - self.target_pose[Y])
        error_angle: float = abs(self._get_angle_error())

        # If the walls are critically close, force stop and signal completion
        d_front = 1 / self.measures.irSensor[CENTER_ID]
        if self.action == "front" and d_front < .5:
            self.logger.debug(f"Critically close (d = {d_front}) to front wall!")
            self.logger.debug("Micro-action COMPLETE")
            return True

        # Check closeness from the:
        # * x-axis
        if error_x > margins["x"]:
            self.logger.debug(f"Not close enough (e = {error_x}) from X set point")
            return False
        # * y axis
        if error_y > margins["y"]:
            self.logger.debug(f"Not close enough (e = {error_y}) from Y set point")
            return False
        # * turn axis (rotation)
        if self.axis == ROT:
            if error_angle > margins["turn"] * .25:  # 75% less margin when rotating
                # We shrank the tolerance here since it is more important for the robot's angle to
                #   be correct in this case because it is likely starting an action of "going ahead",
                #   propagating any angle error to larger proportions
                self.logger.debug(f"Not close enough (e = {error_angle}) from Angle set point")
                return False
        else:
            self._halted_robot_correction()

            if error_angle > margins["turn"]:
                # In this case, the robot is going ahead and the self.correct_pose is already adjusting
                #   the C4's orientation, so a slightly larger slack is acceptable
                self.logger.debug(f"Not close enough (e = {error_angle}) from Angle set point")
                return False

        # if self.axis in (X, Y):
        #     # TODO check here if the robot is getting too close to the
        #     #  front wall (the accumulated error from the position estimate can cause that)
        #     self._halted_robot_correction()

        self.logger.debug("Micro-action COMPLETE")
        return True

    def _next_micro_action(self):
        """
        This function should give orders whether to:
            * Rotate (left, right or back)
            * Move (one cell forward)
        Based on:
            * The obstacle sensors
            * The moving mode (explore or go to certain position)

        :return: action command
        """

        # Double command
        if self.next_action != "":
            self.action = self.next_action
            self.next_action = ""
            return

        walls_per_axis, walls_per_dir = self._get_cell_walls()
        self.logger.debug(f"Walls per axis: {walls_per_axis}")
        self.logger.debug(f"Walls per sides: {walls_per_dir}")

        walls_count: int = list(walls_per_axis.values()).count("wall")

        # Normalized to the center of the cell the robot is currently in
        normalized_coordinates = self._get_normalized_estimate()
        self.logger.debug(f"Normalized coordinates: {normalized_coordinates}")

        # * Dead-end cell
        if walls_count == 3:
            # assert that the robot encounters the dead-end front face first
            assert utils.get_key(walls_per_dir, "clear") == BACK_ID
            assert self.explore_mode == EXPLORE, f"Explore expected, found {self.explore_mode}"
            # Command -> Go back
            self.explore_mode = RETURN
            self.action = "back"
            self.next_action = "front"
        # * Crossroad -> Go to next unexplored passage--
        elif walls_count in (0, 1) or normalized_coordinates == (0, 0):

            assert len(normalized_coordinates) == 2, f"Unexpected number of elements in normalized coordinates."

            if normalized_coordinates not in self.crossroads.values():
                # New crossroad
                # -> add to c-dict and add info about the walls/passages
                self.crossroads[normalized_coordinates] = dict()
                for axis, passage in walls_per_axis.items():
                    self.crossroads[normalized_coordinates][axis] = WALL if passage == "wall" else UNEXPLORED

                # Go to first unexplored
                unexplored_axis_list: list = utils.get_keys(
                    dic=self.crossroads[normalized_coordinates], value=UNEXPLORED)
                self.action = self._what_side(axis_precedence(unexplored_axis_list))
            else:
                # Known crossroad case
                if self.explore_mode == EXPLORE:
                    # - Exploring -> Turn back, then go front (using next_action)
                    self.explore_mode = RETURN
                    self.action = "back"
                    self.next_action = "front"
                elif self.explore_mode == RETURN:
                    # - Returning -> Go to next unexplored (if all explored, proceed to ORIGIN)
                    self.explore_mode = EXPLORE

                    # Go to first unexplored
                    unexplored_axis_list: list = utils.get_keys(
                        dic=self.crossroads[normalized_coordinates], value=UNEXPLORED)
                    if unexplored_axis_list:  # List not empty
                        self.action = self._what_side(axis_precedence(unexplored_axis_list))
                    else:
                        # Assert all are explored: we assume that the search is always depth first
                        assert all((elem in (EXPLORED, ORIGIN) for elem in self.crossroads[normalized_coordinates]))
                        # Maintain return mode
                        self.explore_mode = RETURN
                        # Go to origin
                        self.action = self._what_side(
                            utils.get_key(dic=self.crossroads[normalized_coordinates], value=ORIGIN))

                    if self.action != "front":
                        self.next_action = "front"
        # * Normal road -> Go front---------------------
        elif walls_count == 2:
            open_passages_sides = utils.get_keys(walls_per_dir, "clear")
            if LEFT_ID in open_passages_sides:
                self.action = "left"
            elif RIGHT_ID in open_passages_sides:
                self.action = "right"
            elif CENTER_ID in open_passages_sides:
                self.action = "front"
            else:
                raise AssertionError("Unexpected state: at least 3 walls found, but 2 expected")

            # If robot turns, give double command
            #   (remember that it has already turned and doesn't need to turn another time
            if self.action in ("left", "right"):
                self.next_action = "front"

        else:
            raise AssertionError(f"Unexpected number of walls: {walls_count}.")

    def _do_micro_action(self):

        lin: float = .0
        rot: float = .0

        angle_error: float = self._get_angle_error()  # between -180 and 180

        if self.axis in (X, Y):
            left_dist: float = 1 / self.measures.irSensor[LEFT_ID]
            right_dist: float = 1 / self.measures.irSensor[RIGHT_ID]

            # Checks if the robot is in a corridor (has walls on both sides in the current cell)
            is_corridor: bool = (left_dist + right_dist) - 0.8 <= .2

            # Measures how far the robot is from the center of the cell (using side sensors)
            deviation_error: float = left_dist - right_dist if is_corridor else 0
            if is_corridor and deviation_error > utils.WALL_MAX_DIST - 0.4:
                self.logger.warning("Robot is straying too much from the center at this corridor!")

            # Scope: 0 <= error <= 1
            error: float = (angle_error / 20.0) * 0.7 + (deviation_error / .8) * 0.3  # Tunable percentages | TODO tune
            rot = MAX_POW * error

            # Linear velocity value should not make the global velocity surpasses MAX_POW
            max_lin = MAX_POW - rot
            lin = max_lin

        elif self.axis == ROT:
            k: float = MAX_POW / ROTATION_DECELERATION_THRESH

            rot = k * angle_error  # Rotational control

            self.logger.debug(f"Angle error = ({round(angle_error, 2)}), "
                              f"rot = ({round(rot, 2)})")

            # Upper bound rot to MAX_POW value
            if abs(rot) > MAX_POW:
                rot = MAX_POW if rot > MAX_POW else -MAX_POW
        else:
            self.logger.critical(f"Unexpected self.axis value: {self.axis}.")
            exit(1)

        self.logger.debug(f"Lin = ({round(lin, 2)}), Rot = ({round(rot, 2)})")

        # Save the values from this iteration for the next one
        self.lin_m1 = lin
        self.rot_m1 = rot

        in_r: float = lin + rot
        in_l: float = lin - rot

        self._calculate_effective_powers(in_l=in_l, in_r=in_r)
        self.driveMotors(in_l, in_r)

    def _get_cell_walls(self) -> Tuple[dict, dict]:
        """

        :return:
        """
        axis: str = self.get_orientation_axis()
        walls = [utils.eval_distance(wall) for wall in self.measures.irSensor]
        walls_dir: Dict[int, float] = {side: utils.eval_distance(self.measures.irSensor[side]) for side in range(4)}

        axis_walls = dict()

        if axis == "NORTH":
            axis_walls = dict(WEST=walls[LEFT_ID], SOUTH=walls[BACK_ID], EAST=walls[RIGHT_ID], NORTH=walls[CENTER_ID])
        elif axis == "SOUTH":
            axis_walls = dict(WEST=walls[RIGHT_ID], SOUTH=walls[CENTER_ID], EAST=walls[LEFT_ID], NORTH=walls[BACK_ID])
        elif axis == "EAST":
            axis_walls = dict(WEST=walls[BACK_ID], SOUTH=walls[RIGHT_ID], EAST=walls[CENTER_ID], NORTH=walls[LEFT_ID])
        elif axis == "WEST":
            axis_walls = dict(WEST=walls[CENTER_ID], SOUTH=walls[LEFT_ID], EAST=walls[BACK_ID], NORTH=walls[RIGHT_ID])
        else:
            self.logger.critical(f"Unexpected axis: {axis}.")
            exit(1)

        return axis_walls, walls_dir

    def write_cell_to_map(self):
        pass

    def _what_side(self, target_axis: str):
        """

        :param target_axis: Reference axis
        :return: Given the orientation of the robot, what side of it is the referenced axis?
        E.g.: NORTH is to the right side of a WEST faced C4
        """
        front_side_axis: str = self.get_orientation_axis()
        ref: Dict[str, str]

        if target_axis == "WEST":
            ref = dict(NORTH="left", EAST="back", SOUTH="right", WEST="front")
            return ref[front_side_axis]
        elif target_axis == "NORTH":
            ref = dict(NORTH="front", EAST="left", SOUTH="back", WEST="right")
            return ref[front_side_axis]
        elif target_axis == "EAST":
            ref = dict(NORTH="right", EAST="front", SOUTH="left", WEST="back")
            return ref[front_side_axis]
        elif target_axis == "SOUTH":
            ref = dict(NORTH="back", EAST="right", SOUTH="front", WEST="left")
            return ref[front_side_axis]
        else:
            raise AssertionError(f"Reference axis not recognized: {target_axis}.")

    def _log_cycle_class_vars(self):
        self.logger.debug(f"Explore mode: {self.explore_mode}.")
        self.logger.debug(f"Current action: {self.action}.")
        self.logger.debug(f"Next action: {self.next_action}.") if self.next_action != "" else None
        self.logger.debug(f"Obstacle sensors: {self.measures.irSensor}")
        self.logger.debug(f"Crossroads info: {self.crossroads}.")
        self.logger.debug(f"Current axis: {self.axis}")
        self.logger.debug(f"Robot position estimate: x=({round(self.last_corr_pos[0], 2)}), "
                          f"y=({round(self.last_corr_pos[1], 2)}), angle=({round(self.measures.compass, 2)})")
        self.logger.debug(f"Robot target position: x=({round(self.target_pose[X], 2)}), "
                          f"y=({round(self.target_pose[Y], 2)}), angle=({round(self.target_pose[ROT], 2)})")

    def _translate_micro_command(self):

        assert self.action in POSSIBLE_ACTIONS, f"Action not recognized: {self.action}."

        rot_translation = dict(left=90, right=-90, back=-180)
        move_translation = dict(WEST=-1, NORTH=1, SOUTH=-1, EAST=1)

        if self.action in rot_translation.keys():
            self.axis = ROT
            self.target_pose[self.axis] += rot_translation[self.action]
        else:
            self._update_axis()
            self.target_pose[self.axis] += move_translation[self.get_orientation_axis()] * 2

        # Normalize target pose angle attribute
        self.target_pose[ROT] = (self.target_pose[ROT] + 180) % 360 - 180

    def _get_angle_error(self) -> float:
        """

        :return: Difference between target angle and actual robot angle, max 2 or -2, if <0.
        """
        a1: float = self.target_pose[ROT]
        a2: float = self.measures.compass

        if a1 == -180 and a2 > 0:
            # Target orientation is WEST
            a2 -= 360

        elif a1 == -90 and a2 >= 90:
            # Target orientation is SOUTH
            a2 -= 360

        elif a1 == 90 and a2 < -90:
            # Target orientation is NORTH
            a2 += 360

        return a1 - a2

    def _halted_robot_correction(self) -> bool:
        """
        Assuming this function is called after the completion of a (translation?) micro-action,
            this function calculates the robot's position regarding the center of the cell
            based on the: front wall (more walls may be taken into account in the future TODO do it now)
        :return: True if a correction was carried out
        """
        center, left, right, back = (1 / self.measures.irSensor[j] for j in range(3))
        axis_index: int = 0 if self.axis == X else 1

        # Check if there is a wall at the front of the robot
        if not utils.eval_distance(self.measures.irSensor[CENTER_ID]) == "wall":
            self.logger.debug("Halted robot estimate: No front wall detected!")
            return False

        # Calculate the robot position estimate at self.axis based on the front wall
        pos_center: tuple = self._get_normalized_estimate()
        pos_center[axis_index] += .45 - center
        self.logger.debug(f"Halted robot estimate: {self.axis} = ({round(pos_center[axis_index], 3)})")

        # Average the result (50/50 or other)
        self.last_corr_pos[axis_index] = self.last_corr_pos[axis_index] * .5 + pos_center[axis_index] * .5
        return True

    def _get_normalized_estimate(self) -> tuple:
        """
        Calculate coordinates of the center of the cell where the robot is
            estimated to be in
        :return:
        """
        return tuple(int(2 * round(elem / 2)) for elem in self.last_corr_pos)


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
