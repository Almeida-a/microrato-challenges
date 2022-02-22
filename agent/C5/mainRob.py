import csv
import logging
import logging.config
import sys
import xml.etree.ElementTree as ET
from typing import Tuple, List, Dict, Union

import numpy as np
import yaml

import utils
from croblink import *

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
UNEXPLORED: int = 100
PART_EXPLORED: int = 200
ORIGIN: int = 300
EXPLORED: int = 400
WALL: int = 500
# Explore modes
EXPLORE: int = 110
RETURN: int = 220
# -------------------

# POSSIBLE_MOVEMENT_TYPES: tuple = ("explore", "return")
POSSIBLE_ACTIONS: tuple = ("front", "left", "right", "back", "finished")
AXIS_PRECEDENCE: tuple = ("WEST", "SOUTH", "EAST", "NORTH")

# Max angle error measure value (assuming always > 0)
MAX_ANGLE_ERROR: float = 180.0
# Map initial char
MAP_DEFAULT_CHAR: str = ' '

# --------------------------- Major Configurations -----------------------------

# The robot tries to go towards the beacon 0 - DEPRECATE
SEARCH_FOR_BEACON_0: bool = False

# ----------------------------- TUNABLE constants ------------------------------

# Minimum distance between two measurements: for the agent to estimate the beacon's position
MIN_DISTANCE_B0_POS: float = 2.0
# The closest a robot can be to the front wall without stopping
WALL_MAX_PROXIMITY: float = .47
# Velocity upper bound
MAX_POW: float = .1
# When angle error falls below this value, velocity begins to decelerate
ROTATION_DECELERATION_THRESH: float = 30.0
# When translation error falls below, velocity begins to decelerate
TRANSLATION_DECELERATION_THRESH: float = .5
# Increment correction weight to the orientation process
ANGLE_RELATIVE_WEIGHT: float = .0
ANGLE_ABSOLUTE_WEIGHT: float = .8


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
    return AXIS_PRECEDENCE[
        min([AXIS_PRECEDENCE.index(axis) for axis in axis_list])
    ]


def calculate_intersection(m1: float, b1: float, m2: float, b2: float) -> Tuple[float, float]:
    """
    Computes the intersection point of the lines y = m1*x + b1 and y = m2*x + b2
    :param m1:
    :param b1:
    :param m2:
    :param b2:
    :return: The intersection point (x, y)
    """

    # Unsolvable equations
    if m1 == m2:
        return (sys.maxsize,) * 2

    # Calculate the solution
    x: float = (b2 - b1) / (m1 - m2)
    y: float = m1 * x + b1

    # Mathematical check
    assert y == m2 * x + b2, "Mathematical impossibility!"

    return x, y


class MyRob(CRobLinkAngs):
    # Logging configuration load
    with open('logging.yml', 'r') as stream:
        config = yaml.load(stream, Loader=yaml.FullLoader)
    logging.config.dictConfig(config)

    def __init__(self, rob_name, rob_id, angles, host):
        self.rob_name = rob_name
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # Logger
        self.logger = logging.getLogger("mainRob_file")
        # Keep the wheels' power values from every previous iteration
        self.rot_m1: float = .0
        self.lin_m1: float = .0
        self.eff_pow: Tuple[float, float] = (.0, .0)  # outL, outR

        # Target pose
        self.target_pose: Dict[int, int] = {X: 0, Y: 0, ROT: 0}
        # Micro-action command
        self.action: str = "start"
        self.next_action: str = ""
        # Initialize explore mode
        self.explore_mode: int = EXPLORE
        # Associates a crossroad's center coordinates to the crossroads where they lead to
        # e.g.: (x, y) -> {north=(x1, y1), east=ORIGIN, west=(x2, y2), south=EXPLORED}
        self.crossroads: Dict[Tuple[int, int], Dict[str, Union[Tuple[int, int], int]]] = {}
        # Hold position estimation with M.M. + with
        self.last_corr_pos: List[float, float] = [.0, .0, .0]
        # Axis - can be X, Y or ROT
        self.axis: int = -1
        # Last sensor value (either back, center or side*)
        self.sensors_m1: List[float] = [-1., -1., -1., -1.]
        # Keep the last not-corrected pose estimation
        self.last_mm_pos: Tuple[float, float, float] = .0, .0, .0
        # Keep the center coordinates of the last cell the robot was in
        self.last_normalized_coordinates: Tuple[int, int] = (0, 0)
        # Flag indicating that a correction was carried out at the x or y-axis!
        self.xy_correction: bool = False
        # Forces an action not to have immediate next actions
        self.single_action: bool = False

        # Mapping
        self.inner_map: List[List[str]] = [[MAP_DEFAULT_CHAR] * (4 * CELLCOLS - 1) for k in range(4 * CELLROWS - 1)]
        self.target_count = 0

        # --- C5 vars ---
        # Beacon sensor measurement data from a previous cycle AND the x, y, angle of that cycle
        self.save_point: Tuple[Tuple[bool, float], Tuple[float, float, float]] = (False, .0), (.0, .0, .0)
        # Position of the beacon 0 in the map with respect to the spawn cell
        # Max-size values means that its location is still unknown
        self.beacon_0_pos: Tuple[int, int] = sys.maxsize, sys.maxsize

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
        spawn_position: tuple = (self.measures.x, self.measures.y, self.measures.compass)

        with open('pos.csv', mode='w') as f:
            pos_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            pos_writer.writerow(["est_x", "est_y", "est_ang", "real_x", "real_y", "real_ang"])

            while True:
                self.readSensors()

                if self.measures.endLed is True:
                    self.write_inner_map("solution.map")
                    print(self.rob_name + " exiting")
                    quit()

                if state == 'stop' and self.measures.start:
                    state = stopped_state

                if state != 'stop' and self.measures.stop:
                    stopped_state = state
                    state = 'stop'

                self.logger.info(f"State: {state}")

                if state != "stop":
                    # GPS coordinates
                    x: float = self.measures.x - spawn_position[0]
                    y: float = self.measures.y - spawn_position[1]
                    angle: float = self.measures.compass - spawn_position[2]

                    # If robot is rotating, there is no point in finding the axis
                    if self.axis != ROT:
                        self._update_axis()  # Set axis according to the robot's orientation

                    # Log state and real coordinates
                    self.logger.debug(f"Current real coordinates are: (x,y,ang) = "
                                      f"{(round(x, 3), round(y, 3), round(angle, 3))}")

                    # Save this measurement for later calculation of the beacon sensor
                    if self.save_point[0][0] is False and self.measures.beacon[0][0] is True:
                        self.save_point = tuple(self.measures.beacon), tuple(self.last_corr_pos,)

                    # If no correction at (x,y) was performed: Set to mov. model's results
                    if not self.xy_correction:
                        self.last_corr_pos[0:2] = self.last_mm_pos[0:2]
                    # Reset the flag
                    self.xy_correction = False

                    # Compute position change using odometry
                    self._movement_model()

                    # Type of correction depends on whether the beacon 0 was already found or not
                    if sys.maxsize not in self.beacon_0_pos:
                        if self._angle_absolute_correction() is False:
                            self.last_corr_pos[2] = self.last_mm_pos[2]
                        else:
                            self.logger.debug("Absolute rot correction performed!")
                    elif self.save_point[0][0] and self.measures.beacon[0][0]\
                            and abs(self.save_point[1][0] - self.last_corr_pos[0]) > MIN_DISTANCE_B0_POS:
                        # Compute the position of the beacon

                        # Calculate the line parameters of the current measurement
                        angle = self.last_corr_pos[2] + self.measures.beacon[0][1]  # degrees
                        x, y = self.last_corr_pos[0:2]

                        slope = np.arctan(np.deg2rad(angle))
                        offset = y - slope * x

                        # Calculate the line parameters of the previous measurement
                        prev_angle = self.save_point[1][2] + self.save_point[0][1]
                        prev_x, prev_y = self.save_point[1]

                        prev_slope = np.arctan(np.deg2rad(prev_angle))
                        prev_offset = prev_y - prev_slope * prev_x

                        self.beacon_0_pos = calculate_intersection(
                            m1=slope, b1=offset, m2=prev_slope, b2=prev_offset
                        )

                        self.logger.info(f"Beacon estimated position is: (x, y) = {self.beacon_0_pos}")

                    self.logger.debug(f"Beacon measure is: {self.measures.beacon}")

                    # Write in csv
                    pos_writer.writerow([x, y, angle, *self.last_corr_pos])

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

                # Save values of sensor as memory for the next loop iteration
                self.sensors_m1 = self.measures.irSensor

    def wander(self):

        if self.measures.collision is True:
            self.logger.critical("Robot has crashed!")
            exit(1)

        # If close enough to the target cell, proceed
        if self._is_micro_action_complete():
            # if beacon is found
            if self.measures.ground == 1 and sys.maxsize in self.beacon_0_pos:
                self.beacon_0_pos = self._get_normalized_estimate()
                assert False, "Forced breakpoint"

            # Save info to the internal map
            self.write_cell_to_map()
            # Decide next actions (move 1 cell front or turn)
            #   based on the walls detected
            self._next_micro_action()
            self._translate_micro_command()
            # A second kind of position correction: if correction is performed, set flag to true
            self.xy_correction |= self._halted_robot_correction()

            assert self.action in POSSIBLE_ACTIONS, f"Action: {self.action} not an action!"

        # Move
        self._do_micro_action()
        # Logs
        self._log_cycle_class_vars()

    def _movement_model(self):
        """
        :return: Based on the:
            * estimated effective wheel power values,
            * last estimated pose,
             Estimate new pose
        """
        # Unpack variables
        out_l, out_r = self.eff_pow
        x_m1, y_m1, angle_m1 = self.last_corr_pos
        lin = (out_r + out_l) / 2
        rot = out_r - out_l
        self.logger.debug(f"Linear velocity: {round(lin, 3)}")
        self.logger.debug(f"Rotational velocity: {round(rot, 3)}")

        # Calculate new x coordinates
        x: float = x_m1 + lin * np.cos(np.radians(angle_m1))
        # Calculate new y coordinates
        y: float = y_m1 + lin * np.sin(np.radians(angle_m1))
        # Calculate new orientation value
        angle: float = angle_m1 + np.rad2deg(rot)

        self.logger.debug(f"New m.m. coordinates: x = ({round(x, 3)}),"
                          f" y = ({round(y, 3)}), angle = ({round(angle, 3)})")
        # Output
        self.last_mm_pos = (x, y, angle)

    def _calculate_effective_powers(self, in_r: float, in_l: float):
        """

        :return: Estimates each wheel's effective power based on the commands given to them.

        """

        # Normalize in_{l,r} to their max power if applicable
        if abs(in_l) > MAX_POW:
            in_l = MAX_POW if in_l > 0 else -MAX_POW
        if abs(in_r) > MAX_POW:
            in_r = MAX_POW if in_r > 0 else -MAX_POW

        # Output
        self.eff_pow = (
            (in_l + self.eff_pow[0]) / 2,
            (in_r + self.eff_pow[1]) / 2
        )

        self.logger.debug("Directed power: "
                          f"inL = {round(in_l, 3)}, "
                          f"inR = {round(in_r, 3)}.")
        self.logger.debug(f"Effective power: "
                          f"outL = {round(self.eff_pow[0], 3)}, "
                          f"outR = {round(self.eff_pow[1], 3)}.")

    def _update_axis(self):
        # Evaluate if N/S/E/W, then update axis and other variables as needed
        orientation: str = self._get_orientation_axis()

        self.logger.debug(f"Current axis: {orientation}")

        if orientation in ("NORTH", "SOUTH"):
            self.axis = Y
        else:
            self.axis = X

    def _get_orientation_axis(self) -> str:
        return self._get_axis_for_side(CENTER_ID)

    def _get_axis_for_side(self, side_id: int) -> str:

        orientation: List[str]

        if -45 < self.last_corr_pos[2] <= 45:
            orientation = ["EAST", "NORTH", "SOUTH", "WEST"]
        elif -135 < self.last_corr_pos[2] <= -45:
            orientation = ["SOUTH", "EAST", "WEST", "NORTH"]
        elif 45 < self.last_corr_pos[2] <= 135:
            orientation = ["NORTH", "WEST", "EAST", "SOUTH"]
        else:  # Below -135 or above 135
            orientation = ["WEST", "SOUTH", "NORTH", "EAST"]

        return orientation[side_id]

    def _get_axis_for_angle(self, angle: float) -> str:
        """

        :param angle:
        :return:
        """

        if 180 <= angle or angle < -180:
            self.logger.warning(f"Angle is outside the normal scope: {angle}!")
            angle = (angle + 180) % 360 - 180

        orientation: str

        if -45 < angle <= 45:
            orientation = "EAST"
        elif -135 < angle <= -45:
            orientation = "SOUTH"
        elif 45 < angle <= 135:
            orientation = "NORTH"
        else:  # Below -135 or above 135
            orientation = "WEST"

        return orientation

    def _is_micro_action_complete(self):

        if self.action == "start":
            return True

        # Tolerance margins from the target pose
        margins: Dict[str, float] = dict(x=.20, y=.20, turn=5.)  # margin values can be tuned

        error_x: float = abs(self.last_corr_pos[0] - self.target_pose[X])
        error_y: float = abs(self.last_corr_pos[1] - self.target_pose[Y])
        error_angle: float = abs(self._get_angle_error())

        # If the walls are critically close, force stop and signal completion
        d_front = 1 / self.measures.irSensor[CENTER_ID]
        if self.action == "front" and d_front < WALL_MAX_PROXIMITY:
            self.logger.debug(f"Critically close (d = {d_front}) to front wall!")
            self.target_pose[X], self.target_pose[Y] = self._get_normalized_estimate()
            self.logger.info(f"Reset target position: (x, y) = {tuple(self.target_pose[ax] for ax in (X, Y))}")
            if self._mend_crossroad() is True:
                self.logger.warning(f"Mending was applied!")
            self.logger.debug("Micro-action COMPLETE")
            return True

        # Check closeness from the:
        # * x-axis
        if error_x > margins["x"] and self.axis == X:
            self.logger.debug(f"Not close enough (e = {error_x}) from X set point")
            return False
        # * y axis
        if error_y > margins["y"] and self.axis == Y:
            self.logger.debug(f"Not close enough (e = {error_y}) from Y set point")
            return False
        # * turn axis (rotation)
        if self.axis == ROT:
            if error_angle > margins["turn"]:
                # We shrank the tolerance here since it is more important for the robot's angle to
                #   be correct in this case because it is likely starting an action of "going ahead",
                #   propagating any angle error to larger proportions
                self.logger.debug(f"Not close enough (e = {error_angle}) from Angle set point")
                return False
            # If robot finishes rotating, try a correction
            self.xy_correction |= self._halted_robot_correction()
        else:
            if error_angle > margins["turn"] * 4:  # 4 times more margin
                # In this case, the robot is going ahead and the self.correct_pose is already adjusting
                #   the C4's orientation, so a slightly larger slack is acceptable
                self.logger.debug(f"Not close enough (e = {error_angle}) from Angle set point")
                return False

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

        # Check if last cell was a crossroad
        if self.last_normalized_coordinates in self.crossroads.keys() and self.explore_mode == EXPLORE:
            # Assert consistent logic
            assert self.crossroads[self.last_normalized_coordinates][self._get_orientation_axis()] in (
                UNEXPLORED, PART_EXPLORED
            ), f"Inconsistent logic on cell {self.last_normalized_coordinates}."
            # Set the traversed pathway as partially explored
            self.crossroads[
                self.last_normalized_coordinates][self._get_orientation_axis()] = PART_EXPLORED

        # * Dead-end cell
        if walls_count == 3:
            assert self.explore_mode == EXPLORE, f"Explore expected, found " \
                                                 f"{'Return' if self.explore_mode == RETURN else self.explore_mode}"
            # Command -> Go back
            self.explore_mode = RETURN
            self.action = "back"
        # * Crossroad -> Go to next unexplored passage--
        elif walls_count in (0, 1) or normalized_coordinates == (0, 0):

            assert len(normalized_coordinates) == 2, f"Unexpected number of elements in normalized coordinates."

            if normalized_coordinates not in self.crossroads.keys():
                # New crossroad
                # -> add to c-dict and add info about the walls/passages
                self.crossroads[normalized_coordinates] = dict()
                for cardinal_orientation, passage in walls_per_axis.items():
                    if passage == "wall":
                        self.crossroads[normalized_coordinates][cardinal_orientation] = WALL
                    elif normalized_coordinates != (0, 0) and cardinal_orientation == self._get_axis_for_side(BACK_ID):
                        # Assign back side as ORIGIN (as long as this is not the root cell)
                        self.crossroads[normalized_coordinates][cardinal_orientation] = ORIGIN
                    else:
                        self.crossroads[normalized_coordinates][cardinal_orientation] = UNEXPLORED
            else:
                # Known crossroad case
                if self.explore_mode == RETURN:
                    # Mark the pathway the robot came from as explored
                    self.crossroads[normalized_coordinates][self._get_axis_for_side(BACK_ID)] = EXPLORED
                    # Reset to exploring mode
                    self.explore_mode = EXPLORE

            self._goto_unexplored(walls_per_dir, normalized_coordinates)

        # * Normal road -> Continue w/o turning back ---------------------
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
        else:
            raise AssertionError(f"Unexpected number of walls: {walls_count}.")

        # If robot turns, give double command (makes no sense to do 2 turns in this logic)
        #   except when the robot is turning to look for the beacon
        if self.action != "front":
            if self.single_action is False:
                self.next_action = "front"
            # Reset flag
            self.single_action = False  # This flag is only valid for the current cycle

        self.last_normalized_coordinates = self._get_normalized_estimate()

    def _do_micro_action(self):

        lin: float = .0
        rot: float = .0

        angle_error: float = self._get_angle_error()  # between -180 and 180

        if self.axis in (X, Y):

            axis_index: int = 1 if self.axis == Y else 0

            left_dist: float = 1 / self.measures.irSensor[LEFT_ID]
            right_dist: float = 1 / self.measures.irSensor[RIGHT_ID]

            # Checks if the robot is in a corridor (has walls on both sides in the current cell)
            is_corridor: bool = utils.eval_distance(
                self.measures.irSensor[LEFT_ID], distance_threshold=.55) == "wall" \
                                and utils.eval_distance(
                self.measures.irSensor[RIGHT_ID], distance_threshold=.55) == "wall"

            # Measures how far the robot is from the center of the cell (using side sensors)
            deviation_error: float = left_dist - right_dist
            if is_corridor and deviation_error > utils.WALL_MAX_DIST - 0.4:
                self.logger.warning("Robot is straying too much from the center at this corridor!")

            # Scope: 0 <= error <= 1
            error: float = (angle_error / 90.0) * .3 + (deviation_error / 1.0) * .7
            if not is_corridor:
                error = angle_error / 90.
            rot = MAX_POW * error

            self.logger.debug(f"Is corridor: {is_corridor}, "
                              f"deviation error: {round(deviation_error, 3)}, "
                              f"angle error: {round(angle_error, 3)}")

            # Linear velocity value should not make the global velocity surpasses MAX_POW
            max_lin = MAX_POW - rot
            k: float = 1.
            if len(self.last_corr_pos) == 3:
                # Get the difference from current robot position and set point
                error: float = abs(self.last_corr_pos[axis_index] - self.target_pose[self.axis])
                k = min(error / TRANSLATION_DECELERATION_THRESH, 1.)

            lin = max_lin * k

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
        axis: str = self._get_orientation_axis()
        walls: List[str] = [utils.eval_distance(wall) for wall in self.measures.irSensor]
        walls_dir: Dict[int, str] = {side: utils.eval_distance(self.measures.irSensor[side]) for side in range(4)}

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
        """

        :return:
        """
        # Holds the distances of the corresponding cardinal axis
        north, south, east, west = [self.measures.irSensor[POSSIBLE_ACTIONS.index(self._what_side(ax))]
                                    for ax in ("NORTH", "SOUTH", "EAST", "WEST")]
        # Get the (local) position coordinates of the robot in the map
        x, y = self._get_normalized_estimate()
        # Maps the position coordinates to the inner map's
        u, v = (CELLROWS * 2 - y, CELLCOLS * 2 + x)

        # Write 'X' or 'O'/'1'/'2'/'3' to the current cell if cell hasn't been written on before
        if ((x, y) == (0, 0) or self.measures.ground is True) and self.inner_map[u][v] == MAP_DEFAULT_CHAR:
            self.inner_map[u][v] = f"{self.target_count}"
            self.target_count += 1
        elif self.inner_map[u][v] == MAP_DEFAULT_CHAR:
            self.inner_map[u][v] = "X"

        # Write '-' to NORTH or SOUTH cells, if wall exists, and it has not been written on before
        if utils.eval_distance(north) == "wall" and self.inner_map[u - 1][v] == MAP_DEFAULT_CHAR:
            self.inner_map[u - 1][v] = '-'
        elif self.inner_map[u - 1][v] == MAP_DEFAULT_CHAR:
            self.inner_map[u - 1][v] = 'X'
        if utils.eval_distance(south) == "wall" and self.inner_map[u + 1][v] == MAP_DEFAULT_CHAR:
            self.inner_map[u + 1][v] = '-'
        elif self.inner_map[u + 1][v] == MAP_DEFAULT_CHAR:
            self.inner_map[u + 1][v] = 'X'

        # Write '|' to EAST or SOUTH cells, if wall exists, and it has not been written on before
        if utils.eval_distance(west) == "wall" and self.inner_map[u][v - 1] == MAP_DEFAULT_CHAR:
            self.inner_map[u][v - 1] = '|'
        elif self.inner_map[u][v - 1] == MAP_DEFAULT_CHAR:
            self.inner_map[u][v - 1] = 'X'
        if utils.eval_distance(east) == "wall" and self.inner_map[u][v + 1] == MAP_DEFAULT_CHAR:
            self.inner_map[u][v + 1] = '|'
        elif self.inner_map[u][v + 1] == MAP_DEFAULT_CHAR:
            self.inner_map[u][v + 1] = 'X'

    def _what_side(self, target_axis: str) -> str:
        """

        :param target_axis: Cardinal axis
        :return: Given the orientation of the robot, what side of it is the referenced axis?
        E.g.: NORTH is to the right side of a WEST faced C4
        """
        front_side_axis: str = self._get_orientation_axis()
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
        self.logger.debug(f"Obstacle distances: {[1 / self.measures.irSensor[j] for j in range(4)]}")
        self.logger.debug(f"Crossroads info: {self.crossroads}.")
        self.logger.debug(f"Current axis: {self.axis}")
        self.logger.debug(f"Robot position estimate: x=({round(self.last_corr_pos[0], 2)}), "
                          f"y=({round(self.last_corr_pos[1], 2)}), angle=({round(self.last_corr_pos[2], 2)})")
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
            self._update_axis()  # update axis to X or Y
            self.target_pose[self.axis] += move_translation[self._get_orientation_axis()] * 2

        # Normalize target pose angle attribute
        self.target_pose[ROT] = (self.target_pose[ROT] + 180) % 360 - 180

    def _get_angle_error(self) -> float:
        """

        :return: Difference between target angle and actual robot angle, max 2 or -2, if <0.
        """
        a1: float = self.target_pose[ROT]
        a2: float = self.last_corr_pos[2]

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
        Assuming this function is called after the completion of a micro-action,
            this function calculates the robot's position regarding the center of the cell
            based on the: front wall (more walls may be taken into account in the future)
        :return: True if a correction was carried out
        """
        center, _, __, back = (1 / self.measures.irSensor[j] for j in range(4))
        if self._get_orientation_axis() in ("NORTH", "SOUTH"):
            axis_index: int = 1  # Y
        elif self._get_orientation_axis() in ("EAST", "WEST"):
            axis_index: int = 0  # X
        else:
            raise AssertionError(f"Invalid cardinal orientation: {self._get_orientation_axis()}")

        # Check if there is a wall at the front of the robot
        back_exists, front_exists = (utils.eval_distance(self.measures.irSensor[_id]) == "wall" for _id in
                                     (BACK_ID, CENTER_ID))
        if not back_exists and not front_exists:
            self.logger.debug("Halted robot estimate: No front nor back wall detected!")
            return False

        # Calculate the robot position estimate at self.axis based on the front and back wall
        pos_center = list(self._get_normalized_estimate())
        optimal_wall_dist: float = .45  # .5 when wall width is 0.0, .4 when 0.1
        orientation_signal: int = 1 if self._get_orientation_axis() in ("SOUTH", "WEST") else -1
        if front_exists:
            pos_center[axis_index] += orientation_signal * (optimal_wall_dist - center)
        if back_exists:
            pos_center[axis_index] -= orientation_signal * (optimal_wall_dist - back)
        self.logger.debug(f"Halted robot estimate: {'x' if axis_index == 0 else 'y'} "
                          f"= ({round(pos_center[axis_index], 3)})")

        # Average the result (50/50 or other)
        self.last_corr_pos[axis_index] = self.last_corr_pos[axis_index] * .2 + pos_center[axis_index] * .8
        return True

    def _get_normalized_estimate(self) -> Tuple[int, int]:
        """
        Calculate coordinates of the center of the cell where the robot is estimated to be in
        :return:
        """
        assert len(self.last_corr_pos) in (2, 3), f"Incorrect coordinates format! {self.last_corr_pos}"
        x: int = int(2 * round(self.last_corr_pos[0] / 2))
        y: int = int(2 * round(self.last_corr_pos[1] / 2))
        return x, y

    def _goto_unexplored(self, walls_per_dir: dict, normalized_coordinates: tuple) -> bool:
        """
        Orders the robot to go to a [partially] unexplored crossroad. If none, go to ORIGIN pathway
        :param walls_per_dir:
        :param normalized_coordinates:
        :return: False if all pathways are fully explored, and so the robot is set to ORIGIN
        """
        backup_action: bool = False

        # Go to first unexplored
        for side_id in range(4):
            # The first clear pathway is the way to go
            status: int = self.crossroads[normalized_coordinates][self._get_axis_for_side(side_id=side_id)]
            if walls_per_dir[side_id] == "clear":
                if status == UNEXPLORED:
                    self.action = POSSIBLE_ACTIONS[side_id]
                    break
                elif status == PART_EXPLORED and backup_action is False:
                    self.action = POSSIBLE_ACTIONS[side_id]
                    backup_action = True
        else:
            # If a backup action exists, return
            if backup_action:
                return True
            # No pathways left to explore
            # Consistency assert
            assert all(
                (elem in (EXPLORED, ORIGIN, WALL) for elem in self.crossroads[normalized_coordinates].values())
            ), f"Inconsistent crossroad: {normalized_coordinates}: {self.crossroads[normalized_coordinates]}"
            # Set return mode
            self.explore_mode = RETURN
            # Go to origin (if robot has finished exploring, it is at spawn cell and none exists!)
            if ORIGIN not in self.crossroads[normalized_coordinates].values():
                self.finish()
            else:
                self.action = self._what_side(
                    utils.get_key(self.crossroads[normalized_coordinates], ORIGIN)
                )
            return False
        return True

    def _mend_crossroad(self) -> bool:
        """
        Mends the information held regarding the current crossroad
        :return: True if a change occurred
        """
        norm_pos: tuple = self._get_normalized_estimate()
        # Check if current cell is in the crossroad dict
        if not (norm_pos in self.crossroads.keys()):
            return False
        # Check if current cell is a crossroad. If not, remove from list and return
        if [utils.eval_distance(sensor) for sensor in self.measures.irSensor].count("clear") <= 2:
            self.crossroads.pop(norm_pos)
            return True
        changed: bool = False
        # Correct false pathways
        for key, value in self.crossroads[norm_pos].items():
            # Get side from key
            side: int = POSSIBLE_ACTIONS.index(self._what_side(key))
            # Detect and correct false pathway, if such exists
            if value in (UNEXPLORED, ORIGIN, PART_EXPLORED) \
                    and utils.eval_distance(self.measures.irSensor[side]) == "wall":
                # False pathway detected. Carry out correction
                self.crossroads[norm_pos][key] = WALL
                changed = True
        return changed

    def write_inner_map(self, filename: str):
        f = open(filename, mode="w")
        for i, row in enumerate(self.inner_map):
            for j, value in enumerate(row):
                f.write(value)
            f.write("\n")
        f.close()

    def _angle_relative_correction(self) -> bool:
        """
        Conditions: robot must not be moving its position (only its orientation)
        Use beacon sensor to measure the increment in the angle of the orientation of the robot
        :return:
        """

        # assert self.measures.beaconReady is True, "BeaconReady is false!"

        # If beacon 0 is not visible in this cycle OR wasn't visible in the last one, abort
        if not self.measures.beacon[0][0] or not self.beacon_0_save[0] or self.axis != ROT:
            return False

        # Check the difference between the last and current cycle's beacon measurement
        now: float = self.measures.beacon[0][1]
        last: float = self.beacon_0_save[1]

        # Check the difference between the last and current cycle's mov. model estimate
        mm_diff: float = self.last_mm_pos[2] - self.last_corr_pos[2]

        # Carry out the increment
        self.last_corr_pos[2] += mm_diff * (1 - ANGLE_RELATIVE_WEIGHT) \
                                 + (last - now) * ANGLE_RELATIVE_WEIGHT

        return True

    def _angle_absolute_correction(self) -> bool:
        """
        Obtains the absolute angle value, provided the robot knows where the beacon 0 is
        :return: True if: the beacon 0 is visible to the robot, and it is in a different cell. False otherwise
        """
        x, y = self.last_corr_pos[0:2]
        bx, by = self.beacon_0_pos

        if (x, y) == (bx, by):
            # Can't measure angle in these conditions!
            return False

        # Make sure this function is called only when the b0 position is known
        assert sys.maxsize not in (bx, by), "Position of beacon 0 is still unknown!"

        # Check if the beacon is visible
        if self.measures.beacon[0] is False:
            return False

        # Check if slope is infinite
        if bx == x:
            if by > y:
                self.last_corr_pos[2] = 0.0
            else:
                self.last_corr_pos[2] = -180.0
            return True

        # Compute the angle between the positive-y-axis (NORTH)
        #   and the line that unites the robot with the beacon0 (direction is: r -> b0)
        slope: float = (by - y) / (bx - x)
        beacon_robot_angle: float = np.rad2deg(np.arctan(slope))

        # Actual angle: sum up the angles of the beacon sensor with the angle of the slope
        robot_orientation_angle: float = self.measures.beacon[1] + beacon_robot_angle

        # Perform correction
        self.last_corr_pos[2] = robot_orientation_angle * ANGLE_ABSOLUTE_WEIGHT \
                                + self.last_corr_pos[2] * (1 - ANGLE_ABSOLUTE_WEIGHT)

        # Normalize the angle to interval [-180, 180[
        self.last_corr_pos[2] = (self.last_corr_pos[2] + 180) % 360 - 180

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
