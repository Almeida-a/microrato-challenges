# Utility functions
from typing import Any

from C2 import ternary_tree


def eval_distance(obstacle_sensor: float) -> int:
    """
    Assumes the robot is at the center of the cell
    :param obstacle_sensor: Obstacle sensor's value (any one)
    :return: WALL or CLEAR, if WALL, it means the obstacle detector is detecting a wall
    """
    distance_threshold: float = .6
    if obstacle_sensor > 1 / distance_threshold:
        return ternary_tree.WALL
    return ternary_tree.CLEAR


def get_key(dic: dict, value: Any) -> Any:
    for k, v in dic.items():
        if v == value:
            return k
    raise Warning("No keys were found!")


def opposite_angle(angle: float) -> float:
    """
    Assumes the given angle falls within [-180, 180[
    :param angle:
    :return:
    """
    if -180.0 > angle or angle >= 180.0:
        angle = angle % 360 - 180  # Normalize

    if angle < 0:
        return angle + 180
    else:
        return angle - 180
