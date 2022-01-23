# Utility functions
from typing import Any, List


def eval_distance(obstacle_sensor: float) -> str:
    """
    Assumes the robot is at the center of the cell
    :param obstacle_sensor: Obstacle sensor's value (any one)
    :return: WALL or CLEAR, if WALL, it means the obstacle detector is detecting a wall
    """
    distance_threshold: float = .7
    obstacle_sensor_dist = 1 / obstacle_sensor
    if obstacle_sensor_dist < distance_threshold:
        return "wall"
    return "clear"


def get_key(dic: dict, value: Any) -> Any:
    for k, v in dic.items():
        if v == value:
            return k
    raise Warning("No keys were found!")


def get_keys(dic: dict, value: Any) -> List[Any]:
    ret_vals: List[Any] = list()
    for k, v in dic.items():
        if v == value:
            ret_vals.append(k)
    return ret_vals


# def get_keys(dic: dict, values: tuple) -> tuple:
#     return tuple(get_key(dic, value=value) for value in values)


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
