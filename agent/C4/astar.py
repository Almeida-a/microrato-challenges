import itertools
from typing import List, Tuple

# Distance from each cell to the closest neighbor
UNIT: int = 2


# This class represents a node (cell in the grid)
class Node:

    def __init__(self, position: Tuple[int, int], parent: ()):
        self.position = position
        self.parent = parent
        self.g: int = 0  # Distance to start node
        self.h: int = 0  # Distance to goal node
        self.f: int = 0  # Total cost

    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position

    # Sort nodes
    def __lt__(self, other):
        return self.f < other.f

    # Print node
    def __repr__(self):
        return '({0},{1})'.format(self.position, self.f)

    def generate_heuristics(self, start_node, goal_node):
        """
        Calculates the distance to start (g) and goal (h) nodes
        :param start_node:
        :param goal_node:
        :return: None
        """
        curr_x, curr_y = self.position
        start_x, start_y = start_node.position
        goal_x, goal_y = goal_node.position
        self.g = abs(curr_x - start_x) + abs(curr_y - start_y)
        self.h = abs(curr_x - goal_x) + abs(curr_y - goal_y)
        self.f = self.g + self.h


def write_path(path: List[Tuple[int, int]], beacons):
    """

    :param beacons:
    :param path: List of (x, y) tuples consisting of a path
    :return:
    """

    # Counter
    count: int = 1
    # Open file
    f = open("planning.out", "w")
    # Write list elements, line by line
    f.write(f"0 0")
    for pos in path:
        f.write("\n")
        f.write(f"{pos[0]} {pos[1]}")
        if pos in beacons:
            f.write(f" #{count}")
            count += 1
    # Close stream
    f.close()


def get_char(map_matrix, x: int, y: int) -> str:
    """

    :param map_matrix:
    :param x:
    :param y:
    :return:
    """
    # Get the coordinates of cell 0
    center_i, center_j = int(len(map_matrix) / 2), int(len(map_matrix[0]) / 2)

    # Assert center is 0
    assert map_matrix[center_i][center_j] == '0', f"Center of map expected 0, found {map_matrix[center_i][center_j]}"

    return map_matrix[center_i - y][center_j + x]


def astar_search(map_matrix: List[List[str]], start: tuple, end: tuple) -> List[Tuple[int, int]]:
    """
        Use A* algorithm to compute path from start to end
        :param map_matrix:
        :param start:
        :param end:
        :return:
        """

    # Create lists for open nodes and closed nodes
    open_: list = []
    closed: list = []

    # Create a start node and a goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)

    # Add the start node
    open_.append(start_node)

    # Loop until open list is empty,
    #   in other words, until a solution is found or it is confirmed that there are none
    while len(open_) > 0:

        # Sort the open list to get the node with the lowest cost first
        open_.sort()

        # Get the node with the lowest cost
        current_node = open_.pop(0)

        # Add the current node to the closed list
        closed.append(current_node)

        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            # Return reversed path
            return path[::-1]

        # Unzip the current node position
        (x, y) = current_node.position

        # Get neighbors
        neighbors = [(x - UNIT, y), (x + UNIT, y), (x, y - UNIT), (x, y + UNIT)]

        # Loop neighbors
        for next_ in neighbors:

            # Get value from map
            map_value = get_char(map_matrix, x=int((x+next_[0])/2), y=int((y+next_[1])/2))

            # Check if the node is a wall
            if map_value in ("-", "|"):
                continue

            # Create a neighbor node
            neighbor = Node(next_, current_node)

            # Check if the neighbor is in the closed list
            if neighbor in closed:
                continue

            # Generate heuristics (Manhattan distance)
            neighbor.generate_heuristics(start_node, goal_node)

            # Check if neighbor is in open list and if it has a lower f value
            if neighbor not in open_ or neighbor.f < min(*[elem.f for elem in open_]):
                open_.append(neighbor)

    print("Solution not found. Exiting")
    exit(1)


def get_pos(map_matrix: List[List[str]], char: str) -> Tuple[int, int]:
    """

    :param map_matrix:
    :param char:
    :return:
    """

    # Axiomatic property
    if char == '0':
        return 0, 0

    # Get the coordinates of cell 0
    center_i = int(len(map_matrix) / 2)
    center_j = int(len(map_matrix[0]) / 2)

    # Get row, col of first character found with value char
    for row in map_matrix:
        if char in row:
            i, j = map_matrix.index(row), row.index(char)
            break
    else:
        raise AssertionError(f"Character {char} not present in map matrix!")

    # Parse row, col to x, y
    x = j - center_j
    y = center_i - i

    return x, y


def compute_path(map_matrix: List[List[str]], beacons) -> List[Tuple[int, int]]:

    checkpoints_pos: tuple = ((0, 0), *beacons)

    # Searches 0 -> 1 -> 2 -> 3 -> 0
    solution: List[Tuple[int, int]] = list()
    for i in range(len(checkpoints_pos)):
        # Calculate paths from 0 -> 1 -> 2 -> 3 -> 0
        for pos in astar_search(map_matrix, checkpoints_pos[i], checkpoints_pos[(i+1) % len(checkpoints_pos)]):
            # Append search to final solution
            solution.append(pos)

    return solution


def read_map(map_filename: str) -> List[List[str]]:
    f = open(file=map_filename, mode="r")

    lines: List[str]
    map_matrix: List[List[str]] = list()

    lines = f.read().split("\n")
    for line in lines:
        # Parse line string into an array of characters and
        # Append array into matrix
        map_matrix.append(list(line))

    f.close()

    return map_matrix[:-1]  # Trim last line


def calc_path(map_filename: str):
    """

    :param map_filename: Name of the file containing the map
    :return: Create file containing the path between sensors - "solution.path"
    """
    # Read map into a matrix
    map_matrix: List[List[str]] = read_map(map_filename)
    # Get beacons
    beacons: List[Tuple[int, int]] = [get_pos(map_matrix, char) for char in ('1', '2', '3')]
    # Read matrix into a list of tuples (x, y) and find the shortest list from permutations of beacons
    path = []
    for beacons_shuffled in itertools.permutations(beacons):
        tmp: List[Tuple[int, int]] = compute_path(map_matrix, beacons=beacons_shuffled)
        if not path or path and len(tmp) < len(path):
            path = tmp.copy()
    # Write path into solution.path
    write_path(path, beacons)


if __name__ == '__main__':
    calc_path("solution.map")
