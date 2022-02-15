"""
    References:
        https://www.pythonforbeginners.com/data-structures/tree-data-structure-in-python
"""

from typing import Tuple, Set, List

# enum
WALL: int = 1
CLEAR: int = 2
ORIGIN: int = 3
# Static variables
coordinates_set: Set[Tuple[int, int]] = set()


# Class utility functions
def _dead_end(ways: List[int], valid_check: bool) -> bool:
    assert (not valid_check) or _valid_crossroad(ways), "Invalid crossroad depiction"
    if ORIGIN in ways:
        ways.remove(ORIGIN)
    # The remaining ways are walls
    return all([way == WALL for way in ways])


def _valid_crossroad(ways: List[int]) -> bool:
    """
    Conditions:
        1 - One and only one ORIGIN
        2 - Others are of type CLEAR or WALL
        One of the following:
            3 - More than one CLEAR (meaning there is only one path ahead, excluding the one the C4 came from)
            4 - Dead end [see _dead_end], although not techically a crossroad, it is useful here to define as such
    :param ways:
    :return: All conditions are met => True, False otherwise
    """
    # Condition 1
    if ORIGIN not in ways:
        return False
    ways.remove(ORIGIN)
    if ORIGIN in ways:
        return False

    # Condition 2
    c2: bool = all([way in (CLEAR, WALL) for way in ways])

    # Condition 3
    tmp: list = list(filter(lambda way: not (way == CLEAR), ways))
    c3: bool = len(tmp) <= 1

    # Condition 4
    c4: bool = _dead_end(ways, valid_check=False)

    return c2 and (c3 or c4)


class TreeNode:
    def __init__(self, ways: tuple, x: int, y: int):
        # Pointer to parent node
        self.parent: [TreeNode, None] = None
        # Pointers to other nodes
        self.first: [TreeNode, None] = None
        self.second: [TreeNode, None] = None
        self.third: [TreeNode, None] = None
        # Values of the node itself
        self.ways: Tuple[int, int, int, int] = ways  # format = (W, S, E, N) (counter-clockwise starting from WEST)
        self.coordinates: Tuple[int, int] = (x, y)
        self.repeated_coordinates: bool = self.coordinates in coordinates_set
        # Adds if not existing
        coordinates_set.add(self.coordinates)
        # Not explored by default
        self.explored = False

    def add_child_node(self, which: int, node) -> int:
        """

        :param which:
        :type node: TreeNode
        :return: Status code ->
            1 means unexpected return,
            0 means correct execution has been carried out
        """
        if self.is_node_explored() or _dead_end(list(self.ways), valid_check=True):
            return 1

        # Node's walls configuration must be valid!
        assert _valid_crossroad(list(node.ways)), "Invalid mappings!"

        # Set current node as parent to the argument node (child)
        node.parent = self

        if which == 1:
            if self.first is None:
                self.first = node
                return 0
            return 1
        elif which == 2:
            if self.second is None:
                self.second = node
                return 0
            return 1
        elif which == 3:
            if self.third is None:
                self.third = node
                return 0
            return 1
        else:
            raise AssertionError(f"Unexpected argument: child node no.{which} is not valid!")

    def is_node_explored(self) -> bool:
        """
        Conditions:
            - Node was already marked as explored
            - Another node already has these coordinates
            - This node corresponds with a dead end in the maze
            - All of the 3 child nodes are fully explored
        :return: True if any of the mentioned 3 conditions prove True
        """
        if self.explored:
            return True
        else:
            leaf: bool = any((
                self.repeated_coordinates,  # Consider as a leaf node
                # valid check: actively check if the node is root (0, 0), which is irregular,
                #   making sure the function takes that into account
                _dead_end(ways=list(self.ways), valid_check=not (self.coordinates == (0, 0)))  # Another leaf node
            ))
            if leaf:
                self.explored = True
                return True
            else:
                # Check if any None child nodes does not corresponding to a wall
                # return False if so
                for i, child in enumerate((self.first, self.second, self.third)):
                    if self.ways[i] == ORIGIN:
                        i += 1
                        continue
                    if child is None:
                        if self.ways[i] == CLEAR:
                            return False
                    else:  # if child has been created
                        assert self.ways[i] == CLEAR, "Created pathway through a wall!"
                        if not child.is_node_explored():
                            return False
        return False

    def next_untraveled(self) -> Tuple[str, int]:
        """
        If any child node is not fully explored, return the direction to it.
        Else, return the direction to the parent node
        :return: Direction and which child node (int) should be created: first, second or third
            Or 0 if all were fully traveled over
        """
        compass_axis: Tuple[str, str, str, str] = ("west", "south", "east", "north")
        if self.is_node_explored():
            return compass_axis[self.ways.index(ORIGIN)], 0
        else:
            gone_over_origin: bool = False
            for i, child in enumerate((self.first, self.second, self.third)):
                if self.ways[i] == ORIGIN:
                    i += 1
                    gone_over_origin = True
                    continue
                else:
                    if not child.is_node_explored():
                        return compass_axis[i], i - gone_over_origin
            raise AssertionError("All child nodes explored but node is marked as unexplored!")


# Robot C4 related functions
def is_crossroad_or_root(west: int, south: int, east: int, north: int, node: [TreeNode, None]) -> bool:
    """
    :param node: node of the tree
    :param west:
    :param south:
    :param east:
    :param north:
    :return: True if this is conceptually a crossroad or a dead end
    """
    if node is None:
        return True
    return _valid_crossroad(ways=[west, south, east, north]) or node.coordinates == (0, 0)
