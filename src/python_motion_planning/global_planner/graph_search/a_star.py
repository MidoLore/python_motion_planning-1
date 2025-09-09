"""
@file: a_star.py
@brief: A* motion planning in 3D
@author: Yang Haodong, Wu Maojia
@update: 2025.9.9
"""
import heapq

from .graph_search import GraphSearcher
from python_motion_planning.utils.environment.env import Env, Grid
from python_motion_planning.utils.environment.node import Node


class AStar(GraphSearcher):
    """
    Class for A* motion planning in 3D.

    Parameters:
        start (tuple): start point coordinate (x, y, z)
        goal (tuple): goal point coordinate (x, y, z)
        env (Grid): 3D environment
        heuristic_type (str): heuristic function type ("euclidean", "manhattan", etc.)

    Examples:
        >>> import python_motion_planning as pmp
        >>> planner = pmp.AStar((5, 5, 2), (45, 25, 5), pmp.Grid(51, 31, 11))
        >>> cost, path, expand = planner.plan()     # planning results only
        >>> planner.plot.animation(path, str(planner), cost, expand)  # animation
        >>> planner.run()       # run both planning and animation

    References:
        [1] Hart, Nilsson, Raphael (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)

    def __str__(self) -> str:
        return "A* (3D)"

    def plan(self) -> tuple:
        """
        A* motion plan function.

        Returns:
            cost (float): path cost
            path (list): planning path
            expand (list): all nodes that planner has searched
        """
        # OPEN list (priority queue) and CLOSED list (hash table)
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        while OPEN:
            node = heapq.heappop(OPEN)

            # exists in CLOSED list
            if node.current in CLOSED:
                continue

            # goal found
            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for node_n in self.getNeighbor(node):                
                # exists in CLOSED list
                if node_n.current in CLOSED:
                    continue

                node_n.parent = node.current
                node_n.h = self.h(node_n, self.goal)

                # goal found
                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break

                # update OPEN list
                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node
        return [], [], []

    def getNeighbor(self, node: Node) -> list:
        """
        Find neighbors of node in 3D, constrained to valid grid bounds (y >= 0).
        """
        neighbors = []
        for motion in self.motions:
            neighbor = node + motion
            x, y, z = neighbor.current

            # Only allow nodes within the grid and y >= 0
            if y < 0 or x < 0 or z < 0:
                continue
            if x >= self.env.x_range or y >= self.env.y_range or z >= self.env.z_range:
                continue
            if self.isCollision(node, neighbor):
                continue

            neighbors.append(neighbor)
        return neighbors



    def extractPath(self, closed_list: dict) -> tuple:
        """
        Extract the path based on the CLOSED list.

        Parameters:
            closed_list (dict): CLOSED list

        Returns:
            cost (float): the cost of planned path
            path (list): the planning path
        """
        cost = 0
        node = closed_list[self.goal.current]
        path = [node.current]
        while node != self.start:
            node_parent = closed_list[node.parent]
            cost += self.dist(node, node_parent)
            node = node_parent
            path.append(node.current)
        return cost, path

    def run(self):
        """
        Running both planning and animation.
        """
        cost, path, expand = self.plan()
        self.plot.animation(path, str(self), cost, expand)
