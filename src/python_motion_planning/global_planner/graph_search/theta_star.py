"""
@file: theta_star.py
@brief: Theta* motion planning in 3D
@author: Yang Haodong, Wu Maojia
@update: 2025.9.10 (extended to 3D)
"""
import heapq

from .a_star import AStar
from python_motion_planning.utils.environment.node import Node
from python_motion_planning.utils.environment.env import Grid


class ThetaStar(AStar):
    """
    Class for Theta* motion planning in 3D.

    Parameters:
        start (tuple): start point coordinate (x, y, z)
        goal (tuple): goal point coordinate (x, y, z)
        env (Grid): 3D environment
        heuristic_type (str): heuristic function type

    References:
        [1] Theta*: Any-Angle Path Planning on Grids
        [2] Any-angle path planning on non-uniform costmaps
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)

    def __str__(self) -> str:
        return "Theta* (3D)"

    def plan(self) -> tuple:
        """
        Theta* motion plan function.
        """
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        while OPEN:
            node = heapq.heappop(OPEN)

            if node.current in CLOSED:
                continue

            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for node_n in self.getNeighbor(node):
                if node_n.current in CLOSED:
                    continue

                node_n.parent = node.current
                node_n.h = self.h(node_n, self.goal)

                node_p = CLOSED.get(node.parent)
                if node_p:
                    self.updateVertex(node_p, node_n)

                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break

                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node
        return [], [], []

    def updateVertex(self, node_p: Node, node_c: Node) -> None:
        """
        Update extend node information with current node's parent node.
        """
        if self.lineOfSight(node_p, node_c):
            if node_p.g + self.dist(node_c, node_p) <= node_c.g:
                node_c.g = node_p.g + self.dist(node_c, node_p)
                node_c.parent = node_p.current

    def lineOfSight(self, node1: Node, node2: Node) -> bool:
        """
        Judge collision when moving from node1 to node2 in 3D using voxel traversal.

        Parameters:
            node1 (Node): start node
            node2 (Node): end node

        Returns:
            bool: True if line of sight exists (no collision), else False
        """
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return False

        x1, y1, z1 = node1.current
        x2, y2, z2 = node2.current

        # Bounds check
        for (x, y, z) in [node1.current, node2.current]:
            if not (0 <= x < self.env.x_range and 0 <= y < self.env.y_range and 0 <= z < self.env.z_range):
                return False

        # 3D DDA voxel traversal (Amanatides & Woo)
        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
        step_x = 1 if dx > 0 else -1 if dx < 0 else 0
        step_y = 1 if dy > 0 else -1 if dy < 0 else 0
        step_z = 1 if dz > 0 else -1 if dz < 0 else 0

        t_max_x = (0.5 if step_x > 0 else -0.5) / dx if dx != 0 else float("inf")
        t_max_y = (0.5 if step_y > 0 else -0.5) / dy if dy != 0 else float("inf")
        t_max_z = (0.5 if step_z > 0 else -0.5) / dz if dz != 0 else float("inf")

        t_delta_x = abs(1 / dx) if dx != 0 else float("inf")
        t_delta_y = abs(1 / dy) if dy != 0 else float("inf")
        t_delta_z = abs(1 / dz) if dz != 0 else float("inf")

        x, y, z = x1, y1, z1
        while (x, y, z) != (x2, y2, z2):
            if (x, y, z) in self.obstacles:
                return False

            if t_max_x < t_max_y and t_max_x < t_max_z:
                x += step_x
                t_max_x += t_delta_x
            elif t_max_y < t_max_z:
                y += step_y
                t_max_y += t_delta_y
            else:
                z += step_z
                t_max_z += t_delta_z

        return (x2, y2, z2) not in self.obstacles
