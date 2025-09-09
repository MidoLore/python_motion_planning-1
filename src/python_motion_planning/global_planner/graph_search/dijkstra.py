"""
@file: dijkstra.py
@brief: Dijkstra 3D motion planning
@author: Yang Haodong, Wu Maojia
@update: 2025.9.9
"""
import heapq

from .a_star import AStar
from python_motion_planning.utils import Env, Grid, Node


class Dijkstra(AStar):
    """
    Class for Dijkstra motion planning in 3D.

    Parameters:
        start (tuple): start point coordinate (x, y, z)
        goal (tuple): goal point coordinate (x, y, z)
        env (Grid): 3D grid environment
    """
    def __init__(self, start: tuple, goal: tuple, env: Grid) -> None:
        super().__init__(start, goal, env)
    
    def __str__(self) -> str:
        return "Dijkstra"

    def plan(self) -> tuple:
        """
        Dijkstra 3D motion plan function.

        Returns:
            cost (float): path cost
            path (list): planned path
            expand (list): all nodes explored
        """
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = dict()

        while OPEN:
            node = heapq.heappop(OPEN)

            # already explored
            if node.current in CLOSED:
                continue

            # goal reached
            if node == self.goal:
                CLOSED[node.current] = node
                cost, path = self.extractPath(CLOSED)
                return cost, path, list(CLOSED.values())

            for node_n in self.getNeighbor(node):
                # skip obstacles
                if hasattr(self.env, "obstacles") and node_n.current in self.env.obstacles:
                    continue

                # already explored
                if node_n.current in CLOSED:
                    continue

                node_n.parent = node.current
                node_n.h = 0  # Dijkstra has no heuristic

                # goal found
                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break

                heapq.heappush(OPEN, node_n)

            CLOSED[node.current] = node

        return [], [], []
