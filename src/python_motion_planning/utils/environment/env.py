"""
@file: env.py
@breif: 2-dimension environment
@author: Winter
@update: 2023.1.13
"""
from math import sqrt
from abc import ABC, abstractmethod
from scipy.spatial import cKDTree
import numpy as np

from .node import Node  # keep your Node class

class Env(ABC):
    """
    Class for building a workspace of robots in 2D or 3D.
    """
    def __init__(self, x_range: int, y_range: int, z_range: int = None, eps: float = 1e-6) -> None:
        """
        x_range, y_range, z_range: dimensions of the environment
        If z_range is None, environment is 2D.
        """
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.eps = eps

    @property
    def grid_map(self) -> set:
        """
        Returns all coordinates in the environment as tuples.
        """
        if self.z_range is None:
            return {(i, j) for i in range(self.x_range) for j in range(self.y_range)}
        else:
            return {(i, j, k) for i in range(self.x_range)
                              for j in range(self.y_range)
                              for k in range(self.z_range)}

    @abstractmethod
    def init(self) -> None:
        pass


class Grid(Env):
    """
    Class for discrete 3D grid map.
    """
    def __init__(self, x_range: int, y_range: int, z_range: int) -> None:
        super().__init__(x_range, y_range, z_range)

        # 26-connected 3D motions (neighbors in 3D grid)
        self.motions = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == dy == dz == 0:
                        continue
                    cost = sqrt(dx**2 + dy**2 + dz**2)
                    self.motions.append(Node((dx, dy, dz), None, cost, None))

        self.obstacles = None
        self.obstacles_tree = None
        self.init()

    def init(self) -> None:
        """
        Initialize 3D grid map with boundaries and some obstacles.
        """
        x, y, z = self.x_range, self.y_range, self.z_range
        obstacles = set()

        # boundaries (all outer surfaces)
        for i in range(x):
            for j in range(y):
                obstacles.add((i, j, 0))
                obstacles.add((i, j, z-1))
        for i in range(x):
            for k in range(z):
                obstacles.add((i, 0, k))
                obstacles.add((i, y-1, k))
        for j in range(y):
            for k in range(z):
                obstacles.add((0, j, k))
                obstacles.add((x-1, j, k))

        # example internal obstacles (can customize)
        for i in range(5, 10):
            for j in range(5, 10):
                for k in range(2, 4):
                    obstacles.add((i, j, k))

        self.obstacles = obstacles
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))

    def update(self, obstacles):
        """
        Update obstacles in the 3D grid.
        """
        self.obstacles = obstacles
        self.obstacles_tree = cKDTree(np.array(list(obstacles)))

class Map(Env):
    """
    Class for continuous 3D map.
    """
    def __init__(self, x_range: int, y_range: int, z_range: int) -> None:
        super().__init__(x_range, y_range, z_range)
        self.boundary = None
        self.obs_circ = None  # spherical obstacles
        self.obs_rect = None  # rectangular/prism obstacles
        self.init()

    def init(self):
        """
        Initialize 3D map.
        """
        x, y, z = self.x_range, self.y_range, self.z_range

        # boundaries: 6 faces of the 3D box (xmin, ymin, zmin, dx, dy, dz)
        self.boundary = [
            [0, 0, 0, x, 1, 1],        # xmin face
            [0, 0, 0, 1, y, 1],        # ymin face
            [0, 0, 0, 1, 1, z],        # zmin face
            [x-1, 0, 0, 1, y, z],      # xmax face
            [0, y-1, 0, x, 1, z],      # ymax face
            [0, 0, z-1, x, y, 1]       # zmax face
        ]

        # user-defined rectangular/prism obstacles: [x, y, z, dx, dy, dz]
        self.obs_rect = [
            [14, 12, 5, 8, 2, 3],
            [18, 22, 2, 8, 3, 4],
            [26, 7, 3, 2, 12, 5],
            [32, 14, 6, 10, 2, 3]
        ]

        # user-defined spherical obstacles: [x, y, z, radius]
        self.obs_circ = [
            [7, 12, 5, 3],
            [46, 20, 6, 2],
            [15, 5, 4, 2],
            [37, 7, 5, 3],
            [37, 23, 6, 3]
        ]

    def update(self, boundary=None, obs_circ=None, obs_rect=None):
        self.boundary = boundary if boundary else self.boundary
        self.obs_circ = obs_circ if obs_circ else self.obs_circ
        self.obs_rect = obs_rect if obs_rect else self.obs_rect
