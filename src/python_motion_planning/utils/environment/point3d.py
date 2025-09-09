"""
@file: point3d.py
@breif: 3-dimension point data structure
@author: Wu Maojia
@update: 2025.9.9
"""
import math

class Point3D(object):
    """
    Class for searching and manipulating 3-dimensional points.

    Parameters:
        x: x-coordinate of the 3D point
        y: y-coordinate of the 3D point
        z: z-coordinate of the 3D point
        eps: tolerance for float comparison

    Examples:
        >>> p1 = Point3D(1, 2, 3)
        >>> p2 = Point3D(3, 4, 5)
        >>> p1 + p2
        >>> Point3D(4, 6, 8)
        >>> p1.dist(p2)
        >>> 3.4641016151377544
    """
    def __init__(self, x: float, y: float, z: float = 0.0, eps: float = 1e-6) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.eps = eps

        if abs(self.x - round(self.x)) < self.eps:
            self.x = round(self.x)
        if abs(self.y - round(self.y)) < self.eps:
            self.y = round(self.y)
        if abs(self.z - round(self.z)) < self.eps:
            self.z = round(self.z)

    def __add__(self, point):
        assert isinstance(point, Point3D)
        return Point3D(self.x + point.x, self.y + point.y, self.z + point.z)

    def __sub__(self, point):
        assert isinstance(point, Point3D)
        return Point3D(self.x - point.x, self.y - point.y, self.z - point.z)

    def __eq__(self, point) -> bool:
        if not isinstance(point, Point3D):
            return False
        return (abs(self.x - point.x) < self.eps and
                abs(self.y - point.y) < self.eps and
                abs(self.z - point.z) < self.eps)

    def __ne__(self, point) -> bool:
        return not self.__eq__(point)

    def __hash__(self) -> int:
        return hash((self.x, self.y, self.z))

    def __str__(self) -> str:
        return f"Point3D({self.x}, {self.y}, {self.z})"

    def __repr__(self) -> str:
        return self.__str__()

    @staticmethod
    def from_tuple(point: tuple):
        if len(point) == 2:
            return Point3D(point[0], point[1], 0.0)
        return Point3D(point[0], point[1], point[2])

    @property
    def to_tuple(self) -> tuple:
        return int(self.x), int(self.y), int(self.z)

    def dist(self, point) -> float:
        assert isinstance(point, Point3D)
        dx = self.x - point.x
        dy = self.y - point.y
        dz = self.z - point.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def angle_xy(self, point) -> float:
        """Angle in XY plane relative to this point."""
        assert isinstance(point, Point3D)
        return math.atan2(point.y - self.y, point.x - self.x)

    def angle_xz(self, point) -> float:
        """Angle in XZ plane relative to this point."""
        assert isinstance(point, Point3D)
        return math.atan2(point.z - self.z, point.x - self.x)

    def angle_yz(self, point) -> float:
        """Angle in YZ plane relative to this point."""
        assert isinstance(point, Point3D)
        return math.atan2(point.z - self.z, point.y - self.y)
