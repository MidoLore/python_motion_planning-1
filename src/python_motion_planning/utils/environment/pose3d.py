"""
@file: pose3d.py
@breif: 3-dimension pose data structure
@author: Adapted
@update: 2025.3.9
"""
import math


class Pose3D(object):
    """
    Class for searching and manipulating 3-dimensional poses.

    Parameters:
        x: x-coordinate of the 3D pose
        y: y-coordinate of the 3D pose
        z: z-coordinate of the 3D pose
        roll: rotation around X axis in radians
        pitch: rotation around Y axis in radians
        yaw: rotation around Z axis in radians
        eps: tolerance for float comparison

    Examples:
        >>> from python_motion_planning.utils.environment.pose3d import Pose3D
        >>> p1 = Pose3D(1, 2, 3)
        >>> p2 = Pose3D(3, 4, 5, 0.1, 0.2, 0.3)
        >>> p1
        Pose3D(1, 2, 3, 0, 0, 0)
        >>> p2
        Pose3D(3, 4, 5, 0.1, 0.2, 0.3)
        >>> p1 + p2
        Pose3D(4, 6, 8, 0.1, 0.2, 0.3)
        >>> p1 - p2
        Pose3D(-2, -2, -2, -0.1, -0.2, -0.3)
        >>> p1 == Pose3D(1, 2, 3)
        True
    """

    def __init__(self, x: float, y: float, z: float,
                 roll: float = 0, pitch: float = 0, yaw: float = 0,
                 eps: float = 1e-6) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.eps = eps

        # Snap to nearest int if within epsilon
        for attr in ["x", "y", "z", "roll", "pitch", "yaw"]:
            val = getattr(self, attr)
            if abs(val - round(val)) < self.eps:
                setattr(self, attr, round(val))

    def __add__(self, pose):
        assert isinstance(pose, Pose3D)
        return Pose3D(
            self.x + pose.x,
            self.y + pose.y,
            self.z + pose.z,
            self.roll + pose.roll,
            self.pitch + pose.pitch,
            self.yaw + pose.yaw,
        )

    def __sub__(self, pose):
        assert isinstance(pose, Pose3D)
        return Pose3D(
            self.x - pose.x,
            self.y - pose.y,
            self.z - pose.z,
            self.roll - pose.roll,
            self.pitch - pose.pitch,
            self.yaw - pose.yaw,
        )

    def __eq__(self, pose) -> bool:
        if not isinstance(pose, Pose3D):
            return False
        return (abs(self.x - pose.x) < self.eps and
                abs(self.y - pose.y) < self.eps and
                abs(self.z - pose.z) < self.eps and
                abs(self.roll - pose.roll) < self.eps and
                abs(self.pitch - pose.pitch) < self.eps and
                abs(self.yaw - pose.yaw) < self.eps)

    def __ne__(self, pose) -> bool:
        return not self.__eq__(pose)

    def __hash__(self) -> int:
        return hash((self.x, self.y, self.z, self.roll, self.pitch, self.yaw))

    def __str__(self) -> str:
        return f"Pose3D({self.x}, {self.y}, {self.z}, {self.roll}, {self.pitch}, {self.yaw})"

    def __repr__(self) -> str:
        return self.__str__()

    @staticmethod
    def from_tuple(pose: tuple):
        return Pose3D(*pose)

    @property
    def to_tuple(self) -> tuple:
        return self.x, self.y, self.z, self.roll, self.pitch, self.yaw
