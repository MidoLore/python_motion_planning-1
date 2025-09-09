# test_pose3d.py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from python_motion_planning.utils.environment.pose3d import Pose3D


def rotation_matrix(roll, pitch, yaw):
    """Return a 3x3 rotation matrix from roll, pitch, yaw."""
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx


def test_pose3d_operations():
    # Create some poses
    p1 = Pose3D(1, 2, 3)
    p2 = Pose3D(4, 5, 6, 0.2, 0.3, 0.4)
    p3 = Pose3D(1, 2, 3)

    # Arithmetic
    print("p1 + p2 =", p1 + p2)
    print("p2 - p1 =", p2 - p1)

    # Equality
    print("p1 == p3:", p1 == p3)
    print("p1 != p2:", p1 != p2)

    # Tuple conversion
    print("Tuple p1:", p1.to_tuple)
    print("From tuple:", Pose3D.from_tuple(p1.to_tuple))

    # Visualization
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Pose3D Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    poses = [p1, p2]
    colors = ["blue", "green"]

    for pose, color in zip(poses, colors):
        # Position
        ax.scatter(pose.x, pose.y, pose.z, color=color, s=50)

        # Orientation (draw yaw axis as arrow)
        R = rotation_matrix(pose.roll, pose.pitch, pose.yaw)
        direction = R @ np.array([1, 0, 0])  # X-axis direction
        ax.quiver(
            pose.x, pose.y, pose.z,
            direction[0], direction[1], direction[2],
            length=1.0, color=color, alpha=0.6
        )

        ax.text(pose.x, pose.y, pose.z, f"{pose}", fontsize=8)

    ax.set_xlim(0, 7)
    ax.set_ylim(0, 7)
    ax.set_zlim(0, 7)

    plt.show()


if __name__ == "__main__":
    test_pose3d_operations()
