"""
@file: common_examples_3d_animated_city.py
@brief: 3D animated path visualization with skyscraper obstacles (drone delivery scenario)
@author: Adapted
@update: 2025.9.9
"""
import sys, os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning import *


def animate_path_3d(path, expand=None, start=None, goal=None, buildings=None, grid_env=None, title="3D Path Animation"):
    """
    Animate 3D path line from start to goal with buildings and visited nodes.
    """
    plt.close('all')
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)

    # Start and goal points
    if start:
        ax.scatter(*start, c="blue", s=100, marker="o", label="Start")
    if goal:
        ax.scatter(*goal, c="green", s=100, marker="X", label="Goal")

    # Draw buildings
    if buildings:
        for x, y, w, d, h in buildings:
            ax.bar3d(x, y, 0, w, d, h, color='black', alpha=0.3)

    # Visited nodes in light gray
    if expand:
        # Only plot every few nodes to reduce lag
        sample_step = max(len(expand)//500, 1)
        ex, ey, ez = zip(*[node.current for node in expand[::sample_step]])
        ax.scatter(ex, ey, ez, c="lightgray", alpha=0.2, s=15, marker="o", label="Visited")

    # Path line
    line, = ax.plot([], [], [], c="red", linewidth=2, label="Path")

    # Explicitly set axis limits to match grid
    if grid_env:
        ax.set_xlim(0, grid_env.x_range)
        ax.set_ylim(0, grid_env.y_range)   # Y-axis starts at 0
        ax.set_zlim(0, grid_env.z_range)

    # Labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    # Animation update
    frame_step = max(len(path)//200, 1)  # reduces number of frames for long paths
    def update(num):
        num = num * frame_step
        if num < len(path):
            px, py, pz = zip(*path[:num+1])
            line.set_data(px, py)
            line.set_3d_properties(pz)
        return line,

    ani = FuncAnimation(fig, update, frames=len(path)//frame_step, interval=50, blit=True, repeat=False)
    plt.show()


def generate_city(grid_env, num_buildings=10, max_height=15):
    """
    Generate skyscraper obstacles randomly on the grid.
    Returns a list of tuples: (x_start, y_start, width, depth, height)
    """
    buildings = []
    wall_coords = []

    np.random.seed(42)  # For reproducibility
    for _ in range(num_buildings):
        x_start = np.random.randint(5, grid_env.x_range - 10)
        y_start = np.random.randint(5, grid_env.y_range - 10)
        width = np.random.randint(3, 7)
        depth = np.random.randint(3, 7)
        height = np.random.randint(5, max_height)
        buildings.append((x_start, y_start, width, depth, height))

        # Generate all coordinates for collision checking
        for x in range(x_start, min(x_start + width, grid_env.x_range)):
            for y in range(y_start, min(y_start + depth, grid_env.y_range)):
                for z in range(height):
                    wall_coords.append((x, y, z))
    
    return buildings, wall_coords


def main():
    # ---------- 3D Grid Environment ----------
    grid_env = Grid(51, 31, 21)

    # ---------- Generate city ----------
    buildings, wall_coords = generate_city(grid_env, num_buildings=15, max_height=18)
    grid_env.update(obstacles=set(wall_coords))

    # ---------- Start and Goal ----------
    start = (5, 5, 1)       # low altitude
    goal = (45, 25, 10)     # higher altitude

    # ---------- Planner ----------
    # Switch algorithms easily by commenting/uncommenting
    planner = AStar(start=start, goal=goal, env=grid_env)
    # planner = Dijkstra(start=start, goal=goal, env=grid_env)

    # ---------- Plan ----------
    cost, path, expand = planner.plan()
    print(f"Path cost: {cost}")
    print(f"Path length: {len(path)}")

    # Reverse path if needed
    path = path[::-1]

    # ---------- Animate ----------
    animate_path_3d(path, expand=expand, start=start, goal=goal,
                    buildings=buildings, grid_env=grid_env, title=str(planner))


if __name__ == '__main__':
    main()
