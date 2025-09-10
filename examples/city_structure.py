"""
@file: common_examples_3d_animated_city.py
@brief: 3D animated path visualization with deterministic skyscraper obstacles (drone delivery scenario) + metrics collection
@author: Adapted
@update: 2025.9.10
"""
import sys, os, time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

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
        sample_step = max(len(expand)//500, 1)  # sample to reduce lag
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
    frame_step = max(len(path)//200, 1)
    def update(num):
        num = num * frame_step
        if num < len(path):
            px, py, pz = zip(*path[:num+1])
            line.set_data(px, py)
            line.set_3d_properties(pz)
        return line,

    ani = FuncAnimation(fig, update, frames=len(path)//frame_step, interval=50, blit=True, repeat=False)
    plt.show()


def generate_city(grid_env, num_blocks_x=4, num_blocks_y=4, max_height=18, street_width=3):
    """
    Generate skyscrapers in a grid layout with equal spacing (streets) between them.
    Deterministic sizes, no overlap.
    """
    buildings = []
    wall_coords = []

    block_size_x = (grid_env.x_range - (num_blocks_x + 1) * street_width) // num_blocks_x
    block_size_y = (grid_env.y_range - (num_blocks_y + 1) * street_width) // num_blocks_y

    for bx in range(num_blocks_x):
        for by in range(num_blocks_y):
            x_start = street_width + bx * (block_size_x + street_width)
            y_start = street_width + by * (block_size_y + street_width)

            width = block_size_x - 1 - ((bx + by) % 2)
            depth = block_size_y - 1 - ((bx * 2 + by) % 2)
            height = 5 + ((bx * 3 + by * 5) % max_height)

            width = max(3, width)
            depth = max(3, depth)

            buildings.append((x_start, y_start, width, depth, height))

            for xi in range(x_start, min(x_start + width, grid_env.x_range)):
                for yi in range(y_start, min(y_start + depth, grid_env.y_range)):
                    for zi in range(height):
                        wall_coords.append((xi, yi, zi))

    return buildings, wall_coords


def main():
    # ---------- 3D Grid Environment ----------
    grid_env = Grid(50, 50, 20)

    # ---------- Generate city ----------
    buildings, wall_coords = generate_city(grid_env, num_blocks_x=3, num_blocks_y=4, max_height=16)
    grid_env.update(obstacles=set(wall_coords))

    # ---------- Start and Goal ----------
    start = (10, 5, 5)
    goal = (45, 40, 10)

    # ---------- Planner ----------
    # planner = AStar(start=start, goal=goal, env=grid_env)
    planner = Dijkstra(start=start, goal=goal, env=grid_env)
    # planner = ThetaStar(start=start, goal=goal, env=grid_env)

    # ---------- Plan + Metrics ----------
    t0 = time.time()
    cost, path, expand = planner.plan()
    t1 = time.time()
    exec_time = t1 - t0

    visited_nodes = len(expand)
    path_length = len(path)

    print("===== Metrics =====")
    print(f"Execution time: {exec_time:.6f} s")
    print(f"Visited nodes: {visited_nodes}")
    print(f"Path length (cells): {path_length}")
    print(f"Path cost: {cost}")

    # Reverse path if needed
    path = path[::-1]

    # ---------- Animate ----------
    animate_path_3d(path, expand=expand, start=start, goal=goal,
                    buildings=buildings, grid_env=grid_env, title=str(planner))


if __name__ == '__main__':
    main()
