import sys
sys.path.insert(0, r"C:\Users\moham\Documents\python_motion_planning\src")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from python_motion_planning.utils.environment.env import Grid, Map
from python_motion_planning.utils.environment.node import Node

def plot_grid_nodes(grid: Grid, nodes=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot grid obstacles (transparent red)
    obs = np.array(list(grid.obstacles))
    if obs.size > 0:
        ax.scatter(obs[:,0], obs[:,1], obs[:,2], c='red', marker='s', s=50, alpha=0.2, label='Grid obstacles')

    # Plot nodes if provided
    if nodes:
        colors = ['green', 'yellow', 'purple']
        for node, color in zip(nodes, colors):
            ax.scatter(node.x, node.y, node.z, c=color, s=100, label=f'Node {color}')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, grid.x_range)
    ax.set_ylim(0, grid.y_range)
    ax.set_zlim(0, grid.z_range)
    ax.set_title("3D Grid + Nodes")
    ax.legend()
    plt.show()

def plot_map_nodes(map_obj: Map, nodes=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Rectangular/prism obstacles (transparent blue)
    for rect in map_obj.obs_rect:
        x, y, z, dx, dy, dz = rect
        ax.bar3d(x, y, z, dx, dy, dz, color='blue', alpha=0.2)

    # Spherical obstacles (transparent red)
    for sphere in map_obj.obs_circ:
        cx, cy, cz, r = sphere
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        xs = cx + r*np.cos(u)*np.sin(v)
        ys = cy + r*np.sin(u)*np.sin(v)
        zs = cz + r*np.cos(v)
        ax.plot_surface(xs, ys, zs, color='red', alpha=0.2)

    # Nodes
    if nodes:
        colors = ['green', 'yellow', 'purple']
        for node, color in zip(nodes, colors):
            ax.scatter(node.x, node.y, node.z, c=color, s=100, label=f'Node {color}')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, map_obj.x_range)
    ax.set_ylim(0, map_obj.y_range)
    ax.set_zlim(0, map_obj.z_range)
    ax.set_title("3D Map + Nodes")
    ax.legend()
    plt.show()

if __name__ == "__main__":
    # --- Test Grid ---
    grid = Grid(20, 20, 10)
    start_node = Node((1, 1, 1))
    neighbor_node = Node((3, 3, 2), parent=start_node.current, g=1, h=2)
    summed_node = start_node + neighbor_node

    plot_grid_nodes(grid, nodes=[start_node, neighbor_node, summed_node])

    # --- Test Map ---
    map3d = Map(30, 30, 15)
    rect_node = Node((map3d.obs_rect[0][0], map3d.obs_rect[0][1], map3d.obs_rect[0][2]))
    sphere_node = Node((map3d.obs_circ[0][0], map3d.obs_circ[0][1], map3d.obs_circ[0][2]))

    plot_map_nodes(map3d, nodes=[rect_node, sphere_node, start_node])
