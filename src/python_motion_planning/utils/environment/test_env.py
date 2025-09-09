import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from python_motion_planning.utils.environment.env import *

def plot_grid(grid: Grid):
    """
    Visualize the 3D grid obstacles.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    obs = np.array(list(grid.obstacles))
    if obs.size > 0:
        ax.scatter(obs[:,0], obs[:,1], obs[:,2], c='red', marker='s', s=20, label='Obstacles')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, grid.x_range)
    ax.set_ylim(0, grid.y_range)
    ax.set_zlim(0, grid.z_range)
    ax.set_title("3D Grid Map")
    plt.legend()
    plt.show()


def plot_map(map_obj: Map):
    """
    Visualize the 3D continuous map with rectangular and spherical obstacles.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot rectangular/prism obstacles
    for rect in map_obj.obs_rect:
        x, y, z, dx, dy, dz = rect
        xx = [x, x+dx, x+dx, x, x, x+dx, x+dx, x]
        yy = [y, y, y+dy, y+dy, y, y, y+dy, y+dy]
        zz = [z, z, z, z, z+dz, z+dz, z+dz, z+dz]
        vertices = [[list(zip(xx, yy, zz))]]  # shape for Poly3DCollection
        ax.bar3d(x, y, z, dx, dy, dz, color='blue', alpha=0.5)

    # Plot spherical obstacles
    for sphere in map_obj.obs_circ:
        cx, cy, cz, r = sphere
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        xs = cx + r*np.cos(u)*np.sin(v)
        ys = cy + r*np.sin(u)*np.sin(v)
        zs = cz + r*np.cos(v)
        ax.plot_surface(xs, ys, zs, color='red', alpha=0.5)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, map_obj.x_range)
    ax.set_ylim(0, map_obj.y_range)
    ax.set_zlim(0, map_obj.z_range)
    ax.set_title("3D Continuous Map")
    plt.show()

grid = Grid(50, 50, 10)  # works fine

plot_grid(grid)

# Example for Map
map3d = Map(50, 50, 10)
plot_map(map3d)