import matplotlib.pyplot as plt
from .scikdtree import KDTree, Direction
import numpy as np


def plot_kd_tree(kdtree: KDTree, title: str = "KDTree", outpath: str = None, show_gui: bool = False):
    """Plots an 3d KD-Tree using matplotlib

    :param kdtree: The KDTree to plot
    :type kdtree: KDTree

    :return The resulting figure for the user to adjust.
    :rtype Figure
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for vertex in kdtree.geometry():
        ax.scatter(vertex[0], vertex[1], vertex[2], color='b', s=1)
    for plane, box in kdtree.planes():
        orientation_index = list(Direction).index(plane.orientation)
        min_corner = np.array(box.minPoint, dtype=float)
        max_corner = np.array(box.maxPoint, dtype=float)

        if orientation_index == 0:  # x = constant
            y = np.array([min_corner[1], max_corner[1]], dtype=float)
            z = np.array([min_corner[2], max_corner[2]], dtype=float)
            y_grid, z_grid = np.meshgrid(y, z)
            x_grid = np.full_like(y_grid, plane.axisCoordinate, dtype=float)
        elif orientation_index == 1:  # y = constant
            x = np.array([min_corner[0], max_corner[0]], dtype=float)
            z = np.array([min_corner[2], max_corner[2]], dtype=float)
            x_grid, z_grid = np.meshgrid(x, z)
            y_grid = np.full_like(x_grid, plane.axisCoordinate, dtype=float)
        else:  # z = constant
            x = np.array([min_corner[0], max_corner[0]], dtype=float)
            y = np.array([min_corner[1], max_corner[1]], dtype=float)
            x_grid, y_grid = np.meshgrid(x, y)
            z_grid = np.full_like(x_grid, plane.axisCoordinate, dtype=float)

        ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.2)
        
    fig.suptitle(title)
    if outpath is not None:
        fig.savefig(outpath)
    if show_gui:
        plt.show()
    return fig