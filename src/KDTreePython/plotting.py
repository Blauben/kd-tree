import numpy as np
import sys

import matplotlib.pyplot as plt
from tqdm import tqdm

from .scikdtree import KDTree, Direction


def plot_kd_tree(kdtree: KDTree, title: str = "KDTree", outpath: str = None, show_gui: bool = False, print_to_stdout: bool = True) -> plt.Figure:
    """Plots a 3d KD-Tree using matplotlib

        :param show_gui: Shows the resulting plot in a gui. Attention: this blocks the program until the plot window is closed.
        :param print_to_stdout: If True, prints the progress of plotting to stdout. Otherwise compute silent. Useful for keeping logs clean.
        :param title: Sets the title of the plot
        :param outpath: Where to save the resulting plot. If None, the plot will not be saved.
        :param kdtree: The KDTree to plot
        :type kdtree: KDTree
        :return The resulting matplotlib figure
        :rtype: plt.Figure
    """
    fig = plt.figure(dpi=800)
    ax = fig.add_subplot(111, projection='3d')
    tqdm_kwargs = {"disable": not print_to_stdout}
    if print_to_stdout:
        tqdm_kwargs["file"] = sys.stdout

    for vertex in tqdm([v for geom in kdtree.geometry() for v in geom.vertices()], desc="Plotting vertices", **tqdm_kwargs):
        ax.scatter(*vertex, color=[0, 0, 1, 0.3], s=1, marker='.')

    pbar = tqdm(desc="Building and plotting planes", **tqdm_kwargs)
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
        pbar.update(1)
    pbar.close()

    if print_to_stdout:
        print("Plot complete\n")

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    fig.suptitle(title)
    if outpath is not None:
        fig.savefig(outpath)
    if show_gui:
        plt.show()
    return fig