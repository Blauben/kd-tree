import os
import numpy as np
from PIL import Image

def test_plot_no_throw():
    import scikdtree as sci
    mesh_path = "resources/Eros_scaled-1000"
    tree = sci.KDTree(mesh_path + ".node", mesh_path + ".face")
    sci.plot_kd_tree(tree, outpath="kd_tree_plot_test.png")
    assert os.path.exists("kd_tree_plot_test.png"), "Plot file was not created as expected."

    generated = np.array(Image.open("kd_tree_plot_test.png"))
    reference = np.array(Image.open("resources/reference_kd_tree_plot.png"))

    assert generated.shape == reference.shape
    assert np.mean(np.abs(generated.astype(np.int16) - reference.astype(np.int16))) < 2
