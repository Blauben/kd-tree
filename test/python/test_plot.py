import os
import numpy as np
from PIL import Image

def test_plot_no_throw(tmp_path):
    import scikdtree as sci
    mesh_path = "resources/Eros_scaled-1000"
    tree = sci.KDTree(mesh_path + ".node", mesh_path + ".face")
    outpath = os.path.join(tmp_path, "kd_tree_plot_test.png")
    sci.plot_kd_tree(tree, outpath=outpath, show_gui=False)
    assert os.path.exists(outpath), "Plot file was not created as expected."

    generated = np.array(Image.open(outpath))
    reference = np.array(Image.open("resources/reference_kd_tree_plot.png"))

    assert generated.shape == reference.shape
    assert np.mean(np.abs(generated.astype(np.int16) - reference.astype(np.int16))) < 2
