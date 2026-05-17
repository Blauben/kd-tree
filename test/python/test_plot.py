import os
import sys

import PIL
import numpy as np
from PIL import Image

def test_plot_no_throw(tmp_path):
    import scikdtree as sci
    mesh_path = "resources/Eros_scaled-1000"
    tree = sci.KDTree(mesh_path + ".node", mesh_path + ".face")
    outpath = os.path.join(tmp_path, "kd_tree_plot_test.png")
    sci.plot_kd_tree(tree, outpath=outpath)
    assert os.path.exists(outpath), "Plot file was not created as expected."

    reference_path = "resources/reference_kd_tree_plot.PNG"
    generated = np.array(Image.open(outpath))
    print(f"test_plot_no_throw: Test plot written to {outpath}")
    print(f"Reading reference image from {reference_path}")
    try:
        reference = np.array(Image.open(reference_path))
    except PIL.UnidentifiedImageError as e:
        print(f"Error reading reference image: {e}\nCurrent working directory: {os.getcwd()}", file=sys.stderr)
        if os.path.exists(reference_path):
            print(f"Reference image exists at {reference_path}, but could not be opened.", file=sys.stderr)
        else:
            print(f"Reference image does not exist at {reference_path}.", file=sys.stderr)
        raise e

    assert generated.shape == reference.shape
    assert np.mean(np.abs(generated.astype(np.int16) - reference.astype(np.int16))) < 2
