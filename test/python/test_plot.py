import os
import hashlib

def test_plot_no_throw():
    import scikdtree as sci
    mesh_path = "resources/Eros_scaled-1000"
    tree = sci.KDTree(mesh_path + ".node", mesh_path + ".face")
    sci.plot_kd_tree(tree, outpath="kd_tree_plot_test.png")
    assert os.path.exists("kd_tree_plot_test.png"), "Plot file was not created as expected."
    with open("kd_tree_plot_test.png", "rb") as image:
        file_hash = hashlib.sha256(image.read()).hexdigest()
    expected_hash = "64b68383dbcc4035f9d62050b90c68bf26f855e690c81c5dc1b2caf11c441875"
    assert file_hash == expected_hash
