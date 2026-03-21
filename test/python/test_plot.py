def test_plot_no_throw():
    import scikdtree as sci
    mesh_path = "resources/Eros_scaled-1000"
    tree = sci.KDTree(mesh_path + ".node", mesh_path + ".face")
    sci.plot_kd_tree(tree, show_gui=True)