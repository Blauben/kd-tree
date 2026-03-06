try:
    from .kdtree_py import *

    del kdtree_py
except Exception as e:
    print(f"Import error: {e}")
    # This will fail because kdtree_py isn't imported yet
    # print(dir(kdtree_py))

    # Better: Try to find what's actually in the package
    import os
    import sys
    pkg_dir = os.path.dirname(__file__)
    print(f"Package directory: {pkg_dir}")
    print(f"Contents: {os.listdir(pkg_dir)}")
