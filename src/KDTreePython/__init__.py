try:
    from .scikdtree import *

    del scikdtree
except Exception as e:
    import os
    import sys
    pkg_dir = os.path.dirname(__file__)
    print(f"Package directory: {pkg_dir}")
    print(f"Contents: {os.listdir(pkg_dir)}")
    raise ImportError("Failed to import from scikdtree. Check the contents of the package directory. Original error: " + str(e))
