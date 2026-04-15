import os

try:
    from .scikdtree import *
    from .plotting import *
    del scikdtree
except Exception as e:
    pkg_dir = os.path.dirname(__file__)
    print(f"Package directory: {pkg_dir}")
    print(f"Contents: {os.listdir(pkg_dir)}")
    raise ImportError("Failed to import from scikdtree. Check the contents of the package directory. Original error: " + str(e))
