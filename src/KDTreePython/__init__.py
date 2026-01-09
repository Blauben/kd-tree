try:
    from .KDTree_Python import *

    del KDTree_Python
except Exception as e:
    print(f"Import error: {e}")
    # This will fail because KDTree_Python isn't imported yet
    # print(dir(KDTree_Python))

    # Better: Try to find what's actually in the package
    import os
    import sys
    pkg_dir = os.path.dirname(__file__)
    print(f"Package directory: {pkg_dir}")
    print(f"Contents: {os.listdir(pkg_dir)}")
