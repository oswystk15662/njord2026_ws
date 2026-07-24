"""Make the source package importable in direct repository pytest runs."""

import os
import sys


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)
