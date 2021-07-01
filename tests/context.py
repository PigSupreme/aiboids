"""Context file for unit tests."""

import os
import sys

aiboids_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if sys.path[0] != aiboids_root:
    sys.path.insert(0, aiboids_root)

import aiboids
