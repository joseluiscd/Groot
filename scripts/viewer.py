#!/bin/env python3

import sys
if "--debug" in sys.argv:
    sys.path.append("build/groot_python")
else:
    sys.path.append("build_release/groot_python")

import pygroot as groot

reg = groot.Registry()
reg.run_viewer()