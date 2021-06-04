#!/bin/env python3

import sys
if "--debug" in sys.argv:
    sys.path.append("build/app")
    sys.argv.remove("--debug")
else:
    sys.path.append("build_release/app")

import groot

reg = groot.Registry()
reg.run_viewer()