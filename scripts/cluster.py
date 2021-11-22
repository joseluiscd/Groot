#!/bin/env python3

import sys
if "--debug" in sys.argv:
    sys.path.append("build/app")
else:
    sys.path.append("build_release/app")

import groot
import toml

registry = groot.Registry()
entity = registry.load_ply("/home/joseluis/Trees/palo.ply")
entity.visible = True

entity.graph_from_cloud_knn(10)
entity.graph_cluster(10)

resampled = entity.graph_resample(0.3)
resampled.visible = True

registry.run_viewer()