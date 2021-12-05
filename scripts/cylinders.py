#!/bin/env python3

import sys
if "--debug" in sys.argv:
    sys.path.append("build/groot_python")
else:
    sys.path.append("build_release/groot_python")

import pygroot as groot
import toml

registry = groot.Registry()
params = toml.load(sys.argv[1])

print("Reading from", params["input"])


entity = registry.load_ply(params["input"])
print("Loaded PLY")

if "normals" in params:
    print("Computing normals")
    entity.compute_normals(**params["normals"])

print("Marching cylinders")
entity.cylinder_marching(**params["cylinders"])
print("Cylinder count:", len(entity.cylinders()))
print("Filtering cylinders")
entity.cylinder_filter(**params["cylinder_filter"])
print("Cylinder count:", len(entity.cylinders()))

if params["filter_cloud_with_cylinders"]: 
    print("Rebuilding cloud from found cylinders...")
    entity.rebuild_cloud_from_cylinders()

#entity.build_graph_from_cylinders()

print("Done!")
print("Writing output file")
registry.save(params["output"])
print("Finished!")