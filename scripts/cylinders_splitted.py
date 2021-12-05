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

print("Computing normals")
entity.compute_normals(**params["normals"])

for entity in entity.split_cloud(params["cylinders"]["voxel_size"]):
    entity.cylinder_marching(**params["cylinders"])
    print("Cylinder count:", len(entity.cylinders()))
    entity.cylinder_filter(**params["cylinder_filter"])
    print("Cylinder count after filter:", len(entity.cylinders()))

print("Done!")
print("Writing output file")
registry.save(params["output"])
print("Finished!")