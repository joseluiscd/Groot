#!/bin/env python3

import sys
if "--debug" in sys.argv:
    sys.path.append("build/app")
else:
    sys.path.append("build_release/app")

import groot
import toml

registry = groot.Registry()
params = toml.load(sys.argv[1])

print("Reading from", params["input"])


entity = registry.load_ply(params["input"])

entity.graph_from_cloud_knn(params["cluster"]["k"])
entity.graph_cluster(params["cluster"]["intervals"])

print("Done!")
print("Writing output file")
registry.save(params["output"])
print("Finished!")