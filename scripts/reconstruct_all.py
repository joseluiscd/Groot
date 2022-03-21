#!/bin/env python3

import asyncio
import faulthandler
import os
import toml
import time
faulthandler.enable()

import sys
if "--debug" in sys.argv:
    sys.path.append("build")
    sys.argv.remove("--debug")
else:
    sys.path.append("build_release")

import pygroot as groot
import glob

registry = groot.Registry()
ImGui = groot.ImGui

def reconstruct_knn(entity):
    entity.graph_from_cloud_knn(10)
    entity.graph_cluster_interval_count(30)


async def reconstruct_radius(entity):
    entity.remove_component(groot.components.PointNormals)

    await entity.graph_from_cloud_radius(0.15)
    await entity.graph_cluster_interval_count(30)

async def reconstruct_cardenas_et_al(entity):
    await groot.compute_cardenas_et_al(entity, 0.15)
    entity.graph_cluster_interval_count(30)


async def reconstruct(entity):
    #await reconstruct_cardenas_et_al(entity)
    await reconstruct_radius(entity)

    new_entity = registry.new_entity()
    entity.move_component(new_entity, groot.components.PlantGraph)
    entity.destroy()

async def init():
    global registry

    FILES = list(map(lambda f: str(f).zfill(2), range(1, 61)))
    print(FILES)

    results = dict()

    for f in FILES:
        reconstructed = await registry.load_ply(os.path.join(sys.argv[1], f + ".ply"))
        await reconstruct(reconstructed)


    registry.save("Workspace.groot_workspace")



if __name__ == "__main__":
    asyncio.run(init())
