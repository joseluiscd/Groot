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

def timeit(func):
    async def process(func, *args, **params):
        return await func(*args, **params)

    async def helper(*args, **params):
        start = time.time()
        result = await process(func, *args, **params)

        return time.time() - start

    return helper

@timeit
async def reconstruct_knn(entity):
    entity.graph_from_cloud_knn(10)
    entity.graph_cluster(30)


@timeit
async def reconstruct_radius(entity):
    entity.remove_component(groot.components.PointNormals)

    await entity.graph_from_cloud_radius(0.15)
    await entity.graph_cluster(30)

@timeit
async def run_cardenas_et_al(entity):
    entity.remove_component(groot.components.PointNormals)

    await groot.compute_cardenas_et_al(ply, 0.15)
    await ply.graph_cluster(30)

reconstruct = reconstruct_knn

async def resample(entity):
    sampled = await entity.graph_resample(0.03)
    sampled.move_component(entity, groot.components.PlantGraph)
    sampled.destroy()

async def evaluate(ground_truth, reconstructed):
    await asyncio.gather(
        resample(ground_truth),
        resample(reconstructed)
    )

    diff = await groot.compute_difference(ground_truth, reconstructed)
    res = await groot.evaluate_difference_mse(diff)
    diff.destroy()

    return res
    

async def init():
    global registry

    FILES = list(map(lambda f: str(f).zfill(2), range(1, 61)))
    print(FILES)

    results = dict()

    for f in FILES:

        a = registry.load_graph(os.path.join(sys.argv[1], f + ".obj"))
        b = registry.load_ply(os.path.join(sys.argv[1], f + ".ply"))

        print(a, b)

        (ground_truth, reconstructed) = await asyncio.gather(a, b)

        time = await reconstruct(reconstructed)
        score = await evaluate(ground_truth, reconstructed)

        results[f] = {
            "time": time,
            "score": score
        }

    toml.dump(results, open("result.toml", "w"))



if __name__ == "__main__":
    asyncio.run(init())
