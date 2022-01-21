#!/bin/env python3

import time
import pdb
from functools import partial
from collections import defaultdict
import asyncio
import faulthandler
faulthandler.enable()

import threading

import sys
if "--debug" in sys.argv:
    sys.path.append("build")
else:
    sys.path.append("build_release")

import pygroot as groot
import toml
import glob

registry = groot.Registry()
ImGui = groot.ImGui


def resample(entity):
    sampled = entity.graph_resample(0.03)
    sampled.move_component(entity, groot.components.PlantGraph)
    sampled.destroy()

files = ["A", "B", "C", "D", "E", "F"]
results = {}
ground_truth = {}


async def run_knn(ply):
    ply.remove_component(groot.components.PointNormals)

    ply.graph_from_cloud_knn(10)
    ply.graph_cluster(30)

    return ply

async def run_radius(radius, ply):
    ply.remove_component(groot.components.PointNormals)

    ply.graph_from_cloud_radius(radius)
    ply.graph_cluster(30)

    return ply

async def run_cardenas_et_al(radius, ply):
    await groot.compute_cardenas_et_al(ply, radius)
    ply.graph_cluster(30)

    return ply

run = run_knn
#run = partial(run_cardenas_et_al, 0.1)
#run = partial(run_radius, 0.2)

def update(registry):
    global results
    global task

    ImGui.Begin("Result Window")
    if ImGui.Button("Copy"):
        ImGui.LogToClipboard()

    ImGui.BeginTable("Result Table", 2, ImGui.TableFlags.Borders)
    ImGui.TableSetupColumn("Entity")
    ImGui.TableSetupColumn("MSE")
    ImGui.TableHeadersRow()

    for (name, result) in results.items():
        ImGui.TableNextRow()
        ImGui.TableNextColumn()
        ImGui.Text(name)
        ImGui.TableNextColumn()
        ImGui.Text(str(result))
    ImGui.EndTable()
    ImGui.End()
   

async def update_loop(registry, viewer):
    while not viewer.should_close():
        viewer.step(update)
        await asyncio.sleep(0)

async def init(registry, viewer):
    global ground_truth
    global results

    task = asyncio.create_task(update_loop(registry, viewer))
    await asyncio.sleep(1)

    for name, f in [(x, "/home/joseluis/Trees/TEST_DATA/{}.obj".format(x)) for x in files]:
        e = registry.load_graph(f)
        ground_truth[name] = e

    reconstructed = {}
    for name, f in [(x, "/home/joseluis/Trees/TEST_DATA/{}.ply".format(x)) for x in files]:
        ply = await registry.load_ply(f)
        reconstructed[name] = await run(ply)

    for name in files:
        a = reconstructed[name]
        b = ground_truth[name]

        resample(a)
        resample(b)

        res = await groot.compute_difference(a, b)
        res.name = "Difference {}".format(name)

        results[name] = await groot.evaluate_difference_mse(res)
    
    await task

viewer = registry.create_viewer()
asyncio.run(init(registry, viewer))
