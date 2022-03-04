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
import numpy as np

registry = groot.Registry()
ImGui = groot.ImGui

def resample(entity):
    sampled = entity.graph_resample(0.03)
    sampled.move_component(entity, groot.components.PlantGraph)
    sampled.destroy()

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
    ply.remove_component(groot.components.PointNormals)

    await groot.compute_cardenas_et_al(ply, radius)
    ply.graph_cluster(30)

    return ply

f = "F"
run = run_cardenas_et_al
#run = run_radius
radii = np.arange(0.0, 0.6, 0.005)

results = [] 
ground_truth = {}

def update(registry):
    global results
    global task

    ImGui.Begin("Result Window")
    if ImGui.Button("Copy"):
        ImGui.LogToClipboard()

    ImGui.BeginTable("Result Table", 2, ImGui.TableFlags.Borders)
    ImGui.TableSetupColumn("Radius")
    ImGui.TableSetupColumn("MSE")
    ImGui.TableHeadersRow()

    for (name, result) in results:
        ImGui.TableNextRow()
        ImGui.TableNextColumn()
        ImGui.Text(str(name))
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

    ground_truth_f = "/home/joseluis/Trees/TEST_DATA/{}.obj".format(f)
    ground_truth = registry.load_graph(ground_truth_f)
    resample(ground_truth)

    point_cloud_f = "/home/joseluis/Trees/TEST_DATA/{}.ply".format(f)
    ply = await registry.load_ply(point_cloud_f)
    for radius in radii:
        reconstructed = await run(radius, ply)
        resample(reconstructed)

        res = await groot.compute_difference(ground_truth, reconstructed)
        res.name = "Difference {}".format(radius)

        results.append((radius, await groot.evaluate_difference_mse(res)))
        res.destroy()
    
    print()
    print()
    print()
    for (r, e) in results:
        print("({}, {})".format(r, e), end=" ")
    print()
    await task

viewer = registry.create_viewer()
asyncio.run(init(registry, viewer))
