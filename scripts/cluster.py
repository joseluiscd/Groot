#!/bin/env python3

import time
import pdb

import sys
if "--debug" in sys.argv:
    sys.path.append("build/app")
else:
    sys.path.append("build_release/app")

import groot
import toml
import glob

registry = groot.Registry()
ImGui = groot.ImGui

def f1():
    print("F1")
    return 4

def f2(v):
    time.sleep(1)
    print("F2", v)
    return v+1

def resample(entity):
    sampled = entity.graph_resample(0.03)
    sampled.move_component(entity, groot.components.PlantGraph)
    sampled.destroy()
    
def spawn(reg):
    task = groot.task.Task(f1, "The F Operator", groot.task.TaskMode.Sync)
    task.then(f2, groot.task.TaskMode.Async)
    reg.schedule_task(task)

def init(registry):
    ground_truth = []
    for f in glob.glob("/home/joseluis/Trees/TEST_DATA/A.obj"):
        ground_truth.append(registry.load_graph(f))

    reconstructed = []
    for f in glob.glob("/home/joseluis/Trees/TEST_DATA/A.ply"):
        ply = registry.load_ply(f)

        ply.graph_from_cloud_knn(10)
        ply.graph_cluster(30)
        reconstructed.append(ply)

    for (a, b) in zip(reconstructed, ground_truth):
        resample(a)
        resample(b)

        g = a.match_graph(b)
        g.visible = True


show = False
def update(registry):
    global show

    if show:
        expanded, show = ImGui.Begin("Test Window", show)
        if expanded:
            ImGui.Text("Hello world")
            if ImGui.Button("Push me"):
                print("Wololo")
                spawn(registry)
     
        ImGui.End()
    
    ImGui.Begin("App")
    if ImGui.BeginMenuBar():
        if ImGui.BeginMenu("Window"):
            _, show = ImGui.MenuItem("Show strange window", selected=show)

            ImGui.EndMenu()

        ImGui.EndMenuBar()
    ImGui.End()

registry.run_viewer(init, update)