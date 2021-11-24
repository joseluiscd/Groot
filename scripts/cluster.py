#!/bin/env python3

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

def resample(entity):
    sampled = entity.graph_resample(0.03)
    sampled.visible = True
    entity.destroy()
    return sampled
    
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
        a = resample(a)
        b = resample(b)

        g = a.match_graph(b)
        g.visible = True

        u = b.match_graph(b)
        u.visible = True


show = False
def update(registry):
    global show

    if show:
        expanded, show = ImGui.Begin("Test Window", show)
        if expanded:
            ImGui.Text("Hello world")
            if ImGui.Button("Push me"):
                print("Wololo")
     
        ImGui.End()
    
    ImGui.Begin("App")
    if ImGui.BeginMenuBar():
        if ImGui.BeginMenu("Window"):
            _, show = ImGui.MenuItem("Show strange window", selected=show)

            ImGui.EndMenu()

        ImGui.EndMenuBar()
    ImGui.End()

registry.run_viewer(init, update)