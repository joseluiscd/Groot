#!/bin/env python3

import sys
if "--debug" in sys.argv:
    sys.path.append("build/app")
else:
    sys.path.append("build_release/app")

import groot
import toml

registry = groot.Registry()
ImGui = groot.ImGui

def init(registry):
    entity = registry.load_ply("/home/joseluis/Trees/palo.ply")
    entity.visible = True

    entity.graph_from_cloud_knn(10)
    entity.graph_cluster(10)

    resampled = entity.graph_resample(0.3)
    resampled.visible = True



show = True
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