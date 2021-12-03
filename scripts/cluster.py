#!/bin/env python3

import time
import pdb
from functools import partial
from collections import defaultdict

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
    sampled.move_component(entity, groot.components.PlantGraph)
    sampled.destroy()

def reconstruct_cardenas_et_al(entity):
    ply.graph_from_cloud
    

results = defaultdict(dict) 
evaluate_tasks = {}
compute_cardenas_et_al = {}


def init(registry):
    global evaluate_task

    files = ["A", "B", "C", "D", "E", "F"]

    ground_truth = {} 
    for name, f in map(lambda x: (x, "/home/joseluis/Trees/TEST_DATA/{}.obj".format(x)), files):
        e = registry.load_graph(f)
        ground_truth[name] = e


    reconstructed_base = []
    reconstructed_cd = {}
    reconstructed_cd_tasks = {}
    for name, f in map(lambda x: (x, "/home/joseluis/Trees/TEST_DATA/{}.ply".format(x)), files):
        ply = registry.load_ply(f)

        ply.graph_from_cloud_knn(10)
        ply.graph_cluster(30)
        ply.remove_component(groot.components.PointNormals)

        base = registry.new_entity()
        base.name = "{} BASE".format(name)
        ply.move_component(base, groot.components.PlantGraph)
        reconstructed_base.append(base)


        def cd_task_finish(registry, ply, __):
            cd = registry.new_entity()
            cd.name = "{} CD".format(name)
            ply.graph_cluster(30)
            ply.move_component(cd, groot.components.PlantGraph)
            return cd

        cd_task = groot.compute_cardenas_et_al(ply, 0.1)
        cd_task.then(partial(cd_task_finish, registry, ply))
        reconstructed_cd_tasks[name] = cd_task
        print("TNOW", reconstructed_cd_tasks)

    registry.run_tasks()

    for (name, a) in zip(files, reconstructed_base):
        b = ground_truth[name]

        resample(a)
        resample(b)

        evaluate_tasks[("Base method", name)] = groot.evaluate_difference(a, b, True)

    print(reconstructed_cd_tasks)
    for name in files:
        reconstructed_cd_tasks[name].run_till_completion()
        a = reconstructed_cd_tasks[name].get()
        b = ground_truth[name] 

        resample(a)

        evaluate_tasks[("Cardenas et al.", name)] = groot.evaluate_difference(a, b, True)


def update(registry):
    global evaluate_tasks
    global results

    for x in list(evaluate_tasks.keys()):
        task = evaluate_tasks[x]
        (method, name) = x
        if task.ready():
            e, r = task.get()
            e.name = "{} DIFF {}".format(name, method)
            results[name][method] = r
            del(evaluate_tasks[x])

    ImGui.Begin("Result Window")
    for (name, rest) in results.items():
        for (method, result) in rest.items():
            ImGui.Text("{} - {} - {}".format(method, name, result))

    ImGui.End()
    
registry.run_viewer(init, update)