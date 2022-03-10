#!/bin/env python3

import asyncio
import faulthandler
import os
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

async def repair(filename):
    graph_entity = await registry.load_graph(filename)
    print("LOADED", graph_entity, filename)
    await graph_entity.repair_graph()
    await graph_entity.save_graph(filename)


async def init():
    global registry

    files = glob.glob(os.path.join(sys.argv[1], "??.obj"))
    tasks = list(map(repair, files))

    await asyncio.gather(*tasks)
    await registry.save("workspace.groot_workspace")
    print("Done")

if __name__ == "__main__":
    asyncio.run(init())
