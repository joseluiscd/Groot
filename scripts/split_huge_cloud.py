#!/bin/env python3

import asyncio
import faulthandler
faulthandler.enable()

import sys
if "--debug" in sys.argv:
    sys.path.append("build")
    sys.argv.remove("--debug")
else:
    sys.path.append("build_release")

import pygroot as groot

registry = groot.Registry()
ImGui = groot.ImGui



async def init():
    global registry
    global ground_truth
    global results

    huge_ply = await registry.load_ply(sys.argv[1])
    print("Done")

    parts = await huge_ply.split_cloud(19.9)
    for i, part in enumerate(parts):
        print("Save: ", i, part)
        part.save_ply("/home/joseluis/Trees/tmp/" + str(i).zfill(3) + ".ply")



if __name__ == "__main__":
    asyncio.run(init())
