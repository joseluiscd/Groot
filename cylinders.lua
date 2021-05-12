print(config)
dofile ( config )

print("Reading from", params.input)

registry = Registry.get()

entity = registry:load_ply(params.input)
print("Loaded PLY")

print("Computing normals")
entity:compute_normals(params.normals)
print("Marching cylinders")
entity:cylinder_marching(params.cylinders)
print("Filtering cylinders")
entity:cylinder_filter(params.cylinder_filter)

print("Done!")
print("Writing output file")
registry:save(params.output)
print("Finished!")