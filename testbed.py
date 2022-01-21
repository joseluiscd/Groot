import pygroot

reg = pygroot.Registry()
reg.load("wololo.ggf")

e = reg.entities()[0]

graph = e[pygroot.components.PlantGraph]
print(graph)
print(graph.num_vertices)
print(graph.num_edges)
a = iter(graph.vertices)
print(next(a).point)
print(next(a).point)
print(next(a).point)
print(next(a))
print(next(a))
print(next(a))
print(next(a))