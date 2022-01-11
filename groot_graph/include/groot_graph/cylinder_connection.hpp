#pragma once

#include <groot_graph/groot_graph.hpp>
#include <groot_graph/plant_graph.hpp>
#include <groot_graph/cylinder_marching.hpp>

namespace groot {

GROOT_GRAPH_API PlantGraph connect_cylinders(CylinderWithPoints* points, size_t count);

}