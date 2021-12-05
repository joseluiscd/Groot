#pragma once

#include <groot_graph/plant_graph.hpp>
#include <groot_graph/cylinder_marching.hpp>

namespace groot {

PlantGraph connect_cylinders(CylinderWithPoints* points, size_t count);

}