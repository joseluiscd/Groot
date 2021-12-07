#pragma once

#include <groot/groot.hpp>
#include <groot_graph/plant_graph.hpp>
#include <groot_graph/cylinder_marching.hpp>

namespace groot {

GROOT_API PlantGraph connect_cylinders(CylinderWithPoints* points, size_t count);

}