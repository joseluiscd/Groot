#pragma once

#include <groot/plant_graph.hpp>
#include <groot/cylinder_marching.hpp>

namespace groot {

PlantGraph connect_cylinders(CylinderWithPoints* points, size_t count);

}