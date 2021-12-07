#pragma once

#include <groot/groot.hpp>
#include <groot_graph/plant_graph.hpp>

namespace groot {

GROOT_API groot::PlantGraph load_plant_graph(const char* filename);
GROOT_API void save_plant_graph(const char* filename, const groot::PlantGraph& plant);

}