#pragma once

#include <groot_graph/groot_graph.hpp>
#include <groot_graph/plant_graph.hpp>

namespace groot {

GROOT_GRAPH_API groot::PlantGraph load_plant_graph(const char* filename);
GROOT_GRAPH_API void save_plant_graph(const char* filename, const groot::PlantGraph& plant);

}