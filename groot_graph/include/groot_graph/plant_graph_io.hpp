#pragma once

#include <groot_graph/plant_graph.hpp>

namespace groot {

groot::PlantGraph load_plant_graph(const char* filename);
void save_plant_graph(const char* filename, const groot::PlantGraph& plant);

}