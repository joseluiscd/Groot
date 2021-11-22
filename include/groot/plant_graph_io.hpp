#pragma once

#include <groot/plant_graph.hpp>

namespace groot {

groot::PlantGraph load_plant_graph(const char* filename);
void save_plant_graph(const char* filename, const groot::PlantGraph& plant);

}