#pragma once

#include <groot/plant_graph.hpp>

namespace groot {

struct PlantGraphCompareResult {
};

struct PlantGraphCompareParams {
};

PlantGraph resample_plant_graph(const PlantGraph& graph, float sample_lenght);
void plant_graph_compare(const PlantGraphCompareParams& p, const groot::PlantGraph& ground_truth, const groot::PlantGraph&);

}