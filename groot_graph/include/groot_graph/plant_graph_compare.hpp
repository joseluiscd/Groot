#pragma once

#include <groot_graph/plant_graph.hpp>

namespace groot {

struct PlantGraphCompareResult {
};

struct PlantGraphCompareParams {
};

PlantGraph resample_plant_graph(const PlantGraph& graph, float sample_length);
PlantGraphCompareResult plant_graph_compare(const PlantGraphCompareParams& p, const groot::PlantGraph& ground_truth, const groot::PlantGraph& plant);
groot::PlantGraph plant_graph_nn(const groot::PlantGraph& a, const groot::PlantGraph& b);
float plant_graph_nn_score(const groot::PlantGraph& g);

}