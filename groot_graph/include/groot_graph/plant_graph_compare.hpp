#pragma once

#include <groot_graph/plant_graph.hpp>

namespace groot {

struct GROOT_GRAPH_LOCAL PlantGraphCompareResult {
};

struct GROOT_GRAPH_LOCAL PlantGraphCompareParams {
};

GROOT_GRAPH_API PlantGraph resample_plant_graph(const PlantGraph& graph, float sample_length);
GROOT_GRAPH_API PlantGraphCompareResult plant_graph_compare(const PlantGraphCompareParams& p, const groot::PlantGraph& ground_truth, const groot::PlantGraph& plant);
GROOT_GRAPH_API groot::PlantGraph plant_graph_nn(const groot::PlantGraph& a, const groot::PlantGraph& b);
GROOT_GRAPH_API float plant_graph_nn_score(const groot::PlantGraph& g);

}