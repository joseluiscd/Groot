#pragma once

#include <groot/disjoint_sets.hpp>
#include <groot_graph/plant_graph.hpp>

namespace groot {

GROOT_GRAPH_API DisjointSets compute_connected_components(const PlantGraph& g);
GROOT_GRAPH_API void repair_connectivity(PlantGraph& g, DisjointSets& components);


}