#pragma once

#include <groot_graph/plant_graph.hpp>

namespace groot {

/**
 * @brief Cluster graph with a fixed number of distance groups.
 * 
 * @param[in] graph 
 * @param[out] points Maps `Vertex in output -> Points in input`. Created if not null.
 * @return PlantGraph clustered
 */
PlantGraph graph_cluster_fixed_groups(const PlantGraph& graph, size_t distance_groups, PropertyMap<std::vector<Point_3>>* points = nullptr);

}