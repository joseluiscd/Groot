#pragma once

#include <groot_graph/plant_graph.hpp>

namespace groot {

/**
 * @brief Compute supported branch length.
 * @note Edge lengths must be propperly computed beforehand.
 */
GROOT_GRAPH_API PropertyMap<float> compute_weights(const PlantGraph& g);

GROOT_GRAPH_API PropertyMap<glm::dvec3> compute_orientation_field(
    const PlantGraph& g,
    const PropertyMap<float>& weights);

GROOT_GRAPH_API PlantGraph reconstruct_livny_et_al(
    const Point_3* cloud,
    size_t size,
    point_finder::PointFinder& root_finder);

}
