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
    const PropertyMap<float>& weights,
    unsigned max_iterations = 300);

GROOT_GRAPH_API PlantGraph reconstruct_livny_et_al(
    const Point_3* cloud,
    size_t size,
    point_finder::PointFinder& root_finder,
    unsigned max_iterations = 5,
    unsigned max_orientation_iterations = 300,
    unsigned max_relocation_iterations = 30);

GROOT_GRAPH_API PlantGraph iteration_livny_et_al(
    const PlantGraph& g,
    point_finder::PointFinder& root_finder = point_finder::MinYPointFinder,
    unsigned max_orientation_iterations = 300,
    unsigned max_relocation_iterations = 30);

GROOT_GRAPH_API void update_positions_for_orientation(
    PlantGraph& graph,
    const PropertyMap<float>& weights,
    const PropertyMap<glm::dvec3>& orientations,
    unsigned max_iterations = 30);

}
