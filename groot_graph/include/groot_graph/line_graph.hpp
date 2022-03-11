#pragma once

#include <groot_graph/plant_graph.hpp>
#include <groot/cgal.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/pending/disjoint_sets.hpp>

namespace groot {

struct LineGraph {
    PlantGraph graph;
    PropertyMap<Vector_3> direction;
};

/**
 * @brief Compute a variation of the Line Graph,
 * where edges are only connected if they form different levels on the tree.
 * @param[in] input Input Graph
 * @param[out] direction For each vertex, the tangent direction.
*/
groot::PlantGraph compute_line_graph(const groot::PlantGraph& input, PropertyMap<Vector_3>* direction = nullptr);

}