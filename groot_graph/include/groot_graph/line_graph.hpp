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

LineGraph compute_line_graph(const groot::PlantGraph& line_graph);

}