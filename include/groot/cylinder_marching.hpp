#pragma once

#include <glm/glm.hpp>
#include <CGAL/Monge_via_jet_fitting.h>
#include <groot/cgal.hpp>

namespace groot {

using SearchTraits = CGAL::Search_traits_3<cgal::Kernel>;

using KdTree = CGAL::Kd_tree<SearchTraits>;
using KNeighbour = CGAL::K_neighbor_search<SearchTraits>;
using Monge_via_jet_fitting = CGAL::Monge_via_jet_fitting<cgal::Kernel>;
using Monge_form = Monge_via_jet_fitting::Monge_form;

struct Curvature {
    cgal::Vector_3 direction;
    cgal::Point_3 curvature_center;
    float radius;
};

void find_cylinders(
    glm::vec3* cloud,
    size_t count);

void compute_differential_quantities(cgal::Point_3* cloud, Curvature* q_out, size_t count, size_t k);

}
