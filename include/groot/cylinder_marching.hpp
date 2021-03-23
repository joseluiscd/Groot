#pragma once

#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <boost/utility/result_of.hpp>
#include <glm/glm.hpp>
#include <groot/cgal.hpp>
#include <groot/plant_graph.hpp>
#include <boost/property_map/transform_value_property_map.hpp>

namespace groot {

using SearchTraits = CGAL::Search_traits_3<cgal::Kernel>;

using KdTree = CGAL::Kd_tree<SearchTraits>;
using KNeighbour = CGAL::K_neighbor_search<SearchTraits>;
using Monge_via_jet_fitting = CGAL::Monge_via_jet_fitting<cgal::Kernel>;
using Monge_form = Monge_via_jet_fitting::Monge_form;
using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;

struct Curvature {
    cgal::Point_3 sampled_point;
    cgal::Point_3 curvature_center;
    cgal::Vector_3 direction;
    float radius;
};

template <typename T, typename FieldType, FieldType T::*offset>
struct get_field {
    using type = FieldType&;
    get_field() {}
    const FieldType& operator()(const T& data) const
    {
        return data.*offset;
    }
};

using CurvatureCenterProperty = get_field<Curvature, cgal::Point_3, &Curvature::curvature_center>;
using CurvatureCenterPropertyMap = boost::transform_value_property_map<
    CurvatureCenterProperty,
    boost::iterator_property_map<Curvature*, boost::identity_property_map>>;
using CurvatureCenterSearchTraits = CGAL::Search_traits_adapter<size_t, CurvatureCenterPropertyMap, SearchTraits>;

struct Cylinder {
    cgal::Point_3 center; // Point in the middle of the cylinder
    cgal::Vector_3 direction;
    float radius;
    float middle_height; // Height / 2.0
};

bool point_in_cylinder(const cgal::Point_3& p, const Cylinder& c);

void find_cylinders(
    glm::vec3* cloud,
    size_t count);

void compute_differential_quantities(cgal::Point_3* cloud, Curvature* q_out, size_t count, size_t k, size_t d = 3, size_t dprime = 2);
PlantGraph cylinder_marching(Curvature* input, size_t count, float height, float h_extend = 2.0f, float r_extend = 1.5f);

}
