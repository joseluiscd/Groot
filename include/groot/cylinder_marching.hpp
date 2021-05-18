#pragma once

#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/utility/result_of.hpp>
#include <glm/glm.hpp>
#include <groot/cgal.hpp>
#include <groot/plant_graph.hpp>

namespace groot {

using SearchTraits = CGAL::Search_traits_3<cgal::Kernel>;

using KdTree = CGAL::Kd_tree<SearchTraits>;
using KNeighbour = CGAL::K_neighbor_search<SearchTraits>;
using Monge_via_jet_fitting = CGAL::Monge_via_jet_fitting<cgal::Kernel>;
using Monge_form = Monge_via_jet_fitting::Monge_form;
using FuzzySphere = CGAL::Fuzzy_sphere<SearchTraits>;

using PointMap = boost::iterator_property_map<Point_3*, boost::identity_property_map>;
using NormalMap = boost::iterator_property_map<Vector_3*, boost::identity_property_map>;

using RansacTraits = CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, std::vector<size_t>, PointMap, NormalMap>;
using Ransac = CGAL::Shape_detection::Efficient_RANSAC<RansacTraits>;

struct Curvature {
    cgal::Point_3 sampled_point;
    cgal::Point_3 curvature_center;
    cgal::Vector_3 direction;
    float radius;
};

template <typename T, typename FieldType, FieldType T::*offset>
struct get_field {
    using type = FieldType&;
    get_field() { }
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

float distance(const Cylinder& cylinder, const Point_3& point);

struct CylinderWithPoints {
    Cylinder cylinder;
    std::vector<Point_3> points;
};

using FitCylinder = CGAL::Shape_detection::Cylinder<RansacTraits>;

bool point_in_cylinder(const cgal::Point_3& p, const Cylinder& c);

void find_cylinders(
    glm::vec3* cloud,
    size_t count);

std::vector<CylinderWithPoints> compute_cylinders(Point_3* cloud, Vector_3* normals, std::vector<size_t>& indices, Ransac::Parameters params = Ransac::Parameters());
std::vector<CylinderWithPoints> compute_cylinders_voxelized(Point_3* cloud, Vector_3* normals, size_t count, float voxel_size, Ransac::Parameters params = Ransac::Parameters());
std::vector<CylinderWithPoints> merge_cylinders(const std::vector<CylinderWithPoints>& a, const std::vector<CylinderWithPoints>& b);

std::vector<Vector_3> compute_normals(Point_3* cloud, size_t count, unsigned int k, float radius);
std::vector<Curvature> cylinder_filter(Curvature* input, size_t count, float height);

}

namespace boost {
namespace serialization {
    template <typename Archive>
    void serialize(Archive& ar, groot::Cylinder& cylinder, unsigned int)
    {
        ar& cylinder.center& cylinder.direction& cylinder.radius& cylinder.middle_height;
    }

    template <typename Archive>
    void serialize(Archive& ar, groot::CylinderWithPoints& cylinder, unsigned int)
    {
        ar& cylinder.cylinder& cylinder.points;
    }
}
}