#pragma once

#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/utility/result_of.hpp>
#include <glm/glm.hpp>
#include <groot/cgal.hpp>
#include <groot_graph/plant_graph.hpp>

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

struct GROOT_GRAPH_LOCAL Curvature {
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

struct GROOT_GRAPH_LOCAL Cylinder {
    cgal::Point_3 center; // Point in the middle of the cylinder
    cgal::Vector_3 direction;
    float radius;
    float middle_height; // Height / 2.0
};

GROOT_GRAPH_API float distance(const Cylinder& cylinder, const Point_3& point);

struct GROOT_GRAPH_LOCAL CylinderWithPoints {
    Cylinder cylinder;
    std::vector<Point_3> points;
};

using FitCylinder = CGAL::Shape_detection::Cylinder<RansacTraits>;

class GROOT_GRAPH_LOCAL CurvatureCylinder : public CGAL::Shape_detection::Shape_base<RansacTraits> {
public:
    size_t minimum_sample_size() const override
    {
        return 6;
    }

    void create_shape(const std::vector<size_t>& indices) override;
    float squared_distance(const Point_3& p) const override
    {
        Line_3 axis(this->cylinder.center, this->cylinder.direction);
        return CGAL::squared_distance(axis, p) - this->cylinder.radius;
    }
    void squared_distance(const std::vector<size_t>& indices, std::vector<float>& distances) const override;
    void cos_to_normal(const std::vector<size_t>& indices, std::vector<float>& angles) const override
    {
        for (size_t i = 0; i < indices.size(); i++) {
            angles[i] = 1.0;
        }
    }

    Cylinder cylinder;
};

using DiscardPlane = CGAL::Shape_detection::Plane<RansacTraits>;

bool GROOT_GRAPH_API point_in_cylinder(const cgal::Point_3& p, const Cylinder& c);

void GROOT_GRAPH_API find_cylinders(
    glm::vec3* cloud,
    size_t count);

GROOT_GRAPH_API std::vector<CylinderWithPoints> compute_cylinders(Point_3* cloud, Vector_3* normals, std::vector<size_t>& indices, Ransac::Parameters params = Ransac::Parameters());
GROOT_GRAPH_API std::vector<CylinderWithPoints> compute_cylinders_curvature(Point_3* cloud, Vector_3* normals, std::vector<size_t>& indices, Ransac::Parameters params = Ransac::Parameters());
GROOT_GRAPH_API std::vector<CylinderWithPoints> compute_cylinders_voxelized(Point_3* cloud, Vector_3* normals, size_t count, float voxel_size, Ransac::Parameters params = Ransac::Parameters());
GROOT_GRAPH_API std::vector<CylinderWithPoints> compute_cylinders_voxelized_curvature(Point_3* cloud, Vector_3* normals, size_t count, float voxel_size, Ransac::Parameters params = Ransac::Parameters());
GROOT_GRAPH_API std::vector<CylinderWithPoints> merge_cylinders(const std::vector<CylinderWithPoints>& a, const std::vector<CylinderWithPoints>& b);

GROOT_GRAPH_API std::vector<Vector_3> compute_normals(Point_3* cloud, size_t count, unsigned int k, float radius);
GROOT_GRAPH_API std::vector<Curvature> cylinder_filter(Curvature* input, size_t count, float height);

}
