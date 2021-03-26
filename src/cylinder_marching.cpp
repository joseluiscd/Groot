#include <boost/iterator/transform_iterator.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <groot/cylinder_marching.hpp>
#include <tbb/parallel_for.h>
#include <gfx/debug_draw.hpp>
#include <CGAL/property_map.h>

namespace groot {

void compute_differential_quantities(cgal::Point_3* cloud, Curvature* q_out, size_t count, size_t k, size_t d, size_t dprime)
{
    KdTree kdtree;
    kdtree.insert(cloud, cloud + count);
    kdtree.build<CGAL::Parallel_tag>();

    std::vector<cgal::Point_3> nn(k);
    tbb::parallel_for((size_t)0, count, [&](size_t i) {
        KNeighbour knn(kdtree, cloud[i], k);

        Monge_via_jet_fitting monge_fitting;

        size_t u = 0;
        for (auto it = knn.begin(); it != knn.end(); ++it) {
            nn[u++] = it->first;

            if (i == 20) {
                dd::point({ it->first.x(), it->first.y(), it->first.z() }, dd::colors::Black, 3.0);
            }

        }

        Monge_form f = monge_fitting(nn.begin(), nn.end(), d, dprime);

        float r = 1.0f / f.principal_curvatures(0);
        cgal::Vector_3 n = f.normal_direction();
        cgal::Vector_3 d1 = f.maximal_principal_direction();
        cgal::Point_3 b = cloud[i] - r * n;

        q_out[i].curvature_center = b;
        q_out[i].direction = d1;
        q_out[i].radius = std::abs(r);
    });
}

using KdTreeCurvature = CGAL::Kd_tree<CurvatureCenterSearchTraits>;
using FuzzySphereCurvature = CGAL::Fuzzy_sphere<CurvatureCenterSearchTraits>;

bool point_in_cylinder(const cgal::Point_3& p, const Cylinder& c)
{
    // Line representing the axis of the cylinder
    cgal::Line_3 l(c.center, c.direction);
    // Projection of the point in the axis of the cylinder
    cgal::Point_3 projection = l.projection(p);

    return CGAL::squared_distance(l, p) < c.radius * c.radius
        && CGAL::squared_distance(c.center, projection) < c.middle_height * c.middle_height;
}

void find_points_in_cylinder(const KdTreeCurvature& kdtree, Curvature* curvature, const Cylinder& c, std::vector<size_t>& out_points)
{
    float broad_radius = std::sqrt(c.middle_height * c.middle_height + c.radius * c.radius);
    FuzzySphereCurvature  broad_search(c.center, broad_radius, 0.0, kdtree.traits());

    std::vector<size_t> points;
    kdtree.search(std::back_inserter(points), broad_search);

    for (size_t i = 0; i < points.size(); i++) {
        if (point_in_cylinder(curvature[points[i]].curvature_center, c)) {
            out_points.push_back(points[i]);
        }
    }
}

Cylinder cylinder_from_curvature(const Curvature& c, float height) {
    Cylinder cylinder;
    cylinder.center = c.curvature_center;
    cylinder.direction = c.direction;
    cylinder.middle_height = height; 
    cylinder.radius = c.radius;

    return cylinder;
}

bool cylinders_similar(const Cylinder& c1, const Cylinder& c2)
{
    float cos_angle = c1.direction * c2.direction;
    float radius_ratio = c1.radius / c2.radius;
    return point_in_cylinder(c1.center, c2)
        && point_in_cylinder(c2.center, c1)
        && std::abs(cos_angle) > 0.9
        && radius_ratio > 0.9
        && radius_ratio < 1.1;
}

std::vector<Curvature> cylinder_filter(Curvature* input, size_t count, float height)
{
    CurvatureCenterPropertyMap map(CurvatureCenterProperty(), boost::make_iterator_property_map(input, boost::identity_property_map()));
    KdTreeCurvature kd { KdTreeCurvature::Splitter(), CurvatureCenterSearchTraits(map) };
    for (size_t i = 0; i < count; i++) {
        kd.insert(i);
    }
    kd.build<CGAL::Parallel_tag>();


    std::vector<bool> processed(count, false);
    std::vector<size_t> out_points;
    std::vector<Curvature> output;

    for (size_t i = 0; i < count; i++) {
        if (processed[i]) {
            continue;
        }

        Cylinder cylinder = cylinder_from_curvature(input[i], 0.3f); // TODO: CHANGE RADIUS

        find_points_in_cylinder(kd, input, cylinder, out_points);

        size_t candidates = 0;
        for (size_t j = 0; j < out_points.size(); j++) {
            Cylinder other = cylinder_from_curvature(input[out_points[j]], 0.3f);
            if (cylinders_similar(cylinder, other)) {
                candidates ++;
            }
        }

        if (candidates >= 5) { //TODO: CHANGE THRESHOLD
            for (size_t j = 0; j < out_points.size(); j++) {
                processed[out_points[j]] = true;
                output.push_back(input[out_points[j]]);
            }
        }
        out_points.clear();
    }

    return output;
}

}


#include <doctest/doctest.h>

namespace groot {

TEST_CASE("cylinder inclusion test")
{
    groot::Cylinder c1 = {
        .center = groot::Point_3(0.0, 0.0, 0.0),
        .direction = groot::Vector_3(1.0, 0.0, 0.0),
        .radius = 0.5,
        .middle_height = 0.5,
    };
    //Points in the cylinder
    groot::Point_3 p1(0.0, 0.0, 0.0);
    groot::Point_3 p2(0.499, 0.49, 0.0);

    CHECK(point_in_cylinder(p1, c1));
    CHECK(point_in_cylinder(p2, c1));

    //Points outside the cylinder
    groot::Point_3 p3(0.9, 0.0, 0.0);
    groot::Point_3 p4(0.499, 0.49, 0.49);

    CHECK_FALSE(point_in_cylinder(p3, c1));
    CHECK_FALSE(point_in_cylinder(p4, c1));
}


TEST_CASE("similar cylinders")
{
    groot::Cylinder c = {
        .center = groot::Point_3(0.0, 0.0, 0.0),
        .direction = groot::Vector_3(1.0, 0.0, 0.0),
        .radius = 0.5,
        .middle_height = 0.5,
    };
    groot::Cylinder c_inverted = {
        .center = groot::Point_3(0.0, 0.0, 0.0),
        .direction = groot::Vector_3(-1.0, 0.0, 0.0),
        .radius = 0.5,
        .middle_height = 0.5,
    };
    groot::Cylinder c_displaced = {
        .center = groot::Point_3(0.2, 0.0, 0.0),
        .direction = groot::Vector_3(-1.0, 0.0, 0.0),
        .radius = 0.49,
        .middle_height = 0.7,
    };

    CHECK(cylinders_similar(c, c));
    CHECK(cylinders_similar(c_inverted, c));
    CHECK(cylinders_similar(c_displaced, c));
    CHECK(cylinders_similar(c_displaced, c_inverted));
}

}
