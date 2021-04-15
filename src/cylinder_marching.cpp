#include <CGAL/property_map.h>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <gfx/debug_draw.hpp>
#include <groot/cylinder_marching.hpp>
#include <tbb/parallel_for.h>
#include <CGAL/pca_estimate_normals.h>
#include <spdlog/spdlog.h>

namespace groot {


std::vector<Vector_3> compute_normals(Point_3* cloud, size_t count, unsigned int k, float radius)
{
    std::vector<Vector_3> normals(count);
    
    std::vector<size_t> indices;
    std::copy(boost::make_counting_iterator<size_t>(0), boost::make_counting_iterator(count), std::back_inserter(indices));

    CGAL::pca_estimate_normals<CGAL::Parallel_tag>(indices, k,
        CGAL::parameters::point_map(CGAL::make_property_map(cloud))
        .normal_map(CGAL::make_property_map(normals.data()))
        .neighbor_radius(radius));

    return normals;
}

void compute_cylinders(Point_3* cloud, Vector_3* normals, size_t count, std::vector<Cylinder>& out, Ransac::Parameters params)
{
    std::vector<size_t> indices;
    std::copy(boost::make_counting_iterator<size_t>(0), boost::make_counting_iterator(count), std::back_inserter(indices));

    Ransac ransac;
    ransac.set_input(indices, PointMap(cloud), NormalMap(normals));
    ransac.add_shape_factory<FitCylinder>();
    ransac.detect(params);

    spdlog::info("Shape count: {}", ransac.shapes().size());
    
    for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
        auto cylinder = static_cast<FitCylinder*>(&**it);

        Point_3 center = cylinder->axis().point();
        Vector_3 axis = cylinder->axis().to_vector(); 

        std::cout << cylinder->axis() << " / " << cylinder->axis().direction() << " - " << cylinder->axis().direction().vector();
        out.push_back(Cylinder {
            center,
            axis,
            cylinder->radius(),
            std::sqrt(axis.squared_length()) * 0.5f
        });
    }
}

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
    FuzzySphereCurvature broad_search(c.center, broad_radius, 0.0, kdtree.traits());

    std::vector<size_t> points;
    kdtree.search(std::back_inserter(points), broad_search);

    for (size_t i = 0; i < points.size(); i++) {
        if (point_in_cylinder(curvature[points[i]].curvature_center, c)) {
            out_points.push_back(points[i]);
        }
    }
}

Cylinder cylinder_from_curvature(const Curvature& c, float height)
{
    Cylinder cylinder;
    cylinder.center = c.curvature_center;
    cylinder.direction = c.direction;
    cylinder.middle_height = height;
    cylinder.radius = c.radius;

    return cylinder;
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

}
