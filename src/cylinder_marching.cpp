#include <CGAL/jet_estimate_normals.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/tags.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <gfx/debug_draw.hpp>
#include <groot/cloud.hpp>
#include <groot/cylinder_marching.hpp>
#include <iterator>
#include <spdlog/spdlog.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <spdlog/spdlog.h>

namespace groot {

float distance(const Cylinder& cylinder, const Point_3& point)
{
    Line_3 axis(cylinder.center, cylinder.direction);
    Point_3 projection = axis.projection(point);
    float axis_dist = std::sqrt(CGAL::squared_distance(projection, point));
    if (CGAL::squared_distance(projection, cylinder.center) < cylinder.middle_height * cylinder.middle_height) {
        return std::sqrt(axis_dist) - cylinder.radius;
    } else {
        float proj_center_dist = std::sqrt(CGAL::squared_distance(projection, cylinder.center));
        float x_diff = (axis_dist - cylinder.radius);
        float y_diff = (proj_center_dist - cylinder.middle_height);

        return std::sqrt(x_diff * x_diff + y_diff * y_diff);
    }
}

std::vector<Vector_3> compute_normals(const Point_3* cloud, size_t count, unsigned int k, float radius)
{
    std::vector<Vector_3> normals(count);

    std::vector<size_t> indices;
    std::copy(boost::make_counting_iterator<size_t>(0), boost::make_counting_iterator(count), std::back_inserter(indices));

    CGAL::jet_estimate_normals<CGAL::Parallel_tag>(indices, k,
        CGAL::parameters::point_map(CGAL::make_property_map(cloud))
            .normal_map(CGAL::make_property_map(normals.data()))
            .neighbor_radius(radius));

    return normals;
}

Cylinder from_cgal(const FitCylinder& c)
{
    cgal::Line_3 axis = c.axis();

    return Cylinder {
        axis.point(0.0),
        axis.to_vector(),
        c.radius(),
        c.radius()
    };
}

std::vector<CylinderWithPoints> compute_cylinders(Point_3* cloud, Vector_3* normals, std::vector<size_t>& indices, Ransac::Parameters params)
{
    std::vector<CylinderWithPoints> cylinders;

    Ransac ransac;
    ransac.set_input(indices, PointMap(cloud), NormalMap(normals));
    //ransac.add_shape_factory<DiscardPlane>();
    ransac.add_shape_factory<FitCylinder>();
    ransac.detect(params);

    spdlog::info("Shape count: {}", ransac.shapes().size());

    for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
        FitCylinder* c = dynamic_cast<FitCylinder*>(&**it);
        if (c == nullptr) {
            continue;
        }

        std::vector<Point_3> points;

        const std::vector<size_t>& assigned_points = c->indices_of_assigned_points();
        for (size_t i = 0; i < assigned_points.size(); i++) {
            points.push_back(cloud[indices[assigned_points[i]]]);
        }

        cylinders.emplace_back(CylinderWithPoints {
            from_cgal(*c),
            std::move(points) });
    }

    return cylinders;
}

void CurvatureCylinder::create_shape(const std::vector<size_t>& indices)
{
    std::vector<cgal::Point_3> pts(indices.size());
    Monge_via_jet_fitting monge_fitting;

    size_t u = 0;
    for (size_t i = 0; i < indices.size(); i++) {
        pts[i] = this->point(indices[i]);
    }

    Monge_form f = monge_fitting(pts.begin(), pts.end(), 2, 2);

    Vector_3 n = f.normal_direction();
    cylinder.radius = 1.0f / f.principal_curvatures(0);
    cylinder.direction = f.maximal_principal_direction();
    cylinder.center = point(indices[0]) - cylinder.radius * n;
    cylinder.middle_height = 0.5;
}
 
void CurvatureCylinder::squared_distance(const std::vector<size_t>& indices, std::vector<float>& distances) const
{
    for (size_t i = 0; i < indices.size(); i++) {
        distances[i] = this->squared_distance(this->point(i));
    }
}

std::vector<CylinderWithPoints> compute_cylinders_curvature(Point_3* cloud, Vector_3* normals, std::vector<size_t>& indices, Ransac::Parameters params)
{
    std::vector<CylinderWithPoints> cylinders;

    Ransac ransac;
    ransac.set_input(indices, PointMap(cloud), NormalMap(normals));
    ransac.add_shape_factory<CurvatureCylinder>();
    ransac.detect(params);

    spdlog::info("Shape count: {}", ransac.shapes().size());

    for (auto it = ransac.shapes().begin(); it != ransac.shapes().end(); ++it) {
        CurvatureCylinder* c = dynamic_cast<CurvatureCylinder*>(&**it);
        if (c == nullptr) {
            continue;
        }

        std::vector<Point_3> points;

        const std::vector<size_t>& assigned_points = c->indices_of_assigned_points();
        for (size_t i = 0; i < assigned_points.size(); i++) {
            points.push_back(cloud[indices[assigned_points[i]]]);
        }

        cylinders.emplace_back(CylinderWithPoints {
            c->cylinder,
            std::move(points) });
    }

    return cylinders;
}
std::vector<CylinderWithPoints> compute_cylinders_voxelized(Point_3* cloud, Vector_3* normals, size_t count, float voxel_size, Ransac::Parameters params)
{
    VoxelGrid grid = voxel_grid(cloud, count, voxel_size);
    std::vector<CylinderWithPoints> cylinders;
    std::shared_mutex mutex;

    tbb::parallel_for_each(grid.voxels.begin(), grid.voxels.end(), [&](auto& cell) {
        if (cell.size() > params.min_points) {
            std::vector<CylinderWithPoints> cell_cylinders = compute_cylinders(cloud, normals, cell, params);

            std::shared_lock _lock(mutex);
            std::copy(cell_cylinders.begin(), cell_cylinders.end(), std::back_inserter(cylinders));
        }
    });

    return cylinders;
}

std::vector<CylinderWithPoints> compute_cylinders_voxelized_curvature(Point_3* cloud, Vector_3* normals, size_t count, float voxel_size, Ransac::Parameters params)
{
    VoxelGrid grid = voxel_grid(cloud, count, voxel_size);
    std::vector<CylinderWithPoints> cylinders;
    std::shared_mutex mutex;

    tbb::parallel_for_each(grid.voxels.begin(), grid.voxels.end(), [&](auto& cell) {
        if (cell.size() > params.min_points) {
            std::vector<CylinderWithPoints> cell_cylinders = compute_cylinders_curvature(cloud, normals, cell, params);

            std::shared_lock _lock(mutex);
            std::copy(cell_cylinders.begin(), cell_cylinders.end(), std::back_inserter(cylinders));
        }
    });

    return cylinders;
}

/** Check if cylinders are similar
@param a Cylinder a
@param b Cyilnder b
@param cos_angle The minimum cos (similarity) of the angle between the axes of a-b: 1 = equal (e.g. 0.9 for 25º)
@param max_dist The maximum distance from center of cylinder to the projection on the other.
@param radius_ratio The ratio of radii: 1 = equal, 0 = very different 
*/
bool cylinders_similar(const Cylinder& a, const Cylinder& b, float cos_angle, float max_dist, float radius_ratio)
{
    float m_a = 1.0 / std::sqrt(a.direction.squared_length());
    float m_b = 1.0 / std::sqrt(b.direction.squared_length());

    float ab_cos = std::abs(a.direction * b.direction * m_a * m_b);

    if (ab_cos < cos_angle) {
        return false;
    }

    Line_3 a_axis = cgal::Line_3(a.center, a.direction);
    Line_3 b_axis = cgal::Line_3(b.center, b.direction);

    Point_3 a_proj_b = b_axis.projection(a.center);
    Point_3 b_proj_a = a_axis.projection(b.center);

    float a_axis_dist = CGAL::squared_distance(b_proj_a, a.center);
    float b_axis_dist = CGAL::squared_distance(a_proj_b, b.center);

    float max_dist_squared = max_dist * max_dist;

    if (a_axis_dist > a.middle_height && b_axis_dist > b.middle_height) {
        return false;
    }

    float radius_big;
    float radius_small;

    if (a.radius > b.radius) {
        radius_big = a.radius;
        radius_small = b.radius;
    } else {
        radius_big = b.radius;
        radius_small = a.radius;
    }

    if (radius_small / radius_big < radius_ratio) {
        return false;
    }

    return true;
}

//TODO: Possible change of this
CylinderWithPoints merge_cylinders(const CylinderWithPoints& a, const CylinderWithPoints& b)
{
    CylinderWithPoints ret;

    std::copy(a.points.begin(), b.points.end(), std::back_inserter(ret.points));
    std::copy(a.points.begin(), b.points.begin(), std::back_inserter(ret.points));

    if (a.cylinder.middle_height > b.cylinder.middle_height) {
        ret.cylinder = a.cylinder;
    } else {
        ret.cylinder = b.cylinder;
    }

    return ret;
}

std::vector<CylinderWithPoints> merge_cylinders(const std::vector<CylinderWithPoints>& a, const std::vector<CylinderWithPoints>& b)
{
    std::vector<CylinderWithPoints> result;
    for (const CylinderWithPoints& c1 : a) {
        for (const CylinderWithPoints& c2 : b) {
            if (cylinders_similar(c1.cylinder, c2.cylinder, 0.9, 0.9, 0.9)) {
                result.push_back(merge_cylinders(c1, c2));
            }
        }
    }

    return result;
}

void compute_differential_quantities(cgal::Point_3* cloud, Curvature* q_out, size_t count, size_t k, size_t d, size_t dprime)
{
    KdTree kdtree;
    kdtree.insert(cloud, cloud + count);
    kdtree.build<CGAL::Parallel_tag>();

    tbb::parallel_for((size_t)0, count, [&](size_t i) {
        KNeighbour knn(kdtree, cloud[i], k);
        std::vector<cgal::Point_3> nn(k+1);
        nn[0] = cloud[i];

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
