#pragma once

#include <groot/groot.hpp>
#include <groot/cgal.hpp>

namespace groot {

struct GROOT_API VoxelGrid {
    size_t x_size, y_size, z_size;

    std::vector<std::vector<size_t>> voxels;

    VoxelGrid(size_t x, size_t y, size_t z)
        : x_size(x)
        , y_size(y)
        , z_size(z)
        , voxels(x * y * z)
    {
    }

    std::vector<size_t>& operator()(size_t x, size_t y, size_t z)
    {
        assert(x < x_size);
        assert(y < y_size);
        assert(z < z_size);

        return voxels[z * x_size * y_size + y * x_size];
    }

    const std::vector<size_t>& operator()(size_t x, size_t y, size_t z) const
    {
        assert(x < x_size);
        assert(y < y_size);
        assert(z < z_size);

        return voxels[z * x_size * y_size + y * x_size];
    }
};

template <auto Aggregator>
void recenter_cloud(Point_3* cloud, size_t count)
{
    Vector_3 displacement(CGAL::ORIGIN, Aggregator(cloud, cloud + count));

    for (size_t i = 0; i < count; i++) {
        cloud[i] -= displacement;
    }
}

inline Point_3 bounding_box_center(Point_3* first, Point_3* last)
{
    Bbox_3 box = CGAL::bbox_3(first, last);
    return Point_3((box.xmin() + box.xmax()) * 0.5, (box.ymin() + box.ymax()) * 0.5, (box.zmin() + box.zmax()) * 0.5);
}

inline Point_3 centroid(Point_3* first, Point_3* last)
{
    Vector_3 centroid(0.0, 0.0, 0.0);
    size_t count = 0;

    for (Point_3* i = first; i != last; i++) {
        centroid += Vector_3(CGAL::ORIGIN, *i);
        count++;
    }

    centroid /= (float)count;
    return Point_3(CGAL::ORIGIN) + centroid;
}

GROOT_API void normal_filter(Point_3* cloud, Vector_3* normals, size_t count, float radius, float cos_threshold, float percent_threshold);

GROOT_API VoxelGrid voxel_grid(Point_3* cloud, size_t count, float size);

constexpr void (*recenter_cloud_bounding_box)(Point_3*, size_t) = &recenter_cloud<&bounding_box_center>;
constexpr void (*recenter_cloud_centroid)(Point_3*, size_t) = &recenter_cloud<&centroid>;

}