#pragma once

#include <groot/cgal.hpp>

namespace groot {

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

    centroid /= (float) count;
    return Point_3(CGAL::ORIGIN) + centroid;
}

constexpr void (*recenter_cloud_bounding_box)(Point_3*, size_t) = &recenter_cloud<&bounding_box_center>;
constexpr void (*recenter_cloud_centroid)(Point_3*, size_t) = &recenter_cloud<&centroid>;

}