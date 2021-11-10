#include <groot/cloud.hpp>

namespace groot {

void normal_filter(Point_3* cloud, Vector_3* normals, size_t count, float radius, float cos_threshold, float percent_threshold)
{

}

VoxelGrid voxel_grid(const Point_3* cloud, size_t count, float size)
{
    cgal::Bbox_3 box = CGAL::bbox_3(cloud, cloud+count);

    float x_length = box.xmax() - box.xmin();
    float y_length = box.ymax() - box.ymin();
    float z_length = box.zmax() - box.zmin();
 
    size_t x_size = size_t( ( x_length / size ) + 1.5 );
    size_t y_size = size_t( ( y_length / size ) + 1.5 );
    size_t z_size = size_t( ( z_length / size ) + 1.5 );

    VoxelGrid grid(x_size, y_size, z_size);

    cgal::Vector_3 origin(box.xmin(), box.ymin(), box.zmin());

    for (size_t i = 0; i < count; i++) {
        cgal::Point_3 offset = cloud[i] - origin;
        size_t x_cell = offset.x() / size;
        size_t y_cell = offset.y() / size;
        size_t z_cell = offset.z() / size;

        grid(x_cell, y_cell, z_cell).push_back(i);
    }

    return grid;
}

}