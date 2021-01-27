#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace groot {

pcl::PointCloud<pcl::PointXYZ>::Ptr
load_PLY(
    const std::string& filename,
    bool streaming = false);

}