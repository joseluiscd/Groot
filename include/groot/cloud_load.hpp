#pragma once

#include <groot/groot.hpp>
#include <vector>
#include <groot/cgal.hpp>


namespace groot {

struct GROOT_API CloudData {
    std::vector<cgal::Point_3> points;
    std::optional<std::vector<cgal::Vector_3>> normals;
    std::optional<std::vector<cgal::Vector_3>> colors;
};

GROOT_API CloudData load_PLY(const char* filename);
GROOT_API void save_PLY(const char* filename, Point_3* cloud, size_t size);
GROOT_API void save_PLY(const char* filename, Point_3* cloud, Vector_3* normals, size_t size);

}
