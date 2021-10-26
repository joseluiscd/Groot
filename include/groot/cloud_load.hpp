#pragma once

#include <vector>
#include <groot/cgal.hpp>


namespace groot {

struct CloudData {
    std::vector<cgal::Point_3> points;
    std::optional<std::vector<cgal::Vector_3>> normals;
    std::optional<std::vector<cgal::Vector_3>> colors;
};

CloudData load_PLY(const char* filename);
void save_PLY(const char* filename, Point_3* cloud, size_t size);
void save_PLY(const char* filename, Point_3* cloud, Vector_3* normals, size_t size);

}
