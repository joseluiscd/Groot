#pragma once

#include <vector>
#include <groot/cgal.hpp>


namespace groot {


std::pair<std::vector<cgal::Point_3>, std::vector<cgal::Vector_3>> load_PLY(const char* filename);
void save_PLY(const char* filename, Point_3* cloud, size_t size);
void save_PLY(const char* filename, Point_3* cloud, Vector_3* normals, size_t size);

}
