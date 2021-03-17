#pragma once

#include <vector>
#include <groot/cgal.hpp>


namespace groot {

std::vector<cgal::Point_3> load_PLY(const char* filename);

}
