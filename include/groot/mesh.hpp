#pragma once

#include <vector>
#include <groot/cgal.hpp>

namespace groot {

struct Mesh {
    std::vector<Point_3> vertices;
    std::vector<unsigned int> indices;
    std::optional<std::vector<Vector_3>> normals;
};

}