#pragma once

#include <glm/glm.hpp>

namespace groot {

struct DifferentialQuantities {
    glm::vec3 direction;
    glm::vec3 curvature_center;
    float radius;
};

void find_cylinders(
    glm::vec3* cloud,
    size_t count);

void compute_differential_quantities(glm::vec3* cloud, DifferentialQuantities* q_out, size_t count);

}
