#pragma once

#include <glm/glm.hpp>

struct AABB {
    glm::vec3 min;
    glm::vec3 max;

    //bool ray_cast(const AGRay3D &r, AGVector3D *result = 0);
};
