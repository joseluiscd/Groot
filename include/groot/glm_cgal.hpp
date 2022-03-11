#pragma once

#include <groot/cgal.hpp>
#include <glm/glm.hpp>


namespace groot {

inline glm::vec3 to_glm(const Point_3& p)
{
    return glm::vec3(p.x(), p.y(), p.z());
}

inline glm::vec3 to_glm(const Vector_3& p)
{
    return glm::vec3(p.x(), p.y(), p.z());
}

}