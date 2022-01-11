#pragma once

#include <groot/groot.hpp>
#include <vector>
#include <glm/glm.hpp>

struct AABB {
    glm::vec3 min;
    glm::vec3 max;

    //bool ray_cast(const AGRay3D &r, AGVector3D *result = 0);
};

template <typename T>
class GROOT_API Grid {
public:
    Grid(const AABB& _bounds, size_t x, size_t y, size_t z, const T& default_value = T())
        : bounds(_bounds)
        , size_x(x)
        , size_y(y)
        , size_z(z)
        , data(x * y * z, default_value)
    {
    }

    T& operator()(size_t x, size_t y, size_t z)
    {
        return data[x + size_x * y + size_x * size_y * z];
    }

    const T& operator()(size_t x, size_t y, size_t z) const
    {
        return data[x + size_x * y + size_x * size_y * z];
    }

    T& operator()(const glm::vec3& p);

private:
    AABB bounds;
    size_t size_x;
    size_t size_y;
    size_t size_z;

    std::vector<T> data;
};

template <typename T>
T& Grid<T>::operator()(const glm::vec3& p)
{
    glm::vec3 base = p - bounds.min;
    glm::vec3 size = bounds.max - bounds.min;

}

