#pragma once

#include <string>
#include <entt/entt.hpp>
#include <groot/cgal.hpp>
#include <groot/cylinder_marching.hpp>

struct Name{
    std::string name;
};

/// The item is visible (renderable)
struct Visible{};

struct PointCloud {
    std::vector<groot::Point_3> cloud;
};

struct PointNormals {
    std::vector<groot::Vector_3> normals;
};

struct Cylinders {
    std::vector<groot::Cylinder> cylinders;
};

void init_components(entt::registry& reg);

// Resources

struct SelectedEntity {
    entt::entity selected;
};
