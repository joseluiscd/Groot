#pragma once

#include <entt/entt.hpp>
#include <fmt/format.h>
#include <groot/cgal.hpp>
#include <groot/cylinder_marching.hpp>
#include <bait/bait.hpp>
#include <string>

struct Name {
    std::string name;
};

/// The item is visible (renderable)
struct Visible {
};

struct PointCloud {
    std::vector<groot::Point_3> cloud;
};

struct PointNormals {
    std::vector<groot::Vector_3> normals;
};

struct PointColors {
    std::vector<groot::Vector_3> colors;
};

struct PointCurvature {
    struct Curvature {
        groot::Vector_3 direction;
        float radius;
    };
    std::vector<Curvature> direction;
};

struct Cylinders {
    std::vector<groot::CylinderWithPoints> cylinders;
};

struct AABB {
    groot::cgal::Bbox_3 box;
};

struct SelectedEditEntity {
    entt::entity selected;
};

using Selected = bait::Selected;

void init_components(entt::registry& reg);


template <typename Component, typename... Components>
std::string get_component_name_list()
{
    if constexpr (sizeof...(Components) == 0 ) {
        return typeid(Component).name();
    } else {
        return fmt::format("{}, {}", get_component_name_list<Component>(), get_component_name_list<Components...>());
    }
}


// Returns pointer or tuple of pointers to requested components.
// The pointers are safe to dereference if the components are not modified/removed.
template <typename... Components>
decltype(auto) require_components(entt::handle& h)
{
    entt::registry& registry = *h.registry();
    entt::entity target = h.entity();

    if (!registry.valid(target)) {
        throw std::runtime_error("Need selected entity");
    }

    if (registry.all_of<Components...>(target)) {
        return registry.try_get<Components...>(target);
    } else {
        throw std::runtime_error(
            fmt::format("Selected entity is missing a component. Required: {}",
                get_component_name_list<Components...>()));
    }
}
