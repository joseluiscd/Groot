#pragma once

#include <groot_app/entt.hpp>
#include <fmt/format.h>
#include <groot/cgal.hpp>
#include <groot_graph/cylinder_marching.hpp>
#include <string>
#include <groot_graph/plant_graph.hpp>
#include <groot/disjoint_sets.hpp>

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

struct PlantGraphNodePoints {
    groot::PropertyMap<std::vector<groot::Point_3>> points;
};

struct ConnectedComponents {
    groot::DisjointSets components;
};

//TODO: Distances to root
struct GraphDistanceToRoot {
    std::vector<float> distance;
    float max_distance;
};

//TODO: Root vertex
struct RootVertex {
    groot::Vertex root;
};

void init_components(entt::registry& reg);

// Resources

struct SelectedEntity {
    entt::entity selected;
};

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
decltype(auto) require_components(entt::handle h)
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
