#pragma once

#include <string>
#include <entt/entt.hpp>

struct Name{
    std::string name;
};

/// The item is visible (renderable)
struct Visible{};

/// The item is selected
struct Selected{};

void init_components(entt::registry& reg);