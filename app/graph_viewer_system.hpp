#pragma once

#include "viewer_system.hpp"

namespace graph_viewer_system {

void init(entt::registry& registry);
void deinit(entt::registry& registry);
void run(entt::registry& registry);

}