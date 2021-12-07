#pragma once

#include <groot/groot.hpp>
#include <groot_app/viewer_system.hpp>

namespace graph_viewer_system {

GROOT_LOCAL void init(entt::registry& registry);
GROOT_LOCAL void deinit(entt::registry& registry);
GROOT_LOCAL void run(entt::registry& registry);

}