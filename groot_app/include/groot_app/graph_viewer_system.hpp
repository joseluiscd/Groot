#pragma once

#include <groot_app/groot_app.hpp>
#include <groot_app/viewer_system.hpp>

namespace graph_viewer_system {

GROOT_APP_LOCAL void init(entt::registry& registry);
GROOT_APP_LOCAL void deinit(entt::registry& registry);
GROOT_APP_LOCAL void run(entt::registry& registry);

}