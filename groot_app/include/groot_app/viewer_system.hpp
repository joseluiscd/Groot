#pragma once

#include <groot_app/groot_app.hpp>
#include <groot_app/entt.hpp>
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>

/**
 * 
*/
namespace viewer_system {

GROOT_APP_LOCAL void init(entt::registry& registry);
GROOT_APP_LOCAL void deinit(entt::registry& registry);
GROOT_APP_LOCAL void run(entt::registry& registry);

}