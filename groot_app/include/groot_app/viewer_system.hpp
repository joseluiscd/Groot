#pragma once

#include <groot/groot.hpp>
#include <groot_app/entt.hpp>
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>

/**
 * 
*/
namespace viewer_system {

GROOT_LOCAL void init(entt::registry& registry);
GROOT_LOCAL void deinit(entt::registry& registry);
GROOT_LOCAL void run(entt::registry& registry);

}