#pragma once

#include <entt/entt.hpp>
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>

/**
 * 
*/
namespace viewer_system {


void init(entt::registry& registry);
void run(entt::registry& registry);

}