#pragma once

#include <groot_app/entt.hpp>
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>

/**
 * 
*/
namespace viewer_system {


void init(entt::registry& registry);
void deinit(entt::registry& registry);
void run(entt::registry& registry);

}