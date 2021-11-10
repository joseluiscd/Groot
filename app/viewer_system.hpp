#pragma once

#include <entt/entt.hpp>
#include <bait/system.hpp>
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>

struct ViewerSystem : public bait::System<ViewerSystem> {
    void init(entt::registry& registry);
    void clear(entt::registry& registry);
    void update(entt::registry& registry);
};