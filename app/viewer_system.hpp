#pragma once

#include <entt/entt.hpp>
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>

/**
 * 
*/
namespace viewer_system {

struct SystemData {
    gfx::Framebuffer framebuffer;
    std::unique_ptr<gfx::CameraRig> camera;
    std::unique_ptr<gfx::PerspectiveCameraLens> lens;
    glm::ivec2 size;

    SystemData(const SystemData&) = delete;
    SystemData(SystemData&&) = default;
    SystemData& operator=(SystemData&&) = default;
    SystemData& operator=(const SystemData&) = delete;
};

void init(entt::registry& registry);
void run(entt::registry& registry);

}