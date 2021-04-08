#include "viewer_system.hpp"
#include "entity_editor.hpp"
#include "resources.hpp"
#include "components.hpp"
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imgui.h>

namespace viewer_system {

void run(entt::registry& registry)
{
    ImGui::SetNextWindowSize(ImVec2(200, 200), ImGuiCond_FirstUseEver);

    SystemData& data = registry.ctx<SystemData>();

    ImGui::BeginFramebuffer("3D Viewer", data.framebuffer);
    glm::ivec2 current_size = ImGui::GetWindowContentSize();
    if (current_size != data.size) {
        data.size = current_size;
        data.camera->lens().resize_event(current_size);
    }

    if (ImGui::IsItemActive()) {
        glm::vec2 drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, 0.0);
        data.camera->orbit(drag.x * 0.01);
        data.camera->orbit_vertical(drag.y * 0.01);
        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);

        drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right, 0.0);
        data.camera->truck(-drag.x * 0.01);
        data.camera->crane(drag.y * 0.01);
        ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
    }

    if (ImGui::IsItemHovered()) {
        data.lens->zoom(1.0 + ImGui::GetIO().MouseWheel * 0.001);
    }

    ImGui::EndFramebuffer();
}

void init(entt::registry& registry)
{
    auto lens = std::make_unique<gfx::PerspectiveCameraLens>(70.0, 1.0, 0.2, 100.2);
    auto camera = std::make_unique<gfx::CameraRig>(*lens);

    SystemData data {
        gfx::Framebuffer(),
        std::move(camera),
        std::move(lens),
        glm::ivec2(0, 0)
    };

    data.framebuffer
        .add_color_buffer(glm::ivec2(2048, 2048), gfx::TextureType::Rgba, 4)
        .set_depth_buffer(glm::ivec2(2048, 2048), 4);

    data.camera
        ->with_position({ 0.0, 0.0, 20.0 })
        .look_at({ 0.0, 0.0, 0.0 })
        .with_up_vector({ 0.0, 1.0, 0.0 });

    registry.set<SystemData>(std::move(data));

    registry.ctx<EntityEditor>().registerComponent<Visible>("Visible");
}

}