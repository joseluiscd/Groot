#include "viewer.hpp"

AbstractViewer::AbstractViewer()
    : open(true)
    , camera_lens(70.0, 1.0, 0.2, 100.2)
    , camera_rig(camera_lens)
    , size(0, 0)
{
    framebuffer
        .add_color_buffer(glm::ivec2(2048, 2048), gfx::TextureType::Rgba, 4)
        .set_depth_buffer(glm::ivec2(2048, 2048), 4);

    camera_rig
        .with_position({ 0.0, 0.0, 20.0 })
        .look_at({ 0.0, 0.0, 0.0 })
        .with_up_vector({ 0.0, 1.0, 0.0 });
}

bool AbstractViewer::draw_gui()
{
    ImGui::SetNextWindowSize(ImVec2(200, 200), ImGuiCond_FirstUseEver);

    std::string title = fmt::format("3D render##{}", (size_t) this);
    if (ImGui::BeginFramebuffer(title.c_str(), framebuffer, &open)) {
        glm::ivec2 current_size = ImGui::GetWindowContentSize();
        if (current_size != size) {
            size = current_size;
            camera_lens.resize_event(size);
        }

        if (ImGui::IsItemActive()) {
            glm::vec2 drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, 0.0);
            camera_rig.orbit(drag.x * 0.01);
            camera_rig.orbit_vertical(drag.y * 0.01);
            ImGui::ResetMouseDragDelta(ImGuiMouseButton_Left);

            drag = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right, 0.0);
            camera_rig.truck(-drag.x * 0.01);
            camera_rig.crane(drag.y * 0.01);
            ImGui::ResetMouseDragDelta(ImGuiMouseButton_Right);
        }

        if (ImGui::IsItemHovered()) {
            camera_lens.zoom(1.0 + ImGui::GetIO().MouseWheel * 0.001);
        }

        this->render();

        ImGui::EndFramebuffer();
    }

    return open;
}