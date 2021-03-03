#include "particles.hpp"
#include <gfx/imgui/imgui.h>


ParticleSim::ParticleSim()
    : file_browser(ImGuiFileBrowserFlags_CloseOnEsc)
{
    file_browser.SetTitle("PLY open");
    file_browser.SetTypeFilters({".ply"});
}

void ParticleSim::draw_gui()
{
    switch (state) {
        case Visualization:
            draw_gui_view();
            break;
        case Configuration:
            draw_gui_config();
            break;
        default:
            break;
    }
}

void ParticleSim::draw_gui_config()
{
    ImGui::OpenPopup("Particle simulation");
    if (ImGui::BeginPopup("Particle simulation")) {
        if (ImGui::Button("Select file...")) {
        }
    }
}
