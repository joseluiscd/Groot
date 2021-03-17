#include "particle_sim.hpp"
#include <gfx/imgui/imgui.h>

ParticleSim::ParticleSim()
    : file_browser(ImGuiFileBrowserFlags_CloseOnEsc)
{
    file_browser.SetTitle("PLY open");
    file_browser.SetTypeFilters({ ".ply" });
}

void ParticleSim::draw_gui()
{
    if (config) {
        draw_gui_config();
    }

    if (visualize) {
        draw_gui_view();
    }
}

void ParticleSim::draw_gui_config()
{
    if (ImGui::Begin("Particle simulation")) {
        if (file_browser.HasSelected()) {
            selected_file = file_browser.GetSelected();
            file_browser.ClearSelected();
        }

        if (ImGui::Button("Select file...")) {
            file_browser.Open();
        }

        ImGui::SameLine();

        ImGui::Text("%s", selected_file.c_str());

        ImGui::Separator();

        ImGui::InputInt("K", &k, 1, 5);
        ImGui::Separator();

        if(ImGui::Button("Update view")) {
            run_gui_config();
        }
    }

    ImGui::End();
}

void ParticleSim::run_gui_config()
{
    this->visualize = true;
}
