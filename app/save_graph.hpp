#pragma once

#include "application.hpp"
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>
#include <entt/entt.hpp>

class SaveGraph : public CommandGui {
public:
    SaveGraph(entt::registry& reg);
    CommandState execute() override;
    GuiState draw_gui() override;

private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;

    std::vector<entt::entity> options;

};