#pragma once

#include "command_gui.hpp"
#include "serde.hpp"
#include <entt/entt.hpp>
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>

class SaveWorkspace : public CommandGui {
public:
    SaveWorkspace(entt::registry& reg);
    CommandState execute() override;
    GuiState draw_gui() override;

private:
    entt::registry& reg;

    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;
};