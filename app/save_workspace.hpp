#pragma once

#include "application.hpp"
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>
#include <entt/entt.hpp>
#include "serde.hpp"

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