#pragma once

#include "command_gui.hpp"
#include "lua.hpp"
#include <entt/entt.hpp>
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>
#include <sol/table.hpp>

class OpenWorkspace : public CommandGui {
public:
    OpenWorkspace(entt::registry& registry);
    CommandState execute() override;
    GuiState draw_gui() override;

    OpenWorkspace& set_file(const std::filesystem::path& path)
    {
        selected_file = path;
        return *this;
    }


private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;

    entt::registry& reg;
};
