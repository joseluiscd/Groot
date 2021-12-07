#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/serde.hpp>
#include <groot_app/entt.hpp>
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>

class GROOT_API SaveWorkspace : public CommandGui {
public:
    SaveWorkspace(entt::registry& reg);
    CommandState execute() override;
    GuiState draw_gui() override;

    SaveWorkspace set_file(const std::filesystem::path& path)
    {
        selected_file = path;
        return *this;
    }

private:
    entt::registry& reg;

    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;
};