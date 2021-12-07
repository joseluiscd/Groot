#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/entt.hpp>
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>

class GROOT_API OpenWorkspace : public CommandGui {
public:
    OpenWorkspace();
    CommandState execute() override;
    GuiState draw_gui() override;
    void on_finish(entt::registry& reg) override;

    OpenWorkspace& set_file(const std::filesystem::path& path)
    {
        selected_file = path;
        return *this;
    }


private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;
};
