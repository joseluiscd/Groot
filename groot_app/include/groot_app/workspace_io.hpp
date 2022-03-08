#pragma once

#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>
#include <groot_app/command_gui.hpp>
#include <groot_app/entt.hpp>

GROOT_APP_API async::task<void> open_workspace_command(entt::registry& reg, const std::string& filename);
GROOT_APP_API async::task<void> save_workspace_command(const entt::registry& reg, const std::string& filename);

class GROOT_APP_LOCAL OpenWorkspace : public Gui {
public:
    OpenWorkspace();

    void schedule_commands(entt::registry& reg) override;
    GuiResult draw_gui() override;

private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;
};

class GROOT_APP_LOCAL SaveWorkspace : public Gui {
public:
    SaveWorkspace();

    void schedule_commands(entt::registry& reg) override;
    GuiResult draw_gui() override;

private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;
};
