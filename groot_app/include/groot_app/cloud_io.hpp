#pragma once

#include <groot/groot.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <gfx/imgui/imfilebrowser.h>
#include <gfx/imgui/imgui.h>
#include <groot/cgal.hpp>
#include <optional>

GROOT_API async::task<entt::entity> import_ply_command(entt::registry& reg, const std::string_view& file);
GROOT_API async::task<void> export_ply_command(entt::handle e, const std::string_view& file);

class GROOT_LOCAL ImportPLYGui : public Gui {
public:
    ImportPLYGui();
    void schedule_commands(entt::registry& reg) override;
    GuiResult draw_gui() override;

private:
    ImGui::FileBrowser open;
    std::string input_file;
};

class GROOT_LOCAL ExportPLYGui : public Gui {
public:
    ExportPLYGui(entt::handle h);

    void schedule_commands(entt::registry& reg) override;
    GuiResult draw_gui() override;

private:
    ImGui::FileBrowser save;
    entt::handle target;
    std::string output_file = "";
};