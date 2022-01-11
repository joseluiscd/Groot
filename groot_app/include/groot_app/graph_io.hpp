#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <gfx/imgui/imfilebrowser.h>
#include <gfx/imgui/imgui.h>
#include <groot/cgal.hpp>
#include <optional>

GROOT_APP_API async::task<entt::entity> import_graph_command(entt::registry& reg, const std::string_view& file);
GROOT_APP_API async::task<void> export_graph_command(entt::handle e, const std::string_view& file);

class GROOT_APP_LOCAL ImportGraphGui : public Gui {
public:
    ImportGraphGui();
    void schedule_commands(entt::registry& reg) override;
    GuiResult draw_gui() override;

private:
    ImGui::FileBrowser open;
    std::string input_file;
};

class GROOT_APP_LOCAL ExportGraphGui : public Gui {
public:
    ExportGraphGui(entt::handle h);

    void schedule_commands(entt::registry& reg) override;
    GuiResult draw_gui() override;

private:
    ImGui::FileBrowser save;
    entt::handle target;
    std::string output_file = "";
};