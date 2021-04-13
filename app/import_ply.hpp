#pragma once

#include <gfx/imgui/imfilebrowser.h>
#include <gfx/imgui/imgui.h>
#include <entt/entt.hpp>
#include "command_gui.hpp"
#include <optional>
#include <groot/cgal.hpp>

class ImportPLY : public CommandGui {
public:
    ImportPLY(entt::registry& registry);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish() override;

private:
    entt::registry& registry;

    ImGui::FileBrowser open;
    std::string input_file = "";

    std::vector<groot::Point_3> cloud;
};