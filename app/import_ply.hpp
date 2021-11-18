#pragma once

#include <gfx/imgui/imfilebrowser.h>
#include <gfx/imgui/imgui.h>
#include <entt/entt.hpp>
#include "command_gui.hpp"
#include <optional>
#include <groot/cgal.hpp>
#include "components.hpp"
#include "entt/entity/fwd.hpp"

class ImportPLY : public CommandGui {
public:
    ImportPLY(entt::registry& registry);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish(entt::registry& reg) override;

    entt::entity result;

private:
    entt::registry& registry;

    ImGui::FileBrowser open;

    std::vector<groot::Point_3> cloud;
    std::optional<std::vector<groot::Vector_3>> normals;
    std::optional<std::vector<groot::Vector_3>> colors;
    entt::entity target;
public:
    // Parameters
    std::string input_file = "";
};

class ExportPLY : public CommandGui {
public:
    ExportPLY(entt::handle&& handle);
    ExportPLY(entt::registry& _reg)
        : ExportPLY(entt::handle {
            _reg,
            _reg.ctx<SelectedEntity>().selected })
    {
    }
    GuiState draw_gui() override;
    CommandState execute() override;

    entt::entity result;

private:
    entt::registry& registry;
    entt::entity target;

    ImGui::FileBrowser save;
    PointCloud* cloud;
    std::optional<PointNormals*> normals;
    
public:
    // Parameters
    std::string output_file = "";
};