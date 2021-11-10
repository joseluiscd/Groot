#pragma once

#include <bait/gui_system.hpp>
#include <gfx/imgui/imfilebrowser.h>
#include <gfx/imgui/imgui.h>
#include <entt/entt.hpp>
#include <optional>
#include <groot/cgal.hpp>
#include "components.hpp"

struct ImportPLY;


// Import PLY
// --------------

struct ImportPLYCmd {
    std::string input_file = "";
};

struct ImportPLYResult {
    std::string name;
    std::vector<groot::Point_3> cloud;
    std::optional<std::vector<groot::Vector_3>> normals;
    std::optional<std::vector<groot::Vector_3>> colors;
};

template <>
struct bait::SystemTraits<ImportPLY> : bait::DefaultSystemTraits {
    using Cmd = ImportPLYCmd;
    using Result = ImportPLYResult;

    struct GuiState {
        GuiState()
            : open()
        {
            open.SetTitle("Open PLY Cloud");
            open.SetTypeFilters({ ".ply" });
            open.Open();
        }

        ImGui::FileBrowser open;
    };
};

struct ImportPLY : public bait::CustomGuiSystemImpl<bait::SystemImpl<bait::System<ImportPLY>>> {
    static Result update_async(const Cmd& cmd);
    static void update_sync(entt::handle h, Result&& cloud);
    static bait::GuiOperation draw_custom_gui(Cmd& cmd, GuiState& state);
};

/*
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
};*/

using CloudIOSystem = bait::SystemCollection<ImportPLY>;