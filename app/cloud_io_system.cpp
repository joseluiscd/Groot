#include "cloud_io_system.hpp"
#include "components.hpp"
#include <CGAL/IO/write_ply_points.h>
#include <fstream>
#include <gfx/imgui/imfilebrowser.h>
#include <groot/cloud_load.hpp>
#include <spdlog/spdlog.h>

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

bait::GuiOperation ImportPLY::draw_custom_gui(Cmd& cmd, GuiState& state)
{
    state.open.Display();

    if (state.open.HasSelected()) {
        cmd.input_file = state.open.GetSelected().string();
        state.open.ClearSelected();
        return bait::GuiOperation::Run;
    } else if (!state.open.IsOpened()) {
        return bait::GuiOperation::Close;
    } else {
        return bait::GuiOperation::Nop;
    }
}

ImportPLYResult ImportPLY::update_async(Cmd&& cmd)
{
    spdlog::info("Loading PLY file...");

    if (cmd.input_file.empty()) {
        return Result {};
    }

    auto&& data = groot::load_PLY(cmd.input_file.c_str());

    spdlog::info("Loaded PLY file with {} points!", data.points.size());
    return Result { std::move(cmd.input_file), std::move(data.points), std::move(data.normals), std::move(data.colors) };
}

void ImportPLY::update_sync(entt::handle h, Result&& cloud)
{
    entt::registry& registry = *h.registry();
    
    if (cloud.cloud.empty()) {
        return;
    }

    entt::entity entity = registry.create();
    registry.emplace<PointCloud>(entity, std::move(cloud.cloud));
    registry.emplace<Name>(entity, cloud.name);

    if (cloud.normals) {
        registry.emplace<PointNormals>(entity, std::move(*cloud.normals));
    }
    if (cloud.colors) {
        registry.emplace<PointColors>(entity, std::move(*cloud.colors));
    }
}

/*
ExportPLY::ExportPLY(entt::handle&& handle)
    : registry(*handle.registry())
    , save(save_flags)
{
    save.SetTitle("Open PLY Cloud");
    save.SetTypeFilters({ ".ply" });
    save.Open();
    target = handle.entity();

    if (registry.valid(target) && registry.all_of<PointCloud>(target)) {
        this->cloud = &registry.get<PointCloud>(target);
    } else {
        throw std::runtime_error("Selected entity must have PointCloud");
    }

    if (handle.all_of<PointNormals>()) {
        normals = &handle.get<PointNormals>();
    }
}

GuiState ExportPLY::draw_gui()
{
    save.Display();

    if (save.HasSelected()) {
        output_file = save.GetSelected().string();
        save.ClearSelected();
        return GuiState::RunAsync;
    } else if (!save.IsOpened()) {
        return GuiState::Close;
    } else {
        return GuiState::Editing;
    }
}

CommandState ExportPLY::execute()
{
    if (normals) {
        groot::save_PLY(output_file.c_str(), cloud->cloud.data(), (*normals)->normals.data(), cloud->cloud.size());
    } else {
        groot::save_PLY(output_file.c_str(), cloud->cloud.data(), cloud->cloud.size());
    }
    return CommandState::Ok;
}
*/