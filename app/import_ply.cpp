#include "import_ply.hpp"
#include "components.hpp"
#include <fstream>
#include <gfx/imgui/imfilebrowser.h>
#include <groot/cloud_load.hpp>
#include <spdlog/spdlog.h>
#include <CGAL/IO/write_ply_points.h>

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

ImportPLY::ImportPLY(entt::registry& _registry)
    : registry(_registry)
    , open(open_flags)
{
    open.SetTitle("Open PLY Cloud");
    open.SetTypeFilters({ ".ply" });
    open.Open();
}

GuiState ImportPLY::draw_gui()
{
    open.Display();

    if (open.HasSelected()) {
        input_file = open.GetSelected().string();
        open.ClearSelected();
        return GuiState::RunAsync;
    } else if (! open.IsOpened()) {
        return GuiState::Close;
    } else {
        return GuiState::Editing;
    }
}

CommandState ImportPLY::execute()
{
    spdlog::info("Loading PLY file...");

    if (input_file.empty()) {
        error_string = "Cannot open file";
        return CommandState::Error;
    }
    auto&& ret = groot::load_PLY(input_file.c_str());

    cloud = std::move(ret.first);
    normals = std::move(ret.second);

    spdlog::info("Loaded PLY file with {} points!", cloud.size());
    return CommandState::Ok;
}

void ImportPLY::on_finish()
{
    auto entity = registry.create();
    registry.emplace<PointCloud>(entity, std::move(cloud));
    registry.emplace<Name>(entity, this->input_file);

    if (! normals.empty()) {
        registry.emplace<PointNormals>(entity, std::move(normals));
    }

    result = entity;
}

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
    } else if (! save.IsOpened()) {
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

