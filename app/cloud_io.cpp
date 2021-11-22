#include "cloud_io.hpp"
#include "components.hpp"
#include <gfx/imgui/imfilebrowser.h>
#include <groot/cloud_load.hpp>
#include <spdlog/spdlog.h>

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

ImportPLYGui::ImportPLYGui()
    : open(open_flags)
{
    open.SetTitle("Open PLY Cloud");
    open.SetTypeFilters({ ".ply" });
    open.Open();
}

void ImportPLYGui::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskBroker>()
        .push_task(
            "Loading PLY",
            import_ply_command(reg, this->input_file));
}

GuiResult ImportPLYGui::draw_gui()
{
    open.Display();

    if (open.HasSelected()) {
        input_file = open.GetSelected().string();
        open.ClearSelected();
        return GuiResult::RunAndClose;
    } else if (!open.IsOpened()) {
        return GuiResult::Close;
    } else {
        return GuiResult::KeepOpen;
    }
}

async::task<entt::entity> import_ply_command(entt::registry& reg, const std::string_view& file)
{
    return async::spawn(async_scheduler(), [input_file = std::string(file)]() {
        spdlog::info("Loading PLY file...");

        if (input_file.empty()) {
            throw std::runtime_error("Cannot open file");
        }

        groot::CloudData&& data = groot::load_PLY(input_file.c_str());
        spdlog::info("Loaded PLY file with {} points!", data.points.size());
        return data;
    }).then(sync_scheduler(), [&reg, input_file = std::string(file)](groot::CloudData&& data) {
        entt::entity entity = reg.create();
        reg.emplace<PointCloud>(entity, std::move(data.points));
        reg.emplace<Name>(entity, input_file);

        if (data.normals) {
            reg.emplace<PointNormals>(entity, std::move(*data.normals));
        }
        if (data.colors) {
            reg.emplace<PointColors>(entity, std::move(*data.colors));
        }

        return entity;
    });
}

ExportPLYGui::ExportPLYGui(entt::handle handle)
    : save(save_flags)
    , target(handle)
{
    save.SetTitle("Open PLY Cloud");
    save.SetTypeFilters({ ".ply" });
    save.Open();

    if (!target.valid() || !target.all_of<PointCloud>()) {
        throw std::runtime_error("Selected entity must have PointCloud");
    }
}

GuiResult ExportPLYGui::draw_gui()
{
    save.Display();

    if (save.HasSelected()) {
        output_file = save.GetSelected().string();
        save.ClearSelected();
        return GuiResult::RunAndClose;
    } else if (!save.IsOpened()) {
        return GuiResult::Close;
    } else {
        return GuiResult::KeepOpen;
    }
}
void ExportPLYGui::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskBroker>()
        .push_task(
            "Saving PLY",
            export_ply_command(target, this->output_file));
}
async::task<void> export_ply_command(entt::handle e, const std::string_view& file)
{
    return async::spawn(sync_scheduler(), [e]() {
        if (!e.valid() || !e.all_of<PointCloud>()) {
            throw std::runtime_error("Selected entity must have PointCloud");
        }

        PointCloud* cloud = &e.get<PointCloud>();
        PointNormals* normals = e.try_get<PointNormals>();
        PointColors* colors = e.try_get<PointColors>();

        spdlog::info("Saving PLY...");

        return std::make_tuple(cloud, normals, colors);
    }).then(async_scheduler(), [file = std::string(file)](std::tuple<PointCloud*, PointNormals*, PointColors*>&& data) {
        auto&& [cloud, normals, colors] = data;

        if (normals) {
            spdlog::info("PLY Saving with normals...");
            groot::save_PLY(file.c_str(), cloud->cloud.data(), normals->normals.data(), cloud->cloud.size());
        } else {
            spdlog::info("PLY Saving without normals...");
            groot::save_PLY(file.c_str(), cloud->cloud.data(), cloud->cloud.size());
        }
        spdlog::info("Finished saving PLY!");
    });
}
