#include <groot/cloud_load.hpp>
#include <groot_app/cloud_io.hpp>
#include <groot_app/components.hpp>
#include <spdlog/spdlog.h>

ImportPLYGui::ImportPLYGui()
    : FileDialogGui(FileDialogType::Open, "Open PLY Cloud", { ".ply" })
{
}

void ImportPLYGui::schedule_commands(entt::registry& reg, const std::string& filename)
{
    reg.ctx<TaskManager>()
        .push_task(
            "Loading PLY",
            import_ply_command(reg, filename));
}

async::task<entt::entity> import_ply_command(entt::registry& reg, const std::string_view& file)
{
    return create_task()
        .then_async([input_file = std::string(file)]() {
            spdlog::info("Loading PLY file...");

            if (input_file.empty()) {
                throw std::runtime_error("Cannot open file");
            }

            groot::CloudData data = groot::load_PLY(input_file.c_str());
            spdlog::info("Loaded PLY file with {} points!", data.points.size());
            return data;
        })
        .then_sync([&reg, input_file = std::string(file)](groot::CloudData&& data) {
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
    : FileDialogGui(FileDialogType::Save, "Save PLY Cloud", { ".ply" })
{
    handle_require_components<PointCloud>(handle);
}

void ExportPLYGui::schedule_commands(entt::registry& reg, const std::string& filename)
{
    reg.ctx<TaskManager>()
        .push_task(
            "Saving PLY",
            export_ply_command(target, filename));
}

async::task<void> export_ply_command(entt::handle e, const std::string_view& file)
{
    return create_task()
        .then_sync([e]() {
            if (!e.valid() || !e.all_of<PointCloud>()) {
                throw std::runtime_error("Selected entity must have PointCloud");
            }

            PointCloud* cloud = &e.get<PointCloud>();
            PointNormals* normals = e.try_get<PointNormals>();
            PointColors* colors = e.try_get<PointColors>();

            spdlog::info("Saving PLY...");

            return std::make_tuple(cloud, normals, colors);
        })
        .then_async([file = std::string(file)](std::tuple<PointCloud*, PointNormals*, PointColors*>&& data) {
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
