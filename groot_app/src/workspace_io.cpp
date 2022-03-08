#include <fstream>
#include <groot_app/serde.hpp>
#include <groot_app/workspace_io.hpp>
#include <groot_graph/plant_graph.hpp>
#include <spdlog/spdlog.h>

async::task<void> open_workspace_command(entt::registry& reg, const std::string& filename)
{
    return create_task().then_sync(
        [&reg, filename]() {
            std::ifstream file(filename, std::ios::binary);
            Deserializer in_archive(file);
            reg.clear();

            entt::snapshot_loader { reg }
                .entities(in_archive)
                .component<
                    Name,
                    Visible,
                    PointCloud,
                    PointNormals,
                    Cylinders,
                    groot::PlantGraph>(in_archive);
        });
}

async::task<void> save_workspace_command(const entt::registry& reg, const std::string& filename)
{
    return create_task().then_async([&reg, filename = std::move(filename)]() {
        std::ofstream file(filename, std::ios::binary);
        Serializer out_archive(file);

        entt::snapshot { reg }
            .entities(out_archive)
            .component<
                Name,
                Visible,
                PointCloud,
                PointNormals,
                Cylinders,
                groot::PlantGraph>(out_archive);

        spdlog::info("Exported to file: {}", filename);
    });
}

OpenWorkspace::OpenWorkspace()
    : FileDialogGui(FileDialogType::Open, "Open Workspace", { ".groot_workspace"} )
{
}

void OpenWorkspace::schedule_commands(entt::registry& reg, const std::string& filename)
{
    reg.ctx<TaskBroker>().push_task(
        "Opening workspace",
        open_workspace_command(reg, filename));
}

SaveWorkspace::SaveWorkspace()
    : FileDialogGui(FileDialogType::Save, "Save Workspace", { ".groot_workspace"} )
{
}

void SaveWorkspace::schedule_commands(entt::registry& reg, const std::string& filename)
{
    reg.ctx<TaskBroker>().push_task(
        "Saving workspace",
        save_workspace_command(reg, filename));
}
