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
    : file_dialog()
{
    file_dialog.SetTitle("Open Workspace");
    file_dialog.SetTypeFilters({ ".groot_workspace" });
    file_dialog.Open();
}

GuiResult OpenWorkspace::draw_gui()
{
    file_dialog.Display();

    if (file_dialog.HasSelected()) {
        selected_file = file_dialog.GetSelected();
        file_dialog.ClearSelected();
        return GuiResult::RunAndClose;
    } else if (!file_dialog.IsOpened()) {
        return GuiResult::Close;
    } else {
        return GuiResult::KeepOpen;
    }
}

void OpenWorkspace::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskBroker>().push_task(
        "Opening workspace",
        open_workspace_command(reg, selected_file));
}

SaveWorkspace::SaveWorkspace()
    : file_dialog(ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename)
{
    file_dialog.SetTitle("Graph Save");
    file_dialog.SetTypeFilters({ ".groot_workspace" });
    file_dialog.Open();
}

GuiResult SaveWorkspace::draw_gui()
{
    if (file_dialog.HasSelected()) {
        selected_file = file_dialog.GetSelected();
        file_dialog.ClearSelected();
        return GuiResult::RunAndClose;
    } else if (!file_dialog.IsOpened()) {
        return GuiResult::Close;
    } else {
        return GuiResult::KeepOpen;
    }
}

void SaveWorkspace::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskBroker>().push_task(
        "Saving workspace",
        save_workspace_command(reg, selected_file));
}
