#include <groot_app/save_workspace.hpp>
#include <groot_app/components.hpp>
#include <groot_app/serde.hpp>
#include <groot_graph/plant_graph.hpp>
#include <fstream>
#include <spdlog/spdlog.h>

SaveWorkspace::SaveWorkspace(entt::registry& _reg)
    : reg(_reg)
    , file_dialog(ImGuiFileBrowserFlags_CloseOnEsc | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename)
{
    file_dialog.SetTitle("Graph Save");
    file_dialog.SetTypeFilters({ ".ggf" });
    file_dialog.Open();
}

GuiState SaveWorkspace::draw_gui()
{
    file_dialog.Display();

    if (file_dialog.HasSelected()) {
        selected_file = file_dialog.GetSelected();
        file_dialog.ClearSelected();
        return GuiState::RunAsync;
    }

    return GuiState::Editing;
}

CommandState SaveWorkspace::execute()
{
    std::ofstream file(selected_file, std::ios::binary);
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

    spdlog::info("Exported to file: {}", selected_file.string());

    return CommandState::Ok;
}