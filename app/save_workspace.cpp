#include "save_workspace.hpp"
#include "components.hpp"
#include "serde.hpp"
#include <fstream>
#include <spdlog/spdlog.h>

SaveWorkspace::SaveWorkspace(entt::registry& _reg)
    : file_dialog(ImGuiFileBrowserFlags_CloseOnEsc | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename)
    , reg(_reg)
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
            Cylinders>(out_archive);

    spdlog::info("Exported to file: {}", selected_file.string());

    return CommandState::Ok;
}