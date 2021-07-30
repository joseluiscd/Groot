#include "open_workspace.hpp"
#include <fstream>
#include "serde.hpp"
#include <groot/plant_graph.hpp>
#include <spdlog/spdlog.h>

OpenWorkspace::OpenWorkspace(entt::registry& _reg)
    : reg(_reg)
    , file_dialog()
{
    file_dialog.SetTitle("Graph Open");
    file_dialog.SetTypeFilters({ ".ggf" });
    file_dialog.Open();
}

GuiState OpenWorkspace::draw_gui()
{
    file_dialog.Display();

    if (file_dialog.HasSelected()) {
        selected_file = file_dialog.GetSelected();
        file_dialog.ClearSelected();
        return GuiState::RunSync;
    }

    return GuiState::Editing;
}

CommandState OpenWorkspace::execute()
{
    std::ifstream file(selected_file, std::ios::binary);
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
            

    return CommandState::Ok;
}