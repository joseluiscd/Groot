#include "save_graph.hpp"
#include <spdlog/spdlog.h>
#include <fstream>

SaveGraph::SaveGraph(IDataSource<groot::PlantGraph>& _input)
    : input(_input)
    , file_dialog(ImGuiFileBrowserFlags_CloseOnEsc | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename)
{
    file_dialog.SetTitle("Graph Save");
    file_dialog.SetTypeFilters({ ".ggf" });
    file_dialog.Open();
}

GuiState SaveGraph::draw_gui()
{
    file_dialog.Display();

    if (file_dialog.HasSelected()) {
        selected_file = file_dialog.GetSelected();
        file_dialog.ClearSelected();
        return GuiState::RunAsync;
    }

    return GuiState::Editing;
}

CommandState SaveGraph::execute()
{
    std::ofstream file(selected_file, std::ios::binary);
    groot::write_to_file(*input, file);
    spdlog::info("Writed output file...");
    return CommandState::Ok;
}