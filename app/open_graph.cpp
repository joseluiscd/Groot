#include "open_graph.hpp"
#include <fstream>

OpenGraph::OpenGraph()
    : file_dialog()
{
    file_dialog.SetTitle("Graph Open");
    file_dialog.SetTypeFilters({ ".ggf" });
    file_dialog.Open();
}

GuiState OpenGraph::draw_gui()
{
    file_dialog.Display();
    
    if (file_dialog.HasSelected()) {
        selected_file = file_dialog.GetSelected();
        file_dialog.ClearSelected();
        return GuiState::RunAsync;
    }

    return GuiState::Editing;
}

CommandState OpenGraph::execute()
{
    std::ifstream file(selected_file);
    app->push_plant_graph(groot::read_from_file(file));
    return CommandState::Ok;
}