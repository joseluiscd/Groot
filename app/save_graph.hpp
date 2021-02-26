#pragma once

#include "application.hpp"
#include "data_source.hpp"
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>

class SaveGraph : public CommandGui {
public:
    SaveGraph(IDataSource<groot::PlantGraph>& _input);
    CommandState execute() override;
    GuiState draw_gui() override;

private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;

    IDataSource<groot::PlantGraph>& input;
};