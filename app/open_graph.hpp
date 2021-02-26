#pragma once

#include "application.hpp"
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>
#include "data_output.hpp"

class OpenGraph : public CommandGui {
public:
    OpenGraph(IDataOutput<groot::PlantGraph>& _output);
    CommandState execute() override;
    GuiState draw_gui() override;
private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;

    IDataOutput<groot::PlantGraph>& output;
};