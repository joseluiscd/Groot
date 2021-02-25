#pragma once

#include "application.hpp"
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>

class OpenGraph : public CommandGui {
public:
    OpenGraph();
    CommandState execute() override;
    GuiState draw_gui() override;
private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;
};