#pragma once

#include "application.hpp"
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>
#include "data_output.hpp"
#include "lua.hpp"

class OpenGraph : public CommandGui {
public:
    OpenGraph(IDataOutput<groot::PlantGraph>& _output);
    CommandState execute() override;
    GuiState draw_gui() override;

    OpenGraph& set_file(const std::filesystem::path& path)
    {
        selected_file = path;
        return *this;
    }
private:
    ImGui::FileBrowser file_dialog;
    std::filesystem::path selected_file;

    IDataOutput<groot::PlantGraph>& output;
};


void lua_open_graph(lua_State* L);