#pragma once

#include "application.hpp"
#include <filesystem>
#include <gfx/imgui/imfilebrowser.h>
#include "data_output.hpp"
#include "lua.hpp"
#include <entt/entt.hpp>

class OpenGraph : public CommandGui {
public:
    OpenGraph(entt::registry& registry);
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

    entt::registry& registry;
};


void lua_open_graph(lua_State* L);