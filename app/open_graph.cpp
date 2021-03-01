#include "open_graph.hpp"
#include <fstream>
#include <spdlog/spdlog.h>

OpenGraph::OpenGraph(IDataOutput<groot::PlantGraph>& _output)
    : output(_output)
    , file_dialog()
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
    output = groot::read_from_file(file);
    return CommandState::Ok;
}

int open_graph_lua_impl(lua_State* L) {
    const char* filename = luaL_checkstring(L, 1);
    lua_pushnil(L);
    LuaStackDataOutput<groot::PlantGraph> out(L, -1);
    OpenGraph(out)
        .set_file(filename)
        .execute();
    return 1;
}

void lua_open_graph(lua_State* L) 
{
    lua_getglobal(L, "Groot");
    lua_pushcfunction(L, open_graph_lua_impl);
    lua_setfield(L, -2, "open_graph");
}