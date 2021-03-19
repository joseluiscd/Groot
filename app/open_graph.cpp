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
    try {
        output = groot::read_from_file(file);
    } catch (boost::archive::archive_exception& e) {
        error_string = e.what();
        return CommandState::Error;
    }

    return CommandState::Ok;
}

int open_graph_lua_impl(lua_State* L)
{
    //TODO: Update this method
    try {
        return [&]() {
            const char* filename = luaL_checkstring(L, 1);
            lua_pushnil(L);
            LuaStackDataOutput<groot::PlantGraph> out(L, -1);

            OpenGraph cmd = OpenGraph(out)
                                .set_file(filename);
            CommandState status = cmd.execute();

            if (status == CommandState::Ok) {
                return 1;
            } else {
                throw std::runtime_error("Not ok");
            }
        }();
    } catch (std::exception& e) {
        return luaL_error(L, "open graph error: %s", e.what());
    }
}

void lua_open_graph(lua_State* L)
{
    lua_getglobal(L, "Groot");
    lua_pushcfunction(L, open_graph_lua_impl);
    lua_setfield(L, -2, "open_graph");
}