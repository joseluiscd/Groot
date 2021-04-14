#include "open_workspace.hpp"
#include <fstream>
#include "serde.hpp"
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
            Cylinders>(in_archive);
            
    return CommandState::Ok;
}

int open_graph_lua_impl(lua_State* L)
{
    //TODO: Update this method
    try {
        return [&]() {
            /*const char* filename = luaL_checkstring(L, 1);
            lua_pushnil(L);
            LuaStackDataOutput<groot::PlantGraph> out(L, -1);

            OpenWorkspace cmd = OpenWorkspace(out)
                                .set_file(filename);
            CommandState status = cmd.execute();

            if (status == CommandState::Ok) {
                return 1;
            } else {
                throw std::runtime_error("Not ok");
            }*/
            return 0;
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