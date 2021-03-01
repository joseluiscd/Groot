#pragma once

#include <memory>
#include "editor_imgui.hpp"
#include "lua.hpp"

class Editor {
public:
    Editor(lua_State* _L);

    bool render();
    void draw_menu_bar();

private:
    std::unique_ptr<Zep::ZepEditor_ImGui> editor;
    lua_State* L;
};