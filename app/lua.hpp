#pragma once

extern "C" {
#include <lua.h>

#include <lauxlib.h>
#include <lualib.h>
}
#include <groot/plant_graph.hpp>

template <typename Class>
struct LuaClass {
    static constexpr const char* lua_name = "";

    static const luaL_Reg functions[];

    static void register_lua(lua_State* L);
    static void push_metatable(lua_State* L);
};

template <typename Class>
struct LuaFunc {
};

template <typename Class>
Class* check_value(lua_State* L, int index)
{
    Class** cls = (Class**)luaL_checkudata(L, index, LuaClass<Class>::lua_name);
    return *cls;
}

template <typename Class>
void push_metatable(lua_State* L)
{
    if (luaL_newmetatable(L, LuaClass<Class>::lua_name)) {
        luaL_setfuncs(L, LuaClass<Class>::functions, 0);
        lua_pushvalue(L, -1);
        lua_setfield(L, -2, "__index");
    }
}

template <typename Class>
int gc_value(lua_State* L)
{
    Class* cls = check_value<Class>(L, 1);
    delete cls;
    return 0;
}

template <typename Class>
int new_default_value(lua_State* L)
{
    static_assert(std::is_default_constructible_v<Class>, "Class must be default constructible");

    Class** c = (Class**)lua_newuserdata(L, sizeof(Class*));
    *c = new Class();

    push_metatable<Class>(L);
    lua_setmetatable(L, -2);
    return 1;
}

template <typename Class>
int new_move_value(lua_State* L, Class&& value)
{
    static_assert(std::is_move_constructible_v<Class>, "Class must be move constructible");

    Class** c = (Class**)lua_newuserdata(L, sizeof(Class*));
    *c = new Class(std::move(value));

    push_metatable<Class>(L);
    lua_setmetatable(L, -2);
    return 1;
}

inline static void dumpstack(lua_State* L)
{
    int top = lua_gettop(L);
    for (int i = 1; i <= top; i++) {
        printf("%d\t%s\t", i, luaL_typename(L, i));
        switch (lua_type(L, i)) {
        case LUA_TNUMBER:
            printf("%g\n", lua_tonumber(L, i));
            break;
        case LUA_TSTRING:
            printf("%s\n", lua_tostring(L, i));
            break;
        case LUA_TBOOLEAN:
            printf("%s\n", (lua_toboolean(L, i) ? "true" : "false"));
            break;
        case LUA_TNIL:
            printf("%s\n", "nil");
            break;
        default:
            printf("%p\n", lua_topointer(L, i));
            break;
        }
    }
}


template <>
struct LuaClass<groot::PlantGraph> {
    static constexpr const char* lua_name = "Groot.PlantGraph";
    static const luaL_Reg functions[];
};

