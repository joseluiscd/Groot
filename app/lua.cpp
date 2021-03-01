#include "lua.hpp"
#include "application.hpp"
#include <new>

int run_command(lua_State* L)
{
    return 0;
}

const luaL_Reg LuaClass<groot::PlantGraph>::functions[] = {
    { "new", &new_default_value<groot::PlantGraph> },
    { "__gc", &gc_value<groot::PlantGraph> },
    { nullptr, nullptr }
};
