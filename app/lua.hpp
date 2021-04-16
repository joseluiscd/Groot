#pragma once

#include <sol/sol.hpp>
#include <entt/entt.hpp>

class LuaEnv {
public:
    LuaEnv(entt::registry& _reg)
        : lua()
        , reg(_reg)
    {
        lua_init();
    }

    void run_file(std::string& filename)
    {
        lua.script_file(filename);
    }

private:
    sol::state lua;
    entt::registry& reg;

    void lua_init();

};
