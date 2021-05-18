#pragma once

#ifndef __INTELLISENSE__
#include <sol/sol.hpp>
#endif // Intellisense blocker

#include <entt/entt.hpp>

class LuaEnv {
public:
    LuaEnv(entt::registry& _reg)
        : lua()
        , reg(_reg)
    {
        lua_init();
    }

    void run_file(std::string& filename, const std::string& config)
    {
        lua["config"] = config;
        lua.script_file(filename);
    }

private:
    sol::state lua;
    entt::registry& reg;

    void lua_init();

};
