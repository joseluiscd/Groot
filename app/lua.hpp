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

private:
    sol::state lua;
    entt::registry& reg;

    void lua_init();
};

template <typename T>
T from_lua(sol::table args);