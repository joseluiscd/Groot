#include "application.hpp"
#define DOCTEST_CONFIG_IMPLEMENT
#include "cxxopts.hpp"
#include <doctest/doctest.h>

int main(int argc, const char* argv[])
{
    cxxopts::Options opts(argv[0], " - The Generalized ReconstructiOn Of Trees");

    std::string lua_file;
    std::string config;

    opts.set_width(70)
        .set_tab_expansion()
        .add_options()
            ("s,script", "run a Lua script", cxxopts::value<std::string>(lua_file), "FILE")
            ("c,config", "config string for Lua script", cxxopts::value<std::string>(config), "CONFIG");

    auto result = opts.parse(argc, argv);
    if (!lua_file.empty()) {
        entt::registry reg;
        LuaEnv lua { reg };
        lua.run_file(lua_file, config);
    } else {
        Application app;
        app.main_loop();
    }

    return 0;
}
