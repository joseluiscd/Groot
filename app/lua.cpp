#include "lua.hpp"
#include <entt/entt.hpp>
#include <sol/sol.hpp>
#include "components.hpp"
#include "open_workspace.hpp"
#include "save_workspace.hpp"

void LuaEnv::lua_init()
{
    lua.open_libraries();

    sol::usertype<entt::registry> registry_type = lua.new_usertype<entt::registry>(
        "Registry", sol::no_constructor,
        "get", [&]() { return std::ref(reg); },
        "selected", [](entt::registry& reg) { return entt::handle(reg, reg.ctx<SelectedEntity>().selected); },
        "entities", [](entt::registry& reg) {
            std::vector<entt::handle> handles;
            reg.each([&](auto entity){
                handles.push_back(entt::handle(reg, entity));
            });

            return sol::as_table(handles);
        },
        "load", [](entt::registry& reg, std::string filename) {
            OpenWorkspace cmd(reg);
            cmd.set_file(filename);

            return cmd.execute() == CommandState::Ok;
        },
        "save", [](entt::registry& reg, std::string filename) {
            SaveWorkspace cmd(reg);
            cmd.set_file(filename);

            return cmd.execute() == CommandState::Ok;
        }
    );

    sol::usertype<entt::handle> entity_type = lua.new_usertype<entt::handle>(
        "Entity", sol::no_constructor,
        "new", [&]() { return entt::handle(reg, reg.create()); },
        "point_cloud", [](entt::handle e){ return e.try_get<PointCloud>(); }, 
        "cloud_normals", [](entt::handle e) { return  e.try_get<PointNormals>(); },
        "select", [](entt::handle e) {
            e.registry()->ctx<SelectedEntity>().selected = e.entity();
        }
    );
}
