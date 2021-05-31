#include "lua.hpp"
#include "cloud_system.hpp"
#include "components.hpp"
#include "cylinder_marching.hpp"
#include "import_ply.hpp"
#include "open_workspace.hpp"
#include "save_workspace.hpp"
#include "sol/raii.hpp"
#include <entt/entt.hpp>

#ifndef __INTELLISENSE__
#include <sol/sol.hpp>
#endif // Intellisense blocker

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

            return sol::as_table(handles); },
        "load", [](entt::registry& reg, std::string filename) {
            OpenWorkspace cmd(reg);
            cmd.set_file(filename);

            throw_on_error(cmd.run(), "Could not load workspace"); },
        "save", [](entt::registry& reg, std::string filename) {
            SaveWorkspace cmd(reg);
            cmd.set_file(filename);

            throw_on_error(cmd.run(), "Could not save workspace"); },

        "load_ply", [](entt::registry& reg, std::string filename) {
            ImportPLY cmd(reg);
            cmd.input_file = filename;

            throw_on_error(cmd.run(), "Could not load PLY");
            return entt::handle{ reg, cmd.result }; });

    sol::usertype<entt::handle> entity_type = lua.new_usertype<entt::handle>(
        "Entity", sol::no_constructor,
        "new", [&]() { return entt::handle(reg, reg.create()); },
        "point_cloud", [](entt::handle e) { return e.try_get<PointCloud>(); },
        "cloud_normals", [](entt::handle e) { return e.try_get<PointNormals>(); },
        "cylinders", [](entt::handle e) { return e.try_get<Cylinders>(); },
        "select", [](entt::handle e) { e.registry()->ctx<SelectedEntity>().selected = e.entity(); },
        "compute_normals", [](entt::handle e, sol::table args) {
            ComputeNormals cmd(std::move(e));
            if (std::optional<int> r = args["k"]; r) {
                cmd.k = *r;
            }
            if (std::optional<float> r = args["radius"]; r) {
                cmd.radius = *r;
            }
 
            throw_on_error(cmd.run(), "Error computing normals"); },
        "cylinder_marching", [](entt::handle e, sol::table args) {
            CylinderMarching cm(std::move(e));
            if (std::optional<int> r = args["min_points"]; r) {
                cm.min_points = *r;
            }
            if (std::optional<float> r = args["epsilon"]; r) {
                cm.epsilon = *r;
            }
            if (std::optional<float> r = args["sampling"]; r) {
                cm.sampling = *r;
            }
            if (std::optional<float> r = args["normal_deviation"]; r) {
                cm.normal_deviation = *r;
            }
            if (std::optional<float> r = args["overlook_probability"]; r) {
                cm.overlook_probability = *r;
            }
            if (std::optional<float> r = args["voxel_size"]; r) {
                cm.voxel_size = *r;
            }

            throw_on_error(cm.run(), "Cylinder marching error"); },
        "cylinder_filter", [](entt::handle e, sol::table args){
            CylinderFilter cmd(std::move(e));
            if (std::optional<sol::table> r = args["radius"]; r) {
                cmd.filter_radius = true;
                cmd.radius_range[0] = (*r)[1];
                cmd.radius_range[1] = (*r)[2];
            } else {
                cmd.filter_radius = false;
            }
            if (std::optional<sol::table> r = args["length"]; r) {
                cmd.filter_length = true;
                cmd.length_range[0] = (*r)[1];
                cmd.length_range[1] = (*r)[2];
            } else {
                cmd.filter_length = false;
            }

            throw_on_error(cmd.run(), "Cylinder filter error");
        },
        
        "split_cloud", [](entt::handle e, float voxel_size) {
            SplitCloud cmd(std::move(e));
            cmd.voxel_size = voxel_size;
            throw_on_error(cmd.run(), "Voxel split error");
            return sol::as_table(cmd.result);
        },
        "rebuild_cloud_from_cylinders", [](entt::handle e) {
            CylinderPointFilter cmd(std::move(e));
            throw_on_error(cmd.run(), "Point cloud from cylinders error");
        }
        );

        sol::usertype<Cylinders> cylinders_type = lua.new_usertype<Cylinders>(
            "Cylinders", sol::no_constructor,
            "len", [](const Cylinders& c) { return c.cylinders.size(); });
}
