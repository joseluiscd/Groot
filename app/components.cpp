#include "components.hpp"
#include "entity_editor.hpp"
#include "resources.hpp"
#include <gfx/imgui/imgui_stdlib.h>

void check_normals_cloud(entt::registry& reg, entt::entity entity)
{
    if (!reg.all_of<PointCloud>(entity) && reg.get<PointCloud>(entity).cloud.size() != reg.get<PointNormals>(entity).normals.size()) {
        reg.remove<PointNormals>(entity);
    }
}

void clear_selection(entt::registry& reg, entt::entity entity)
{
    for (auto [it] : reg.view<Selected>().each()) {
        if (entity != it) {
            reg.remove<Selected>(it);
        }
    }
}

void init_components(entt::registry& reg)
{
    auto& entity_editor = reg.ctx<EntityEditor>();

    entity_editor.registerComponent<Name>("Name");
    entity_editor.registerComponent<Selected>("Selected");
    entity_editor.registerComponent<PointCloud>("Point Cloud");

    // Make only one selected item
    reg.on_construct<Selected>().connect<&clear_selection>();

    // Ensure that the normals are always "valid"
    reg.on_construct<PointNormals>().connect<&check_normals_cloud>();
    reg.on_update<PointNormals>().connect<&check_normals_cloud>();
    reg.on_update<PointCloud>().connect<&check_normals_cloud>();
}

namespace MM {

template <>
void ComponentEditorWidget<Name>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<Name>(e);
    ImGui::InputText("Name", &t.name);
}

template <>
void ComponentEditorWidget<PointCloud>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<PointCloud>(e);
    ImGui::Text("Point cloud with %d points.", t.cloud.size());
    if (ImGui::TreeNode("Points")) {
        for (auto it = t.cloud.begin(); it != t.cloud.end(); ++it) {
            ImGui::Text("(%f, %f, %f)", it->x(), it->y(), it->z());
        }
        ImGui::TreePop();
    }
}

template <>
void ComponentEditorWidget<PointNormals>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<PointNormals>(e);
    ImGui::Text("Point cloud with %d normals.", t.normals.size());
}

template <>
void ComponentEditorWidget<Cylinders>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<Cylinders>(e);
    ImGui::Text("Cylinder collection with %d cylinders.", t.cylinders.size());
}

}