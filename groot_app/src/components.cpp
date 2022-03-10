#include <gfx/imgui/imgui.h>
#include <gfx/imgui/imgui_stdlib.h>
#include <groot/cloud_load.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entity_editor.hpp>
#include <groot_app/resources.hpp>
#include <groot_app/task.hpp>
#include <groot_graph/connectivity_repair.hpp>

void check_normals_cloud(entt::registry& reg, entt::entity entity)
{
    if (!reg.all_of<PointCloud, PointNormals>(entity) || reg.get<PointCloud>(entity).cloud.size() != reg.get<PointNormals>(entity).normals.size()) {
        reg.remove<PointNormals>(entity);
    }
}

void init_components(entt::registry& reg)
{
    auto& entity_editor = reg.ctx<EntityEditor>();

    entity_editor.registerComponent<Name>("Name");
    entity_editor.registerComponent<PointCloud>("Point Cloud");
    entity_editor.registerComponent<PointNormals>("Point Normals");
    entity_editor.registerComponent<Cylinders>("Cylinders");
    entity_editor.registerComponent<groot::PlantGraph>("PlantGraph");
    entity_editor.registerComponent<PlantGraphNodePoints>("PlantNodePoints");
    entity_editor.registerComponent<ConnectedComponents>("Connected Components");

    // Ensure that the normals are always "valid"
    reg.on_construct<PointNormals>().connect<&check_normals_cloud>();
    reg.on_update<PointNormals>().connect<&check_normals_cloud>();
    reg.on_update<PointCloud>().connect<&check_normals_cloud>();

    // No connected components if no graph
    reg.on_destroy<groot::PlantGraph>().connect<&entt::registry::remove<ConnectedComponents>>();
    reg.on_update<groot::PlantGraph>().connect<&entt::registry::remove<ConnectedComponents>>();
}

namespace MM {

template <>
void ComponentEditorWidget<Name>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<Name>(e);
    ImGui::InputText("Name", &t.name);
}

template <>
void ComponentEditorWidget<groot::PlantGraph>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<groot::PlantGraph>(e);
    size_t vertices = boost::num_vertices(t);
    size_t edges = boost::num_edges(t);

    ImGui::Text("Plant Graph: %zu vertices, %zu edges", vertices, edges);
    ImGui::Text("Root node ID: %zu", t.m_property->root_index);

    if (ImGui::TreeNode("Points")) {
        auto [it, end] = boost::vertices(t);
        for (; it != end; ++it) {
            const groot::Point_3& p = t[*it].position;
            ImGui::Text("(%f, %f, %f)", p.x(), p.y(), p.z());
        }
        ImGui::TreePop();
    }
}

template <>
void ComponentEditorWidget<PointCloud>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<PointCloud>(e);
    ImGui::Text("Point cloud with %zu points.", t.cloud.size());
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
    ImGui::Text("Point cloud with %zu normals.", t.normals.size());
    if (ImGui::TreeNode("Normals")) {
        for (auto it = t.normals.begin(); it != t.normals.end(); ++it) {
            ImGui::Text("(%f, %f, %f)", it->x(), it->y(), it->z());
        }
        ImGui::TreePop();
    }
}

template <>
void ComponentEditorWidget<Cylinders>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<Cylinders>(e);
    ImGui::Text("Cylinder collection with %zu cylinders.", t.cylinders.size());
    if (ImGui::TreeNode("Cylinders")) {

        for (size_t i = 0; i < t.cylinders.size(); i++) {
            if (ImGui::TreeNode((void*)i, "Cylinder %zu", i)) {
                auto& c = t.cylinders[i];
                ImGui::Text("Center: (%f, %f, %f)", c.cylinder.center.x(), c.cylinder.center.y(), c.cylinder.center.z());
                ImGui::Text("Direction: (%f, %f, %f)", c.cylinder.direction.x(), c.cylinder.direction.y(), c.cylinder.direction.z());
                ImGui::Text("Radius: %f", c.cylinder.radius);
                ImGui::Text("Length: %f", c.cylinder.middle_height * 2.0);
                ImGui::Text("Points: %zu", c.points.size());

                if (ImGui::TreeNode("Points")) {
                    for (size_t j = 0; j < c.points.size(); j++) {
                        ImGui::Text("Point: (%f, %f, %f)", c.points[j].x(), c.points[j].y(), c.points[j].z());
                    }
                    ImGui::TreePop();
                }
                ImGui::TreePop();
            }
        }
        ImGui::TreePop();
    }
}

template <>
void ComponentAddAction<PointNormals>(entt::registry& reg, entt::entity entity)
{
    if (reg.all_of<PointCloud>(entity)) {
        reg.emplace<PointNormals>(entity);
    }
}

template <>
void ComponentEditorWidget<PlantGraphNodePoints>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<PlantGraphNodePoints>(e);

    for (size_t i = 0; i < t.points.size(); i++) {
        if (ImGui::TreeNode((void*)i, "Node %zu", i)) {
            ImGui::Text("Node with %zu points", t.points[i].size());
            if (ImGui::Button("Dump to file")) {
                reg.ctx<TaskManager>().push_task(
                    "Dump points to file",
                    async::spawn(async_scheduler(), [&t, i]() {
                        groot::save_PLY("out.ply", t.points[i].data(), t.points[i].size());
                    }));
            }
            if (ImGui::TreeNode("Points")) {
                for (size_t j = 0; j < t.points[i].size(); j++) {
                    ImGui::Text("Point: (%f, %f, %f)",
                        t.points[i][j].x(),
                        t.points[i][j].y(),
                        t.points[i][j].z());
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
    }
}

template <>
void ComponentAddAction<ConnectedComponents>(entt::registry& reg, entt::entity e)
{
    if (reg.all_of<groot::PlantGraph>(e)) {
        const auto& graph = reg.get<groot::PlantGraph>(e);
        reg.emplace<ConnectedComponents>(e, groot::compute_connected_components(graph));
    }
}

template <>
void ComponentEditorWidget<ConnectedComponents>(entt::registry& reg, entt::entity e)
{
    static int query_component = 0;
    static ssize_t query_result = -1;

    auto& cc = reg.get<ConnectedComponents>(e);
    ImGui::Text("Count: %zu\n", cc.components.num_sets());
    ImGui::Text("Vertices: %zu\n", cc.components.num_vertices());
    if (ImGui::InputInt("Vertex component", &query_component)) {
        if (0 <= query_component && (size_t)query_component < cc.components.num_vertices()) {
            query_result = cc.components.find(query_component);
        } else {
            query_result = -1;
        }
    }

    if (query_result > 0) {
        ImGui::Text("Component: %zu", query_result);
    }
}

}