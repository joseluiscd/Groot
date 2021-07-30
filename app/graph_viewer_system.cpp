#include "graph_viewer_system.hpp"
#include "components.hpp"
#include "entity_editor.hpp"
#include "entt/entity/fwd.hpp"
#include "render.hpp"
#include "resources.hpp"
#include <gfx/buffer.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <groot/plant_graph.hpp>
#include <gfx/debug_draw.hpp>

namespace graph_viewer_system {

extern const char* vertex_shader_source;
extern const char* fragment_shader_source;

entt::observer update_graph;
entt::observer create_graph;

struct GraphViewerComponent {
    GraphViewerComponent();

    bool show_points = true;
    bool show_lines = true;

    gfx::VertexArray point_vao;
    gfx::VertexArray line_vao;

    gfx::Buffer<glm::vec3> points;
    gfx::Buffer<uint32_t> lines;

    gfx::Uniform<Color> color_point;
    gfx::Uniform<Color> color_line;
    glm::vec3 root_color;

    glm::vec3 root;
    gfx::Uniform<PointSize> point_size;
};

GraphViewerComponent::GraphViewerComponent()
    : point_vao()
    , line_vao()
    , points()
    , lines()
    , color_point(glm::vec3(1, 0, 0))
    , color_line(glm::vec3(0, 1, 0))
    , point_size(5.0)
{
    point_vao
        .add_buffer(point_layout, points)
        .set_mode(gfx::Mode::Points);

    line_vao
        .add_buffer(point_layout, points)
        .set_indices_buffer(lines)
        .set_mode(gfx::Mode::Lines);
}

struct SystemData {
    gfx::RenderPipeline pipeline;
};

void init(entt::registry& registry)
{

    //registry.on_construct<groot::PlantGraph>()
    //    .connect<&entt::registry::emplace<GraphViewerComponent>>();
    update_graph.connect(
        registry,
        entt::collector
            .update<groot::PlantGraph>()
            .where<GraphViewerComponent>());

    create_graph.connect(
        registry,
        entt::collector.group<groot::PlantGraph>());

    registry.set<SystemData>(SystemData {
        gfx::RenderPipeline::Builder()
            .with_shader(gfx::ShaderProgram::Builder()
                             .register_uniform<Color>()
                             .register_uniform<PointSize>()
                             .register_class<gfx::CameraLens>()
                             .register_class<gfx::CameraRig>()
                             .with_vertex_shader(vertex_shader_source)
                             .with_fragment_shader(fragment_shader_source)
                             .build())
            .build() });

    auto& entity_editor = registry.ctx<EntityEditor>();
    entity_editor.registerComponent<GraphViewerComponent>("Graph View", true);
    entity_editor.registerComponent<groot::PlantGraph>("PlantGraph");
}

void deinit(entt::registry &registry)
{
    registry.clear<GraphViewerComponent>();
    registry.unset<SystemData>();
}

void run(entt::registry& registry)
{
    auto& data = registry.ctx<SystemData>();
    auto& view_data = registry.ctx<RenderData>();

    auto update_viewer_component = [&](entt::entity entity) {
        groot::PlantGraph& _graph = registry.get<groot::PlantGraph>(entity);

        registry.patch<GraphViewerComponent>(entity, [&_graph](auto& view_data) {
            auto edit_points = view_data.points.edit(false);
            auto edit_lines = view_data.lines.edit(false);

            edit_points.vector().clear();
            edit_lines.vector().clear();

            {
                auto [it, end] = boost::vertices(_graph);
                for (; it != end; ++it) {
                    groot::cgal::Point_3 position = _graph[*it].position;
                    edit_points.vector().push_back(glm::vec3(position.x(), position.y(), position.z()));
                }
            }
            {
                auto vertex_indices = boost::get(boost::vertex_index, _graph);

                auto [it, end] = boost::edges(_graph);
                for (; it != end; ++it) {
                    groot::Vertex u = boost::source(*it, _graph);
                    groot::Vertex v = boost::target(*it, _graph);

                    edit_lines.vector().push_back(vertex_indices[u]);
                    edit_lines.vector().push_back(vertex_indices[v]);
                }
            }

            view_data.point_vao.set_element_count(edit_points.vector().size());
            view_data.line_vao.set_element_count(edit_lines.vector().size());

            if (boost::num_vertices(_graph) > 0) {
                groot::cgal::Point_3 root_p = _graph[_graph.m_property->root_index].position;
                view_data.root = glm::vec3(root_p.x(), root_p.y(), root_p.z());
            }
        });
    };
    for (const auto entity : create_graph) {
        registry.emplace<GraphViewerComponent>(entity);
        registry.emplace_or_replace<Visible>(entity);

        update_viewer_component(entity);
    }

    for (const auto entity : update_graph) {
        update_viewer_component(entity);
    }

    create_graph.clear();
    update_graph.clear();

    const auto view = registry.view<GraphViewerComponent, Visible>();
    for (const auto entity : view) {
        auto [graph, graph_view] = registry.get<groot::PlantGraph, GraphViewerComponent>(entity);
        gfx::DebugDraw::Builder dd;

        dd.set_color(graph_view.root_color);
        // TODO: Dont try at home
        glm::vec3 root = *reinterpret_cast<glm::vec3*>(&graph[graph.m_property->root_index].position);
        dd.point(root);
        gfx::DebugDraw d = dd.build();

        gfx::RenderPass pass(view_data.framebuffer, gfx::ClearOperation::nothing());
        auto pipe = pass.viewport({ 0, 0 }, view_data.size)
                        .set_pipeline(data.pipeline)
                        .with_camera(*view_data.camera);
        if (graph_view.show_points) {
            pipe.bind(graph_view.point_size)
                .bind(graph_view.color_point)
                .draw(graph_view.point_vao);
        }

        if (graph_view.show_lines) {
            pipe
                .bind(graph_view.color_line)
                .draw(graph_view.line_vao);
        }
        pipe.end_pipeline();
    }
}

const char* vertex_shader_source = "\n"
                                   "layout (location=0) in vec3 in_Position;\n"
                                   "\n"
                                   "out vec4 v_Color;\n"
                                   "layout (location=kViewMatrix) uniform mat4 u_mvMatrix;\n"
                                   "layout (location=kProjectionMatrix) uniform mat4 u_pMatrix;\n"
                                   "layout (location=kColor) uniform vec3 u_color;\n"
                                   "layout (location=kPointSize) uniform float point_size;\n"
                                   "\n"
                                   "void main()\n"
                                   "{\n"
                                   "    mat4 u_MvpMatrix = u_pMatrix * u_mvMatrix;\n"
                                   "    gl_Position  = u_MvpMatrix * vec4(in_Position, 1.0);\n"
                                   "    v_Color      = vec4(u_color, 1.0);\n"
                                   "    gl_PointSize = point_size;\n"
                                   "}\n";
const char* fragment_shader_source = "\n"
                                     "in  vec4 v_Color;\n"
                                     "out vec4 out_FragColor;\n"
                                     "\n"
                                     "void main()\n"
                                     "{\n"
                                     "    out_FragColor = v_Color;\n"
                                     "}\n";
}

namespace MM {
template <>
void ComponentEditorWidget<groot::PlantGraph>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<groot::PlantGraph>(e);
    size_t vertices = boost::num_vertices(t);
    size_t edges = boost::num_edges(t);

    ImGui::Text("Plant Graph: %zu vertices, %zu edges", vertices, edges);
}

template <>
void ComponentEditorWidget<graph_viewer_system::GraphViewerComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<graph_viewer_system::GraphViewerComponent>(e);

    ImGui::Checkbox("Show points", &t.show_points);
    if (t.show_points) {
        ImGui::ColorEdit3("Point color", glm::value_ptr(*t.color_point));
        ImGui::DragFloat("Point size", &*t.point_size, 0.05, INFINITY);
    }

    ImGui::Checkbox("Show edges", &t.show_lines);
    if (t.show_lines) {
        ImGui::ColorEdit3("Line color", glm::value_ptr(*t.color_line));
    }
}
}
