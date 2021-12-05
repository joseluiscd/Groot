#include <groot_app/graph_viewer_system.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entity_editor.hpp>
#include "entt/entity/fwd.hpp"
#include <groot_app/render.hpp>
#include <groot_app/resources.hpp>
#include <boost/graph/detail/adjacency_list.hpp>
#include <gfx/buffer.hpp>
#include <gfx/debug_draw.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <groot_graph/plant_graph.hpp>

namespace graph_viewer_system {

extern const char* vertex_shader_source;
extern const char* fragment_shader_source;

struct GraphViewerComponent {
    GraphViewerComponent();

    bool show_points = true;
    bool show_lines = true;
    bool show_ids = false;

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
    SystemData(gfx::RenderPipeline&& p)
        : pipeline(p)
        , update_graph()
    {
    }
    gfx::RenderPipeline pipeline;
    entt::observer update_graph;
};
void update_viewer_component(entt::registry& registry, entt::entity entity)
{
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

void init(entt::registry& registry)
{
    auto& system_data = registry.set<SystemData>(std::move(
        gfx::RenderPipeline::Builder()
            .with_shader(gfx::ShaderProgram::Builder()
                             .register_uniform<Color>()
                             .register_uniform<PointSize>()
                             .register_class<gfx::CameraLens>()
                             .register_class<gfx::CameraRig>()
                             .with_vertex_shader(vertex_shader_source)
                             .with_fragment_shader(fragment_shader_source)
                             .build())
            .build()));

    system_data.update_graph.connect(
        registry,
        entt::collector
            .group<groot::PlantGraph>()
            .update<groot::PlantGraph>());

    registry
            .on_destroy<groot::PlantGraph>()
            .connect<&entt::registry::remove<GraphViewerComponent>>();

    auto& entity_editor = registry.ctx<EntityEditor>();
    entity_editor.registerComponent<GraphViewerComponent>("Plant Graph View", true);
}

void deinit(entt::registry& registry)
{
    registry.clear<GraphViewerComponent>();
    registry.unset<SystemData>();
}

void run(entt::registry& registry)
{
    auto& data = registry.ctx<SystemData>();
    auto& view_data = registry.ctx<RenderData>();
    auto& draw_list = registry.ctx<TextRenderDrawList>();

    glm::mat4 view_proj_matrix = view_data.camera->get_matrix();

    data.update_graph.each([&registry](entt::entity entity) {
        if (! registry.all_of<GraphViewerComponent>(entity)) {
            registry.emplace<GraphViewerComponent>(entity);
        }
        update_viewer_component(registry, entity);
    });

    const auto view = registry.view<GraphViewerComponent, Visible>();
    for (const auto entity : view) {
        auto [graph, graph_view] = registry.get<groot::PlantGraph, GraphViewerComponent>(entity);
        gfx::DebugDraw::Builder dd;

        dd.set_color(graph_view.root_color);

        glm::vec3 root;
        if (graph.m_property->root_index >= boost::num_vertices(graph)) {
            root = glm::vec3(+INFINITY, +INFINITY, +INFINITY);
        } else  {
            root = *reinterpret_cast<glm::vec3*>(&graph[graph.m_property->root_index].position);
        }

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

        if (graph_view.show_ids) {
            for (size_t i = 0; i < graph_view.points.size(); i++) {
                glm::vec4 p = view_proj_matrix * glm::vec4(graph_view.points[i], 1.0);
                p /= p.w;
                if (p.z > 0.01 && p.z < 1.0) {
                    std::string t = fmt::format("{}", i);
                    draw_list.add_text(glm::vec2(p), t.data(), t.size());
                }
            }
        }
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
void ComponentAddAction<graph_viewer_system::GraphViewerComponent>(entt::registry& reg, entt::entity entity)
{
    if (reg.all_of<groot::PlantGraph>(entity)) {
        reg.emplace<graph_viewer_system::GraphViewerComponent>(entity);
        graph_viewer_system::update_viewer_component(reg, entity);
    }
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

    ImGui::Checkbox("Show vertex IDs", &t.show_ids);
}
}
