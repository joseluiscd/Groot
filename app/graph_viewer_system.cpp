#include "graph_viewer_system.hpp"
#include "components.hpp"
#include <gfx/buffer.hpp>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <groot/plant_graph.hpp>

namespace graph_viewer_system {

DEF_UNIFORM_SEMANTICS(Color, glm::vec3, "kColor");

extern const char* vertex_shader_source;
extern const char* fragment_shader_source;

entt::observer update_graph;

struct GraphViewerComponent {
    gfx::VertexArray point_vao;
    gfx::VertexArray line_vao;

    gfx::Buffer<glm::vec3> points;
    gfx::Buffer<uint32_t> lines;

    gfx::Uniform<Color> color_point;
    gfx::Uniform<Color> color_line;

    glm::vec3 root;
};

struct SystemData {
    gfx::RenderPipeline pipeline;
};

void init(entt::registry& registry) {

    update_graph.connect(registry, 
        entt::collector.update<groot::PlantGraph>().group<groot::PlantGraph>()
    );

    registry.set<SystemData>(SystemData {
        gfx::RenderPipeline::Builder()
            .with_shader(gfx::ShaderProgram::Builder()
                             .register_uniform<Color>()
                             .register_class<gfx::CameraLens>()
                             .register_class<gfx::CameraRig>()
                             .with_vertex_shader(vertex_shader_source)
                             .with_fragment_shader(fragment_shader_source)
                             .build())
            .build()});
}

void run(entt::registry& registry)
{
    auto& data = registry.ctx<SystemData>();
    auto& view_data = registry.ctx<viewer_system::SystemData>();

    for (const auto entity : update_graph) {
        groot::PlantGraph& _graph = registry.get<groot::PlantGraph>(entity);
        registry.emplace<GraphViewerComponent>(entity);

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
    }

    update_graph.clear();

    const auto view = registry.view<GraphViewerComponent, Visible>();
    for (const auto entity : view) {
        auto& graph_view = registry.get<GraphViewerComponent>(entity);

        gfx::RenderPass(view_data.framebuffer, gfx::ClearOperation::nothing())
            .viewport({ 0, 0 }, view_data.size)
            .set_pipeline(data.pipeline)
            .with_camera(*view_data.camera)
            .bind(graph_view.color_point)
            .draw(graph_view.point_vao)
            .bind(graph_view.color_line)
            .draw(graph_view.line_vao)
            .end_pipeline();
    }
}

const char* vertex_shader_source = "\n"
                                   "layout (location=0) in vec3 in_Position;\n"
                                   "\n"
                                   "out vec4 v_Color;\n"
                                   "layout (location=kViewMatrix) uniform mat4 u_mvMatrix;\n"
                                   "layout (location=kProjectionMatrix) uniform mat4 u_pMatrix;\n"
                                   "layout (location=kColor) uniform vec3 u_color;\n"
                                   "\n"
                                   "void main()\n"
                                   "{\n"
                                   "    mat4 u_MvpMatrix = u_pMatrix * u_mvMatrix;\n"
                                   "    gl_Position  = u_MvpMatrix * vec4(in_Position, 1.0);\n"
                                   "    v_Color      = vec4(u_color, 1.0);\n"
                                   "    gl_PointSize = 5.0;\n"
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