#include "graph_view.hpp"
#include <gfx/camera.hpp>
#include <gfx/shader_program.hpp>
#include <gfx/render_pass.hpp>

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
;
const char* fragment_shader_source = "\n"
                                     "in  vec4 v_Color;\n"
                                     "out vec4 out_FragColor;\n"
                                     "\n"
                                     "void main()\n"
                                     "{\n"
                                     "    out_FragColor = v_Color;\n"
                                     "}\n";

gfx::VertexArray::Layout point_layout = {
    { 0, 3, gfx::Type::Float } // Position
};

GraphViewer::GraphViewer(IDataSource<groot::PlantGraph>& _graph)
    : Viewer<groot::PlantGraph>(_graph)
    , point_vao()
    , line_vao()
    , points()
    , lines()
    , color_point(glm::vec3(1, 0, 0))
    , color_line(glm::vec3(0, 1, 0))
    , root()
{
    point_vao
        .add_buffer(point_layout, points)
        .set_mode(gfx::Mode::Points);

    line_vao
        .add_buffer(point_layout, points)
        .set_indices_buffer(lines)
        .set_mode(gfx::Mode::Lines);

    pipeline = gfx::RenderPipeline::Builder()
                                           .with_shader(gfx::ShaderProgram::Builder()
                                                            .register_uniform<Color>()
                                                            .register_class<gfx::CameraLens>()
                                                            .register_class<gfx::CameraRig>()
                                                            .with_vertex_shader(vertex_shader_source)
                                                            .with_fragment_shader(fragment_shader_source)
                                                            .build())
                                           .build_unique();
}

gfx::RenderPipeline& GraphViewer::get_pipeline()
{
    return *this->pipeline;
}

void GraphViewer::remove_plant_graph()
{
    open = false;
}

void GraphViewer::update_plant_graph()
{
    groot::PlantGraph& _graph = *input;

    gfx::Buffer<glm::vec3>::Editor edit_points = points.edit(false);
    gfx::Buffer<uint32_t>::Editor edit_lines = lines.edit(false);

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

    point_vao.set_element_count(edit_points.vector().size());
    line_vao.set_element_count(edit_lines.vector().size());

    if (boost::num_vertices(_graph) > 0) {
        groot::cgal::Point_3 root_p = _graph[_graph.m_property->root_index].position;
        root = glm::vec3(root_p.x(), root_p.y(), root_p.z());
    }
}

void GraphViewer::render()
{
    gfx::RenderPass(framebuffer)
        .viewport({ 0, 0 }, size)
        .set_pipeline(get_pipeline())
        .with_camera(camera_rig)
        .bind(color_point)
        .draw(point_vao)
        .bind(color_line)
        .draw(line_vao)
        .end_pipeline();

}
