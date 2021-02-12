#include "graph_view.hpp"
#include <gfx/camera.hpp>
#include <gfx/shader_program.hpp>
#include <gfx/debug_draw.hpp>

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

GraphViewer::GraphViewer(groot::PlantGraph& _graph)
    : graph(nullptr)
    , point_vao()
    , line_vao()
    , points()
    , lines()
    , color_point(glm::vec3(1, 0, 0))
    , color_line(glm::vec3(0, 1, 0))
    , root()
    , draw_root(false)
{
    point_vao
        .add_buffer(point_layout, points)
        .set_mode(gfx::Mode::Points);

    line_vao
        .add_buffer(point_layout, points)
        .set_indices_buffer(lines)
        .set_mode(gfx::Mode::Lines);

    //this->set_plant_graph(_graph);
}

gfx::RenderPipeline& GraphViewer::get_pipeline()
{
    static gfx::RenderPipeline pipeline = gfx::RenderPipeline::Builder()
                                              .with_shader(gfx::ShaderProgram::Builder()
                                                               .register_uniform<Color>()
                                                               .register_class<gfx::CameraLens>()
                                                               .register_class<gfx::CameraRig>()
                                                               .with_vertex_shader(vertex_shader_source)
                                                               .with_fragment_shader(fragment_shader_source)
                                                               .build())
                                              .clear_color(glm::vec4(0.4, 0.4, 0.4, 0.0))
                                              .build();

    return pipeline;
}

void GraphViewer::set_plant_graph(groot::PlantGraph& _graph)
{
    this->graph = &_graph;

    gfx::Buffer<glm::vec3>::Editor edit_points = points.edit(false);
    gfx::Buffer<uint32_t>::Editor edit_lines = lines.edit(false);

    edit_points.vector().clear();
    edit_lines.vector().clear();

    groot::NodeMapT<size_t> locations(*graph->graph, 0xFFFF);

    size_t location = 0;
    for (groot::NodeIt i(*graph->graph); i != lemon::INVALID; ++i) {
        locations[i] = location++;
        edit_points.vector().push_back((*graph->point)[i]);
    }

    for (groot::EdgeIt i(*graph->graph); i != lemon::INVALID; ++i) {
        groot::NodeT u = graph->graph->u(i);
        groot::NodeT v = graph->graph->v(i);

        edit_lines.vector().push_back(locations[u]);
        edit_lines.vector().push_back(locations[v]);
    }

    point_vao.set_element_count(edit_points.vector().size());
    line_vao.set_element_count(edit_lines.vector().size());

    draw_root = true;
    root = (*graph->point)[graph->root];
}

void GraphViewer::render(gfx::RenderSurface& surface, gfx::ICamera& camera)
{
    get_pipeline()
        .begin_render(surface)
        .with_camera(camera)
        .bind(color_point)
        .draw(point_vao)
        .bind(color_line)
        .draw(line_vao)
        .end_render();

    if (draw_root) dd::point(root, dd::colors::Yellow, 15.0, 0, false);
}
