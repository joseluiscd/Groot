#pragma once

#include <groot/skeleton.hpp>
#include <gfx/vertex_array.hpp>
#include <gfx/uniform.hpp>



class GraphViewer {
public:
    GraphViewer(groot::PlantGraph &_graph);
    void set_plant_graph(groot::PlantGraph& _graph);

    void render(gfx::RenderSurface& surface, gfx::ICamera& camera);

    gfx::RenderPipeline& get_pipeline();

    DEF_UNIFORM_SEMANTICS(Color, glm::vec3, "kColor");

private:
    GraphViewer();
    
    groot::PlantGraph *graph;

    gfx::VertexArray point_vao;
    gfx::VertexArray line_vao;

    gfx::Buffer<glm::vec3> points;
    gfx::Buffer<uint32_t> lines;

    gfx::Uniform<Color> color_point;
    gfx::Uniform<Color> color_line;

    bool draw_root;
    glm::vec3 root;

};