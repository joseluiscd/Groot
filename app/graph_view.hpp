#pragma once

#include "data_source.hpp"
#include "viewer.hpp"
#include <gfx/uniform.hpp>
#include <gfx/vertex_array.hpp>
#include <groot/plant_graph.hpp>

class GraphViewer : public Viewer<groot::PlantGraph> {
public:
    GraphViewer(IDataSource<groot::PlantGraph>& graph);
    void update_plant_graph();
    void remove_plant_graph();

    void render();

    gfx::RenderPipeline& get_pipeline();

    DEF_UNIFORM_SEMANTICS(Color, glm::vec3, "kColor");

private:
    GraphViewer();

    gfx::VertexArray point_vao;
    gfx::VertexArray line_vao;

    gfx::Buffer<glm::vec3> points;
    gfx::Buffer<uint32_t> lines;

    gfx::Uniform<Color> color_point;
    gfx::Uniform<Color> color_line;

    glm::vec3 root;

    std::unique_ptr<gfx::RenderPipeline> pipeline;
};
