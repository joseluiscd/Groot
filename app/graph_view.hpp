#pragma once

#include <groot/plant_graph.hpp>
#include <gfx/vertex_array.hpp>
#include <gfx/uniform.hpp>
#include <gfx/framebuffer.hpp>
#include <gfx/camera.hpp>
#include "data_source.hpp"

class GraphViewer {
public:
    GraphViewer(IDataSource<groot::PlantGraph>& graph);
    void update_plant_graph();
    void remove_plant_graph();

    bool render();

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

    gfx::Framebuffer framebuffer;
    gfx::CameraRig camera_rig;
    gfx::PerspectiveCameraLens camera_lens;

    glm::vec3 root;
    glm::ivec2 old_size;

    IDataSource<groot::PlantGraph>& graph;

    bool open = true;

};