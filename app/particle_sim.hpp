#pragma once

#include "command_gui.hpp"
#include "viewer.hpp"
#include <filesystem>
#include <gfx/imgui/gfx.hpp>
#include <gfx/imgui/imfilebrowser.h>
#include <gfx/render_pipeline.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/fwd.hpp>
#include <groot/cylinder_marching.hpp>
#include <groot/particles.hpp>

extern gfx::VertexArray::Layout features_layout;

class ParticleSim : public AbstractViewer {
public:
    ParticleSim();
    bool draw_gui() override;

    void draw_gui_config();
    void draw_gui_view();

    void run_gui_config();

    void render() override;

private:
    bool config = true;
    bool visualize = false;

    //groot::Particles particles;
    ImGui::FileBrowser file_browser;
    std::filesystem::path selected_file;

    std::vector<groot::cgal::Point_3> cloud;

    int k = 10;
    int d = 3;
    int dprime = 2;
    float max_radius = 1.0;

    bool only_lines = false;

    std::unique_ptr<gfx::RenderPipeline> pipeline;
    std::unique_ptr<gfx::RenderPipeline> pipeline_lines;

    gfx::VertexArray vao;
    gfx::Buffer<groot::Curvature> curvatures;

    static const char* curvature_vsh;
    static const char* curvature_gsh;
    static const char* curvature_gsh_lines;
    static const char* curvature_fsh;
};
