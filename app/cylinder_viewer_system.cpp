#include "particle_sim.hpp"
#include <gfx/camera.hpp>
#include <gfx/imgui/imgui.h>
#include <groot/cloud_load.hpp>
#include <gfx/gfx.hpp>
#include <gfx/render_pass.hpp>

gfx::VertexArray::Layout cylinders_layout = {
    { 0, 3, gfx::Type::Float }, // Center 
    { 1, 3, gfx::Type::Float }, // Direction
    { 2, 1, gfx::Type::Float }, // Curvature radius
    { 3, 1, gfx::Type::Float } //Height
};

ParticleSim::ParticleSim()
    : file_browser(ImGuiFileBrowserFlags_CloseOnEsc)
    , curvatures()
    , vao()
{
    file_browser.SetTitle("PLY open");
    file_browser.SetTypeFilters({ ".ply" });

    pipeline = gfx::RenderPipeline::Builder("Curvatures_pipeline")
                   .with_shader(gfx::ShaderProgram::Builder("curvatures")
                                    .register_class<gfx::CameraRig>()
                                    .register_class<gfx::CameraLens>()
                                    .with_vertex_shader(curvature_vsh)
                                    .with_geometry_shader(curvature_gsh)
                                    .with_fragment_shader(curvature_fsh)
                                    .build())
                   .build_unique();
    pipeline_lines = gfx::RenderPipeline::Builder("Curvatures_pipeline")
                         .with_shader(gfx::ShaderProgram::Builder("curvature_lines")
                                          .register_class<gfx::CameraRig>()
                                          .register_class<gfx::CameraLens>()
                                          .with_vertex_shader(curvature_vsh)
                                          .with_geometry_shader(curvature_gsh_lines)
                                          .with_fragment_shader(curvature_fsh)
                                          .build())
                         .build_unique();

    vao
        .add_buffer(features_layout, curvatures)
        .set_mode(gfx::Mode::Points);
}

bool ParticleSim::draw_gui()
{
    if (config) {
        draw_gui_config();
    }

    if (visualize) {
        return AbstractViewer::draw_gui();
    }

    return true;
}

void ParticleSim::draw_gui_config()
{
    if (ImGui::Begin("Particle simulation")) {
        if (file_browser.HasSelected()) {
            selected_file = file_browser.GetSelected();
            file_browser.ClearSelected();
        }

        if (ImGui::Button("Select file...")) {
            file_browser.Open();
        }

        ImGui::SameLine();

        ImGui::Text("%s", selected_file.c_str());

        ImGui::Separator();

        ImGui::InputInt("K", &k, 1, 5);
        ImGui::InputFloat("Max radius", &max_radius);
        ImGui::Separator();

        ImGui::InputFloat("Epsilon", &params.epsilon, 0.05, 0.1);
        ImGui::InputFloat("Normal Threshold", &params.normal_threshold, 0.1, 0.5);
        ImGui::InputFloat("Cluster epsilon", &params.cluster_epsilon);
        ImGui::InputInt("Min points", (int*) &params.min_points);
        ImGui::InputFloat("Missing probability", &params.probability);
        ImGui::Separator();

        ImGui::Checkbox("Show lines only", &only_lines);
        ImGui::Separator();

        if (ImGui::Button("Show")) {
            run_gui_config();
        }
    }
    file_browser.Display();

    ImGui::End();
}

void ParticleSim::run_gui_config()
{
    this->visualize = true;
    this->cloud = groot::load_PLY(this->selected_file.c_str());

    std::vector<groot::Curvature> curv(cloud.size());
    try {
        auto edit = curvatures.edit();
        edit.vector().clear();
        std::vector<groot::Vector_3> normals = groot::compute_normals(cloud.data(), cloud.size(), this->k, this->max_radius);
        groot::compute_cylinders(cloud.data(), normals.data(), cloud.size(), edit.vector());
        vao.set_element_count(edit.vector().size());
    } catch (CGAL::Precondition_exception& e) {
        spdlog::error("{}: {}", e.what(), e.expression());
    }
}

void ParticleSim::render()
{
    
    if (only_lines) {
        gfx::RenderPass(framebuffer, gfx::ClearOperation::color_and_depth({0.7, 0.7, 0.7, 1.0}))
            .viewport({0, 0}, size)
            .set_pipeline(*pipeline_lines)
            .with_camera(camera_rig)
            .draw(vao)
            .end_pipeline();
    } else {
        gfx::RenderPass(framebuffer, gfx::ClearOperation::color_and_depth({0.7, 0.7, 0.7, 1.0}))
            .viewport({ 0, 0 }, size)
            .set_pipeline(*pipeline)
            .with_camera(camera_rig)
            .draw(vao)
            .end_pipeline();
    }

}