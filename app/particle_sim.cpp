#include "particle_sim.hpp"
#include <gfx/camera.hpp>
#include <gfx/imgui/imgui.h>
#include <groot/cloud_load.hpp>

gfx::VertexArray::Layout features_layout = {
    { 0, 3, gfx::Type::Float }, // Position of sampled point
    { 1, 3, gfx::Type::Float }, // Position (curvature center)
    { 2, 3, gfx::Type::Float }, // Direction
    { 3, 1, gfx::Type::Float }, // Curvature radius
};

ParticleSim::ParticleSim()
    : file_browser(ImGuiFileBrowserFlags_CloseOnEsc)
    , curvatures()
    , vao()
{
    file_browser.SetTitle("PLY open");
    file_browser.SetTypeFilters({ ".ply" });

    pipeline = gfx::RenderPipeline::Builder("Curvatures_pipeline")
                   .clear_color({ 0.9, 0.9, 0.9, 1.0 })
                   .with_shader(gfx::ShaderProgram::Builder("curvatures")
                                    .register_class<gfx::CameraRig>()
                                    .register_class<gfx::CameraLens>()
                                    .with_vertex_shader(curvature_vsh)
                                    .with_geometry_shader(curvature_gsh)
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
        ImGui::InputInt("d", &d, 1, 5);
        ImGui::InputInt("d'", &dprime, 1, 5);
        ImGui::InputFloat("Max radius", &max_radius);
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
        groot::compute_differential_quantities(cloud.data(), curv.data(), cloud.size(), this->k, this->d, this->dprime);
    } catch(CGAL::Precondition_exception& e) {
        spdlog::error("{}: {}", e.what(), e.expression());
        return;
    }

    auto edit_curvatures = curvatures.edit();
    edit_curvatures.vector().resize(0);

    std::copy_if(curv.begin(), curv.end(), std::back_inserter(edit_curvatures.vector()), [&](const groot::Curvature &c){
        return std::abs(c.radius) < max_radius;
    });
    vao.set_element_count(edit_curvatures.vector().size());

}

void ParticleSim::render()
{
    pipeline->begin_render(framebuffer)
        .clear()
        .viewport({ 0, 0 }, size)
        .with_camera(camera_rig)
        .draw(vao)
        .end_render();
}

const char* ParticleSim::curvature_vsh = R"(
layout (location = 1) in vec3 v_position;
layout (location = 2) in vec3 v_direction;
layout (location = 3) in float v_curvature_radius;

out VertexData
{
    vec3 position;
    vec3 direction;
    float radius;
};

void main()
{
    position = v_position;
    direction = v_direction;
    radius = v_curvature_radius;
}
)";

const char* ParticleSim::curvature_gsh = R"(
#define CIRCLE_SUBDIVISIONS 20

layout (points) in;
layout (line_strip, max_vertices=CIRCLE_SUBDIVISIONS) out;

layout (location = kProjectionMatrix) uniform mat4 mProj;
layout (location = kViewMatrix) uniform mat4 mView;

in VertexData
{
    vec3 position;
    vec3 direction;
    float radius;
} v_data[];


vec3 random_perpendicular(vec3 v) {
    return v.z < v.x ? vec3(v.y,-v.x,0) : vec3(0,-v.z,v.y);
}

mat4 rotationMatrix(vec3 axis, float angle) {
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;
    
    return mat4(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,  0.0,
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,  0.0,
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c,           0.0,
                0.0,                                0.0,                                0.0,                                1.0);
}

vec3 rotate(vec3 v, vec3 axis, float angle) {
    mat4 m = rotationMatrix(axis, angle);
    return (m * vec4(v, 1.0)).xyz;
}

void main()
{
    vec3 p = v_data[0].position;
    vec3 n = normalize(v_data[0].direction);
    vec3 r = v_data[0].radius * normalize(random_perpendicular(n));

    for (int i = 0; i <= CIRCLE_SUBDIVISIONS; i++) {
        float ang = 3.14159265 * 2.0 / (CIRCLE_SUBDIVISIONS-1) * i;
        vec4 rot = vec4(rotate(r, n, ang) + p, 1.0);
        gl_Position = mProj * mView * rot;
        EmitVertex();
    }

    EndPrimitive();
}
)";

const char* ParticleSim::curvature_fsh = R"(
out vec4 f_color;

void main()
{
    f_color = vec4(1.0, 0.0, 0.0, 1.0);
}
)";
