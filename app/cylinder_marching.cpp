#include "cylinder_marching.hpp"
#include "components.hpp"
#include "resources.hpp"
#include "render.hpp"
#include <future>
#include <gfx/imgui/imgui.h>
#include <gfx/vertex_array.hpp>
#include <gfx/render_pass.hpp>
#include <queue>
#include <glm/gtc/type_ptr.hpp>

CylinderMarching::CylinderMarching(entt::registry& _reg)
    : reg(_reg)
{
    target = reg.ctx<SelectedEntity>().selected;

    if (reg.valid(target) && reg.all_of<PointCloud, PointNormals>(target)) {
        cloud = &reg.get<PointCloud>(target);
        normals = &reg.get<PointNormals>(target);

    } else {
        throw std::runtime_error("Selected entity must have PointCloud and PointNormals");
    }
}

GuiState CylinderMarching::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Cylinder marching");
    if (ImGui::BeginPopupModal("Cylinder marching", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {

        ImGui::Separator();

        ImGui::InputFloat("Epsilon", &params.epsilon, 0.05, 0.1);
        ImGui::InputFloat("Normal Threshold", &params.normal_threshold, 0.1, 0.5);
        ImGui::InputFloat("Cluster epsilon", &params.cluster_epsilon);
        ImGui::InputInt("Min points", (int*)&params.min_points);
        ImGui::InputFloat("Missing probability", &params.probability);

        ImGui::Separator();

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState CylinderMarching::execute()
{
    groot::compute_cylinders(cloud->cloud.data(), normals->normals.data(), cloud->cloud.size(), result, params);
    return CommandState::Ok;
}

void CylinderMarching::on_finish()
{
    reg.emplace_or_replace<Cylinders>(target, std::move(result));
}

namespace cylinder_view_system {

struct CylinderViewComponent {
    CylinderViewComponent();

    gfx::VertexArray vao;
    gfx::Buffer<groot::Cylinder> cylinders;
    gfx::Uniform<Color> color;
};

CylinderViewComponent::CylinderViewComponent()
    : vao()
    , cylinders()
    , color({1.0, 0.0, 0.0})
{
    vao
        .add_buffer(cylinder_layout, cylinders)
        .set_mode(gfx::Mode::Points);
}

struct SystemData {
    gfx::RenderPipeline pipeline;
};

void update_cylinder_view(entt::registry& reg, entt::entity entity)
{
    auto [cylinders, view_data] = reg.get<Cylinders, CylinderViewComponent>(entity);

    auto edit = view_data.cylinders.edit(false);

    edit.vector().clear();

    std::copy(cylinders.cylinders.begin(), cylinders.cylinders.end(), std::back_inserter(edit.vector()));

    view_data.vao.set_element_count(edit.vector().size());
}

void init(entt::registry& reg)
{
    auto& shaders = reg.ctx<ShaderCollection>();
    reg.set<SystemData>(SystemData {
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::Cylinders))
            .build()
        });

    reg.on_destroy<Cylinders>().connect<&entt::registry::remove<CylinderViewComponent>>();
    reg.on_construct<Cylinders>().connect<&entt::registry::emplace<CylinderViewComponent>>();
    reg.on_construct<CylinderViewComponent>().connect<&entt::registry::emplace_or_replace<Visible>>();

    reg.on_construct<Cylinders>().connect<&update_cylinder_view>();
    reg.on_update<Cylinders>().connect<&update_cylinder_view>();


    auto& entity_editor = reg.ctx<EntityEditor>();
    entity_editor.registerComponent<CylinderViewComponent>("Cylinder View");
}

void run(entt::registry& reg)
{
    auto& view_data = reg.ctx<RenderData>();
    auto& system_data = reg.ctx<SystemData>();

    auto views = reg.view<CylinderViewComponent, Visible>().each();

    gfx::RenderPass(view_data.framebuffer, gfx::ClearOperation::nothing())
        .viewport({ 0, 0 }, view_data.size)
        .set_pipeline(system_data.pipeline)
        .with_camera(*view_data.camera)
        .for_each(views.begin(), views.end(), [](auto& pipe, auto&& entity) {
            auto [_, view] = entity;
            pipe
                .bind(view.color)
                .draw(view.vao);
        })
        .end_pipeline();
}

}

namespace MM {

template <>
void ComponentEditorWidget<cylinder_view_system::CylinderViewComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<cylinder_view_system::CylinderViewComponent>(e);

    ImGui::ColorEdit3("Cylinder color", glm::value_ptr(*t.color));
}

}