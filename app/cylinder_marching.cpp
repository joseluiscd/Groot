#include "cylinder_marching.hpp"
#include "components.hpp"
#include "groot/cylinder_marching.hpp"
#include "render.hpp"
#include "resources.hpp"
#include <future>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iterator>
#include <queue>

CylinderMarching::CylinderMarching(entt::registry& _reg)
    : CylinderMarching(entt::handle{
        _reg,
        _reg.ctx<SelectedEntity>().selected
    })
{
}

CylinderMarching::CylinderMarching(entt::handle&& handle)
    : reg(*handle.registry())
{
    target = handle.entity();

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
        ImGui::InputInt("Min points", &min_points);
        ImGui::Separator();

        ImGui::InputFloat("Epsilon", &epsilon, 0.05, 0.1);
        ImGui::InputFloat("Sampling resolution", &sampling, 0.05, 0.1);
        ImGui::InputFloat("Normal Threshold", &normal_deviation, 1.0, 5.0);
        ImGui::InputFloat("Missing probability", &overlook_probability, 0.01, 0.1);
        ImGui::InputFloat("Voxel size", &voxel_size, 0.1, 0.5);

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
    groot::Ransac::Parameters params;
    params.cluster_epsilon = sampling;
    params.epsilon = epsilon;
    params.min_points = min_points;
    params.normal_threshold = std::cos(normal_deviation * M_PI / 180.0);
    params.probability = overlook_probability;

    result = groot::compute_cylinders_voxelized(cloud->cloud.data(), normals->normals.data(), cloud->cloud.size(), voxel_size, params);
    return CommandState::Ok;
}

void CylinderMarching::on_finish()
{
    reg.emplace_or_replace<Cylinders>(target, std::move(result));
}

CylinderFilter::CylinderFilter(entt::registry& _reg)
    : CylinderFilter(entt::handle{
        _reg,
        _reg.ctx<SelectedEntity>().selected
    })
{
}

CylinderFilter::CylinderFilter(entt::handle&& handle)
    : reg(*handle.registry())
{
    target = handle.entity();
    if (reg.valid(target) && reg.all_of<Cylinders>(target)) {
    } else {
        throw std::runtime_error("Selected entity must have Cylinders component");
    }
}
GuiState CylinderFilter::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Cylinder filter");
    if (ImGui::BeginPopupModal("Cylinder filter", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::Checkbox("Filter by radius", &filter_radius);

        if (filter_radius) {
            ImGui::InputFloat2("Radius range", radius_range);
            if (radius_range[1] < radius_range[0]) {
                radius_range[1] = radius_range[0];
            }
        }

        ImGui::Checkbox("Filter by length", &filter_length);
        if (filter_length) {
            ImGui::InputFloat2("Length range", length_range);
            if (length_range[1] < length_range[0]) {
                length_range[1] = length_range[0];
            }
        }

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiState::RunSync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState CylinderFilter::execute()
{
    reg.patch<Cylinders>(target, [&](Cylinders& cylinders) {
        std::vector<groot::CylinderWithPoints> new_cylinders;
        for (auto it = cylinders.cylinders.begin(); it != cylinders.cylinders.end(); ++it) {
            const float radius = it->cylinder.radius;
            const float length = it->cylinder.middle_height * 2;

            if ((!filter_radius
                    || (radius_range[0] < radius && radius < radius_range[1]))
                && (!filter_length
                    || (length_range[0] < length && length < length_range[1]))) {
            
                new_cylinders.push_back(*it);
            }
        }

        cylinders.cylinders.swap(new_cylinders);
    });
    return CommandState::Ok;
}

CylinderPointFilter::CylinderPointFilter(entt::handle&& handle)
    : reg(*handle.registry())
{
    target = handle.entity();
    if (reg.valid(target) && reg.all_of<Cylinders>(target)) {
        cylinders = &reg.get<Cylinders>(target);
    } else {
        throw std::runtime_error("Selected entity must have Cylinders component");
    }
}

CommandState CylinderPointFilter::execute()
{
    for (size_t i = 0; i < cylinders->cylinders.size(); i++) {
        groot::CylinderWithPoints& cylinder = cylinders->cylinders[i];

        std::copy(cylinder.points.begin(), cylinder.points.end(), std::back_inserter(cloud.cloud));
    }
    return CommandState::Ok;
}

void CylinderPointFilter::on_finish()
{
    reg.emplace_or_replace<PointCloud>(target, std::move(cloud));
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
    , color({ 1.0, 0.0, 0.0 })
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
    auto& view_data = reg.get_or_emplace<CylinderViewComponent>(entity);
    auto& cylinders = reg.get<Cylinders>(entity);

    auto edit = view_data.cylinders.edit(false);

    edit.vector().clear();

    for (groot::CylinderWithPoints& c: cylinders.cylinders) {
        edit.vector().push_back(c.cylinder);
    }

    view_data.vao.set_element_count(edit.vector().size());
}

void init(entt::registry& reg)
{
    auto& shaders = reg.ctx<ShaderCollection>();
    reg.set<SystemData>(SystemData {
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::Cylinders))
            .build() });

    reg.on_destroy<Cylinders>().connect<&entt::registry::remove<CylinderViewComponent>>();
    reg.on_construct<Cylinders>().connect<&entt::registry::emplace_or_replace<CylinderViewComponent>>();
    reg.on_construct<CylinderViewComponent>().connect<&entt::registry::emplace_or_replace<Visible>>();

    reg.on_construct<Cylinders>().connect<&update_cylinder_view>();
    reg.on_update<Cylinders>().connect<&update_cylinder_view>();

    auto& entity_editor = reg.ctx<EntityEditor>();
    entity_editor.registerComponent<CylinderViewComponent>("Cylinder View");

    reg.view<Cylinders>().each([&](entt::entity e, const auto& _){
        update_cylinder_view(reg, e);
    });
}

void deinit(entt::registry &reg)
{
    reg.on_destroy<Cylinders>().disconnect<&entt::registry::remove<CylinderViewComponent>>();
    reg.on_construct<Cylinders>().disconnect<&entt::registry::emplace_or_replace<CylinderViewComponent>>();
    reg.on_construct<CylinderViewComponent>().disconnect<&entt::registry::emplace_or_replace<Visible>>();

    reg.on_construct<Cylinders>().disconnect<&update_cylinder_view>();
    reg.on_update<Cylinders>().disconnect<&update_cylinder_view>();

    reg.clear<CylinderViewComponent>();
    reg.unset<SystemData>();
}

void run(entt::registry& reg)
{
    auto& view_data = reg.ctx<RenderData>();
    auto& system_data = reg.ctx<SystemData>();

    auto views = reg.view<CylinderViewComponent, Visible>().each();
    if (views.begin() == views.end()) {
        return;
    }

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