#define _USE_MATH_DEFINES
#include <future>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <groot_app/components.hpp>
#include <groot_app/cylinder_marching.hpp>
#include <groot_app/render.hpp>
#include <groot_app/resources.hpp>
#include <groot_graph/cylinder_marching.hpp>
#include <iterator>
#include <queue>

async::task<void> cylinder_marching_command(entt::handle h, const groot::Ransac::Parameters& params, float voxel_size)
{
    return create_task()
        .then_sync([h]() {
            return require_components<PointCloud, PointNormals>(h);
        })
        .then_async([params, voxel_size](std::tuple<PointCloud*, PointNormals*>&& input) {
            PointCloud* cloud = std::get<0>(input);
            PointNormals* normals = std::get<1>(input);

            return groot::compute_cylinders_voxelized_curvature(
                cloud->cloud.data(),
                normals->normals.data(),
                cloud->cloud.size(),
                voxel_size, params);
        })
        .then_sync([h](std::vector<groot::CylinderWithPoints>&& result) {
            h.emplace_or_replace<Cylinders>(std::move(result));
        });
}

async::task<void> cylinder_filter_command(entt::handle h, const CylinderFilterParams& params)
{
    return create_task()
        .then_sync([h]() {
            return require_components<Cylinders>(h);
        })
        .then_async([params](const Cylinders* cylinders) {
            Cylinders new_cylinders;

            for (auto it = cylinders->cylinders.begin(); it != cylinders->cylinders.end(); ++it) {
                const float radius = it->cylinder.radius;
                const float length = it->cylinder.middle_height * 2;

                if ((!params.radius
                        || (params.radius_range[0] < radius && radius < params.radius_range[1]))
                    && (!params.length
                        || (params.length_range[0] < length && length < params.length_range[1]))) {

                    new_cylinders.cylinders.push_back(*it);
                }
            }

            return new_cylinders;
        })
        .then_sync([h](Cylinders&& cylinders) {
            h.emplace_or_replace<Cylinders>(std::move(cylinders));
        });
}

async::task<void> cylinder_point_filter_command(entt::handle h)
{
    // TODO: Take into account Normals and colors
    return create_task()
        .then_sync([h]() {
            return require_components<Cylinders>(h);
        })
        .then_async([](const Cylinders* cylinders) {
            PointCloud cloud;
            for (size_t i = 0; i < cylinders->cylinders.size(); i++) {
                const groot::CylinderWithPoints& cylinder = cylinders->cylinders[i];

                std::copy(cylinder.points.begin(), cylinder.points.end(), std::back_inserter(cloud.cloud));
            }
            return cloud;
        })
        .then_sync([h](PointCloud&& cloud) {
            h.emplace_or_replace<PointCloud>(std::move(cloud));
        });
}

void CylinderMarching::draw_dialog()
{
    ImGui::Separator();
    ImGui::InputInt("Min points", &min_points);
    ImGui::Separator();

    ImGui::InputFloat("Epsilon", &epsilon, 0.05f, 0.1f);
    ImGui::InputFloat("Sampling resolution", &sampling, 0.05f, 0.1f);
    ImGui::InputFloat("Normal Threshold", &normal_deviation, 1.0f, 5.0f);
    ImGui::InputFloat("Missing probability", &overlook_probability, 0.01f, 0.1f);
    ImGui::InputFloat("Voxel size", &voxel_size, 0.1f, 0.5f);
}

void CylinderMarching::schedule_commands(entt::registry& reg)
{
    groot::Ransac::Parameters params;
    params.cluster_epsilon = sampling;
    params.epsilon = epsilon;
    params.min_points = min_points;
    params.normal_threshold = std::cos(normal_deviation * M_PI / 180.0);
    params.probability = overlook_probability;

    return reg.ctx<TaskBroker>().push_task(
        "Detecting Cylinders",
        cylinder_marching_command(target, params, voxel_size));
}

void CylinderFilter::draw_dialog()
{
    ImGui::Checkbox("Filter by radius", &params.radius);

    if (params.radius) {
        ImGui::InputFloat2("Radius range", params.radius_range);
        if (params.radius_range[1] < params.radius_range[0]) {
            params.radius_range[1] = params.radius_range[0];
        }
    }

    ImGui::Checkbox("Filter by length", &params.length);
    if (params.length) {
        ImGui::InputFloat2("Length range", params.length_range);
        if (params.length_range[1] < params.length_range[0]) {
            params.length_range[1] = params.length_range[0];
        }
    }
}

void CylinderFilter::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskBroker>().push_task("Filtering Cylinders", cylinder_filter_command(target, params));
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

    for (groot::CylinderWithPoints& c : cylinders.cylinders) {
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
    entity_editor.registerComponent<CylinderViewComponent>("Cylinder View", true);

    reg.view<Cylinders>().each([&](entt::entity e, const auto& _) {
        update_cylinder_view(reg, e);
    });
}

void deinit(entt::registry& reg)
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