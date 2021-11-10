#include "cloud_system.hpp"
#include "components.hpp"
#include "entt/entity/fwd.hpp"
#include "render.hpp"
#include "resources.hpp"
#include "viewer_system.hpp"
#include <cfloat>
#include <gfx/buffer.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <groot/cloud.hpp>
#include <groot/cylinder_marching.hpp>
#include <spdlog/spdlog.h>

// Compute normals
// ----------------------

void ComputeNormals::draw_gui(Cmd& cmd)
{
    ImGui::RadioButton("Radius search", &cmd.selected_k, 0);
    ImGui::RadioButton("kNN##normals", &cmd.selected_k, 1);

    ImGui::Separator();
    if (cmd.selected_k == 0) { // Radius search
        ImGui::InputFloat("Radius", &cmd.radius);
        ImGui::InputInt("Max NN", &cmd.k);
    } else {
        cmd.radius = 0.0;
        ImGui::InputInt("kNN", &cmd.k);
    }
}

PointNormals ComputeNormals::update_async(const Cmd& cmd, const PointCloud& cloud)
{
    return PointNormals { groot::compute_normals(cloud.cloud.data(), cloud.cloud.size(), cmd.k, cmd.radius) };
}

void ComputeNormals::update_sync(entt::handle h, PointNormals&& normals)
{
    h.emplace_or_replace<PointNormals>(std::move(normals));
}

void RecenterCloud::draw_gui(Cmd& cmd)
{
    if (ImGui::RadioButton("Centroid", cmd.mode == RecenterCloudCmd::Centroid)) cmd.mode = RecenterCloudCmd::Centroid;
    if (ImGui::RadioButton("Bounding Box Center", cmd.mode == RecenterCloudCmd::BoundCenter)) cmd.mode = RecenterCloudCmd::BoundCenter;
}

PointCloud RecenterCloud::update_async(const Cmd& cmd, const PointCloud& cloud)
{
    PointCloud new_cloud(cloud);
    groot::recenter_cloud_centroid(new_cloud.cloud.data(), new_cloud.cloud.size());
    return new_cloud;
}

void RecenterCloud::update_sync(entt::handle h, PointCloud&& new_cloud)
{
    h.emplace_or_replace<PointCloud>(std::move(new_cloud));
}

// Split cloud
// ----------------------

void SplitCloud::draw_gui(Cmd& cmd)
{
    ImGui::InputFloat("Voxel size", &cmd.voxel_size, 0.5, 0.1);
}

SplitCloudResult SplitCloud::update_async(const Cmd& cmd, const PointCloud& cloud, const PointNormals* normals, const PointColors* colors)
{
    groot::VoxelGrid grid = groot::voxel_grid(cloud.cloud.data(), cloud.cloud.size(), cmd.voxel_size);

    SplitCloudResult result;

    for (size_t i = 0; i < grid.voxels.size(); i++) {
        if (!grid.voxels[i].empty()) {
            PointCloud& current_cloud = result.clouds.emplace_back();
            for (size_t j = 0; j < grid.voxels[i].size(); j++) {
                current_cloud.cloud.push_back(cloud.cloud[grid.voxels[i][j]]);
            }

            if (normals != nullptr) {
                PointNormals& current_normals = result.normals.emplace_back();
                for (size_t j = 0; j < grid.voxels[i].size(); j++) {
                    current_normals.normals.push_back(normals->normals[grid.voxels[i][j]]);
                }
            }

            if (colors != nullptr) {
                PointColors& current_colors = result.colors.emplace_back();
                for (size_t j = 0; j < grid.voxels[i].size(); j++) {
                    current_colors.colors.push_back(colors->colors[grid.voxels[i][j]]);
                }
            }
        }
    }

    return result;
}

void SplitCloud::update_sync(entt::handle h, SplitCloudResult&& split)
{
    entt::registry& reg = *h.registry();

    for (size_t i = 0; i < split.clouds.size(); i++) {
        entt::entity entity = reg.create();

        reg.emplace<PointCloud>(entity, std::move(split.clouds[i]));

        if (!split.normals.empty()) {
            reg.emplace<PointNormals>(entity, std::move(split.normals[i]));
        }

        if (!split.colors.empty()) {
            reg.emplace<PointColors>(entity, std::move(split.colors[i]));
        }
    }
}

// Cloud viewer
// ---------------

struct CloudViewer::Impl {
    entt::observer cloud_modified;
    entt::observer normals_modified;
};

struct PointViewComponent {
    PointViewComponent();

    gfx::VertexArray vao;
    gfx::VertexArray vao_colors;

    gfx::Buffer<glm::vec3> points;
    gfx::Buffer<glm::vec3> colors;

    gfx::Uniform<Color> color;
    gfx::Uniform<PointSize> size;

    bool uniform_color = false;
    bool cloud_has_colors = false;
};

PointViewComponent::PointViewComponent()
    : vao()
    , vao_colors()
    , points()
    , colors()
    , color({ 1.0, 0.0, 0.0 })
    , size(4.0)
{
    vao
        .add_buffer(point_layout, points)
        .set_mode(gfx::Mode::Points);

    vao_colors
        .add_buffer(point_layout, points)
        .add_buffer(color_layout, colors)
        .set_mode(gfx::Mode::Points);
}

struct NormalViewComponent {
    NormalViewComponent(PointViewComponent& points);
    NormalViewComponent(NormalViewComponent&& other) = default;
    NormalViewComponent& operator=(NormalViewComponent&& other) = default;

    gfx::VertexArray vao;
    gfx::Buffer<glm::vec3> directions;

    gfx::Uniform<Color> color;
    gfx::Uniform<VectorSize> vector_size;
};

struct CurvatureViewComponent {
    CurvatureViewComponent(PointViewComponent& points);
    CurvatureViewComponent(CurvatureViewComponent&& other) = default;
    CurvatureViewComponent& operator=(CurvatureViewComponent&& other) = default;

    gfx::VertexArray vao;
    gfx::Buffer<glm::vec3> directions;
    gfx::Buffer<glm::vec3> normals;

    gfx::Uniform<Color> color;
    gfx::Uniform<VectorSize> vector_size;
};

NormalViewComponent::NormalViewComponent(PointViewComponent& points)
    : vao()
    , directions()
    , color({ 0.0, 0.0, 1.0 })
    , vector_size(1.0)
{
    vao
        .add_buffer(point_layout, points.points)
        .add_buffer(direction_layout, directions)
        .set_mode(gfx::Mode::Points);
}

CurvatureViewComponent::CurvatureViewComponent(PointViewComponent& points)
    : vao()
    , directions()
    , normals()
    , color({ 0.0, 0.0, 1.0 })
    , vector_size(1.0)
{
    vao
        .add_buffer(point_layout, points.points)
        .add_buffer(direction_layout, directions)
        .set_mode(gfx::Mode::Points);
}

struct SystemData {
    gfx::RenderPipeline pipeline_points;
    gfx::RenderPipeline pipeline_points_color;
    gfx::RenderPipeline pipeline_vectors;

    std::vector<entt::connection> connections;
};

void update_cloud_view(entt::registry& registry, entt::entity entity)
{
    auto& view_data = registry.emplace_or_replace<PointViewComponent>(entity);
    auto& cloud = registry.get<PointCloud>(entity);

    auto edit_points = view_data.points.edit(false);

    edit_points.vector().clear();

    for (auto it = cloud.cloud.begin(); it != cloud.cloud.end(); ++it) {
        edit_points.vector().push_back(glm::vec3(it->x(), it->y(), it->z()));
    }

    view_data.cloud_has_colors = registry.all_of<PointColors>(entity);
    if (view_data.cloud_has_colors) {
        auto colors = registry.get<PointColors>(entity);
        auto edit_colors = view_data.colors.edit(false);

        edit_colors.vector().clear();

        for (auto it = colors.colors.begin(); it != colors.colors.end(); ++it) {
            edit_colors.vector().push_back(glm::vec3(it->x(), it->y(), it->z()));
        }
        view_data.vao_colors.set_element_count(edit_points.vector().size());
    } else {
        view_data.uniform_color = true;
    }
    view_data.vao.set_element_count(edit_points.vector().size());
}

void create_normal_view(entt::registry& reg, entt::entity entity)
{
    auto& cloud_view = reg.get<PointViewComponent>(entity);
    reg.emplace_or_replace<NormalViewComponent>(entity, cloud_view);

    auto [normals, view_data] = reg.get<PointNormals, NormalViewComponent>(entity);

    auto edit_normals = view_data.directions.edit(false);

    edit_normals.vector().clear();

    for (auto it = normals.normals.begin(); it != normals.normals.end(); ++it) {
        edit_normals.vector().push_back(glm::vec3(it->x(), it->y(), it->z()));
    }

    view_data.vao.set_element_count(edit_normals.vector().size());
}

void create_curvature_view(entt::registry& reg, entt::entity entity)
{
    auto& cloud_view = reg.get<PointViewComponent>(entity);
    reg.emplace_or_replace<CurvatureViewComponent>(entity, cloud_view);

    auto [curvature, view_data] = reg.get<PointCurvature, CurvatureViewComponent>(entity);

    auto edit_curvature = view_data.directions.edit(false);

    edit_curvature.vector().clear();

    for (auto it = curvature.direction.begin(); it != curvature.direction.end(); ++it) {
        edit_curvature.vector().push_back(glm::vec3(it->direction.x(), it->direction.y(), it->direction.z()));
    }

    view_data.vao.set_element_count(edit_curvature.vector().size());
}

CloudViewer::CloudViewer()
    : _impl(new Impl)
{
}

CloudViewer::~CloudViewer()
{
}

void CloudViewer::init(entt::registry& reg)
{
    auto& shaders = reg.ctx<ShaderCollection>();
    auto& system_data = reg.set<SystemData>(SystemData {
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::Points))
            .build(),
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::PointsColor))
            .build(),
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::Vectors))
            .build(),
        {
            // Destroy the views
            reg.on_destroy<PointCloud>().connect<&entt::registry::remove<PointViewComponent>>(),
            reg.on_destroy<PointNormals>().connect<&entt::registry::remove<NormalViewComponent>>(),
            reg.on_destroy<PointCurvature>().connect<&entt::registry::remove<CurvatureViewComponent>>(),
            reg.on_destroy<PointViewComponent>().connect<&entt::registry::remove<NormalViewComponent>>(),
            reg.on_destroy<PointViewComponent>().connect<&entt::registry::remove<CurvatureViewComponent>>(),

            // Create the views
            reg.on_construct<PointCloud>().connect<&entt::registry::emplace_or_replace<Visible>>(),

            // Tie the normals to the cloud
            reg.on_construct<PointCloud>().connect<&entt::registry::remove<PointNormals>>(),
            reg.on_update<PointCloud>().connect<&entt::registry::remove<PointNormals>>(),
            reg.on_destroy<PointCloud>().connect<&entt::registry::remove<PointNormals>>(),
        } });

    _impl->cloud_modified.connect(reg,
        entt::collector
            .group<PointCloud>()
            .update<PointCloud>()
            .update<PointColors>());

    _impl->normals_modified.connect(reg,
        entt::collector
            .group<PointNormals>()
            .update<PointNormals>());

    auto& entity_editor = reg.ctx<EntityEditor>();
    entity_editor.registerComponent<PointViewComponent>("Point View", true);
    entity_editor.registerComponent<NormalViewComponent>("Normal View", true);

    reg.view<PointCloud>().each([&](entt::entity e, const auto& _) {
        reg.emplace<PointViewComponent>(e);
        update_cloud_view(reg, e);
    });

    reg.view<PointNormals>().each([&](entt::entity e, const auto& _) {
        create_normal_view(reg, e);
    });
}

void CloudViewer::clear(entt::registry& reg)
{
    reg.clear<PointViewComponent>();
    reg.clear<NormalViewComponent>();
    reg.clear<CurvatureViewComponent>();

    SystemData& d = reg.ctx<SystemData>();
    for (size_t i = 0; i < d.connections.size(); i++) {
        d.connections[i].release();
    }

    reg.unset<SystemData>();
}

void CloudViewer::update(entt::registry& reg)
{
    auto& view_data = reg.ctx<RenderData>();
    auto& system_data = reg.ctx<SystemData>();

    auto point_views = reg.view<PointViewComponent, Visible>().each();
    auto normal_views = reg.view<NormalViewComponent, Visible>().each();

    _impl->cloud_modified.each([&reg](auto entity) {
        if (!reg.all_of<PointViewComponent>(entity)) {
            reg.emplace<PointViewComponent>(entity);
        }
        update_cloud_view(reg, entity);
    });

    _impl->normals_modified.each([&reg](auto entity) {
        create_normal_view(reg, entity);
    });

    gfx::RenderPass(view_data.framebuffer, gfx::ClearOperation::nothing())
        .viewport({ 0, 0 }, view_data.size)
        .set_pipeline(system_data.pipeline_points)
        .with_camera(*view_data.camera)
        .for_each(point_views.begin(), point_views.end(), [](auto& pipe, auto&& entity) {
            auto [_, view] = entity;

            if (!view.cloud_has_colors || view.uniform_color) {
                pipe
                    .bind(view.size)
                    .bind(view.color)
                    .draw(view.vao);
            }
        })
        .set_pipeline(system_data.pipeline_points_color)
        .with_camera(*view_data.camera)
        .for_each(point_views.begin(), point_views.end(), [](auto& pipe, auto&& entity) {
            auto [_, view] = entity;
            if (view.cloud_has_colors && !view.uniform_color) {
                pipe
                    .bind(view.size)
                    .draw(view.vao_colors);
            }
        })
        .set_pipeline(system_data.pipeline_vectors)
        .with_camera(*view_data.camera)
        .for_each(normal_views.begin(), normal_views.end(), [](auto& pipe, auto&& entity) {
            auto [_, view] = entity;
            pipe
                .bind(view.color)
                .bind(view.vector_size)
                .draw(view.vao);
        })
        .end_pipeline();
}

namespace MM {

template <>
void ComponentEditorWidget<PointViewComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<PointViewComponent>(e);

    ImGui::DragFloat("Point size", &*t.size, 0.05, 0.0, INFINITY);

    if (t.cloud_has_colors) {
        ImGui::Checkbox("Show uniform color", &t.uniform_color);
        if (t.uniform_color) {
            ImGui::ColorEdit3("Point color", glm::value_ptr(*t.color));
        }
    } else {
        ImGui::ColorEdit3("Point color", glm::value_ptr(*t.color));
    }
}

template <>
void ComponentEditorWidget<NormalViewComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<NormalViewComponent>(e);

    ImGui::ColorEdit3("Line color", glm::value_ptr(*t.color));
    ImGui::DragFloat("Line size", &*t.vector_size, 0.01, 0.0, INFINITY);
}

template <>
void ComponentAddAction<PointViewComponent>(entt::registry& reg, entt::entity entity)
{
    if (reg.all_of<PointCloud>(entity)) {
        reg.emplace<PointViewComponent>(entity);
        update_cloud_view(reg, entity);
    }
}

template <>
void ComponentAddAction<NormalViewComponent>(entt::registry& reg, entt::entity entity)
{
    if (reg.all_of<PointNormals, PointViewComponent>(entity)) {
        create_normal_view(reg, entity);
    }
}

}