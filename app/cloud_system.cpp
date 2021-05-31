#include "cloud_system.hpp"
#include "components.hpp"
#include "entt/entity/fwd.hpp"
#include "render.hpp"
#include "resources.hpp"
#include "viewer_system.hpp"
#include <gfx/buffer.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <groot/cloud.hpp>
#include <groot/cylinder_marching.hpp>
#include <spdlog/spdlog.h>

ComputeNormals::ComputeNormals(entt::handle&& handle)
    : registry(*handle.registry())
{
    target = handle.entity();
    if (registry.valid(target) && registry.all_of<PointCloud>(target)) {
        this->cloud = &registry.get<PointCloud>(target);
    } else {
        throw std::runtime_error("Selected entity must have PointCloud");
    }
}

ComputeNormals::ComputeNormals(entt::registry& reg)
    : ComputeNormals(entt::handle {
        reg,
        reg.ctx<SelectedEntity>().selected })
{
}

GuiState ComputeNormals::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Compute normals");
    if (ImGui::BeginPopupModal("Compute normals", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::RadioButton("Radius search", &selected_k, 0);
        ImGui::RadioButton("kNN##normals", &selected_k, 1);

        ImGui::Separator();
        if (selected_k == 0) { // Radius search
            ImGui::InputFloat("Radius", &radius);
            ImGui::InputInt("Max NN", &k);
        } else {
            radius = 0.0;
            ImGui::InputInt("kNN", &k);
        }
        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState ComputeNormals::execute()
{
    normals = std::move(
        groot::compute_normals(cloud->cloud.data(), cloud->cloud.size(), k, radius));

    return CommandState::Ok;
}

void ComputeNormals::on_finish()
{
    registry.emplace_or_replace<PointNormals>(target, std::move(normals));
}

RecenterCloud::RecenterCloud(entt::registry& _reg)
    : reg(_reg)
{
    target = reg.ctx<SelectedEntity>().selected;

    if (reg.valid(target) && reg.all_of<PointCloud>(target)) {
        this->cloud = &reg.get<PointCloud>(target);
    } else {
        throw std::runtime_error("Selected entity must have PointCloud");
    }
}

GuiState RecenterCloud::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Compute normals");
    if (ImGui::BeginPopupModal("Compute normals", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::RadioButton("Using centroid##recenter", &selected, 0);
        ImGui::RadioButton("Using bounding box center##recenter", &selected, 1);

        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiState::RunSync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState RecenterCloud::execute()
{
    reg.patch<PointCloud>(target, [&](auto& cloud) {
        groot::recenter_cloud_centroid(cloud.cloud.data(), cloud.cloud.size());
    });
    return CommandState::Ok;
}

SplitCloud::SplitCloud(entt::handle&& _handle)
    : reg(*_handle.registry())
    , grid(0, 0, 0)
    , normals()
{
    target = _handle.entity();

    if (reg.valid(target) && reg.all_of<PointCloud>(target)) {
        this->cloud = &reg.get<PointCloud>(target);
    } else {
        throw std::runtime_error("Selected entity must have PointCloud");
    }
    if (reg.all_of<PointNormals>(target)) {
        this->normals = &reg.get<PointNormals>(target);
    }
}

GuiState SplitCloud::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Split Cloud in Voxels");
    if (ImGui::BeginPopupModal("Split Cloud in Voxels", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::InputFloat("Voxel size", &this->voxel_size, 0.5, 0.1);
        ImGui::Separator();

        if (ImGui::Button("Run")) {
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState SplitCloud::execute()
{
    this->grid = std::move(groot::voxel_grid(this->cloud->cloud.data(), cloud->cloud.size(), voxel_size));
    for (size_t i = 0; i < this->grid.voxels.size(); i++) {
        if (!this->grid.voxels[i].empty()) {
            PointCloud& current_cloud = this->result_clouds.emplace_back();
            for (size_t j = 0; j < this->grid.voxels[i].size(); j++) {
                current_cloud.cloud.push_back(cloud->cloud[this->grid.voxels[i][j]]);
            }

            if (normals) {
                PointNormals& current_normals = this->result_normals.emplace_back();
                for (size_t j = 0; j < this->grid.voxels[i].size(); j++) {
                    current_normals.normals.push_back((*normals)->normals[this->grid.voxels[i][j]]);
                }
            }
        }
    }

    return CommandState::Ok;
}

void SplitCloud::on_finish()
{
    for (size_t i = 0; i < result_clouds.size(); i++) {
        entt::entity entity = reg.create();
        reg.emplace<PointCloud>(entity, std::move(result_clouds[i]));
        if (normals) {
            reg.emplace<PointNormals>(entity, std::move(result_normals[i]));
        }

        result.emplace_back(reg, entity);
    }
}

namespace cloud_view_system {

struct PointViewComponent {
    PointViewComponent();

    gfx::VertexArray vao;

    gfx::Buffer<glm::vec3> points;

    gfx::Uniform<Color> color;
    gfx::Uniform<PointSize> size;
};

PointViewComponent::PointViewComponent()
    : vao()
    , points()
    , color({ 1.0, 0.0, 0.0 })
    , size(4.0)
{
    vao
        .add_buffer(point_layout, points)
        .set_mode(gfx::Mode::Points);
}

struct NormalViewComponent {
    NormalViewComponent(PointViewComponent& points);
    NormalViewComponent(NormalViewComponent&& other) = default;
    NormalViewComponent& operator=(NormalViewComponent&& other) = default;

    gfx::VertexArray vao;
    gfx::Buffer<glm::vec3> directions;

    gfx::Uniform<Color> color;
};

NormalViewComponent::NormalViewComponent(PointViewComponent& points)
    : vao()
    , directions()
    , color({ 0.0, 0.0, 1.0 })
{
    vao
        .add_buffer(point_layout, points.points)
        .add_buffer(direction_layout, directions)
        .set_mode(gfx::Mode::Points);
}

struct SystemData {
    gfx::RenderPipeline pipeline_points;
    gfx::RenderPipeline pipeline_vectors;
};

void update_cloud_view(entt::registry& registry, entt::entity entity)
{
    auto [cloud, view_data] = registry.get<PointCloud, PointViewComponent>(entity);

    auto edit_points = view_data.points.edit(false);

    edit_points.vector().clear();

    for (auto it = cloud.cloud.begin(); it != cloud.cloud.end(); ++it) {
        edit_points.vector().push_back(glm::vec3(it->x(), it->y(), it->z()));
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

void init(entt::registry& reg)
{
    auto& shaders = reg.ctx<ShaderCollection>();
    reg.set<SystemData>(SystemData {
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::Points))
            .build(),
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::Vectors))
            .build() });
    // Destroy the views
    reg.on_destroy<PointCloud>().connect<&entt::registry::remove<PointViewComponent>>();
    reg.on_destroy<PointNormals>().connect<&entt::registry::remove<NormalViewComponent>>();
    reg.on_destroy<PointViewComponent>().connect<&entt::registry::remove<NormalViewComponent>>();

    // Create/update the views
    reg.on_construct<PointCloud>().connect<&entt::registry::emplace<PointViewComponent>>();
    reg.on_construct<PointCloud>().connect<&update_cloud_view>();
    reg.on_construct<PointCloud>().connect<&entt::registry::emplace_or_replace<Visible>>();
    reg.on_update<PointCloud>().connect<&update_cloud_view>();

    // Tie the normals to the cloud
    reg.on_construct<PointCloud>().connect<&entt::registry::remove<PointNormals>>();
    reg.on_update<PointCloud>().connect<&entt::registry::remove<PointNormals>>();
    reg.on_destroy<PointCloud>().connect<&entt::registry::remove<PointNormals>>();

    reg.on_construct<PointNormals>().connect<&create_normal_view>();
    reg.on_construct<PointNormals>().connect<&entt::registry::emplace_or_replace<Visible>>();
    reg.on_update<PointNormals>().connect<&create_normal_view>();

    auto& entity_editor = reg.ctx<EntityEditor>();
    entity_editor.registerComponent<PointViewComponent>("PointCloud View");
    entity_editor.registerComponent<NormalViewComponent>("PointCloud Normal View");
}

void run(entt::registry& reg)
{
    auto& view_data = reg.ctx<RenderData>();
    auto& system_data = reg.ctx<SystemData>();

    auto point_views = reg.view<PointViewComponent, Visible>().each();
    auto normal_views = reg.view<NormalViewComponent, Visible>().each();

    gfx::RenderPass(view_data.framebuffer, gfx::ClearOperation::nothing())
        .viewport({ 0, 0 }, view_data.size)
        .set_pipeline(system_data.pipeline_points)
        .with_camera(*view_data.camera)
        .for_each(point_views.begin(), point_views.end(), [](auto& pipe, auto&& entity) {
            auto [_, view] = entity;
            pipe
                .bind(view.size)
                .bind(view.color)
                .draw(view.vao);
        })
        .set_pipeline(system_data.pipeline_vectors)
        .with_camera(*view_data.camera)
        .for_each(normal_views.begin(), normal_views.end(), [](auto& pipe, auto&& entity) {
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
void ComponentEditorWidget<cloud_view_system::PointViewComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<cloud_view_system::PointViewComponent>(e);

    ImGui::ColorEdit3("Point color", glm::value_ptr(*t.color));
    ImGui::InputFloat("Point size", &*t.size);
}

template <>
void ComponentEditorWidget<cloud_view_system::NormalViewComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<cloud_view_system::NormalViewComponent>(e);

    ImGui::ColorEdit3("Line color", glm::value_ptr(*t.color));
}

template <>
void ComponentAddAction<cloud_view_system::NormalViewComponent>(entt::registry& reg, entt::entity entity)
{
    if (reg.all_of<PointNormals, cloud_view_system::PointViewComponent>(entity)) {
        cloud_view_system::create_normal_view(reg, entity);
    }
}

}