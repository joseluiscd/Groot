#include <gfx/buffer.hpp>
#include <gfx/imgui/imgui.h>
#include <gfx/render_pass.hpp>
#include <gfx/vertex_array.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <groot/cloud.hpp>
#include <groot_app/cloud_system.hpp>
#include <groot_app/components.hpp>
#include <groot_app/render.hpp>
#include <groot_app/resources.hpp>
#include <groot_app/viewer_system.hpp>
#include <groot_graph/cylinder_marching.hpp>
#include <spdlog/spdlog.h>

ComputeNormals::ComputeNormals(entt::handle h)
    : target(h)
{
    if (!h.valid() || !h.all_of<PointCloud>()) {
        throw std::runtime_error("Selected entity must have PointCloud");
    }
}

void ComputeNormals::draw_dialog()
{
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
}

void ComputeNormals::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskManager>().push_task("Computing Cloud Normals", compute_normals_command(entt::handle(reg, target), k, radius));
}

async::task<void> compute_normals_command(entt::handle e, size_t k, float radius)
{
    return create_task()
        .then_sync([e]() {
            if (!e.valid() || !e.all_of<PointCloud>()) {
                throw std::runtime_error("Selected entity must have PointCloud");
            }

            return &e.get<PointCloud>();
        })
        .then_async([k, radius](PointCloud* cloud) {
            return PointNormals { groot::compute_normals(cloud->cloud.data(), cloud->cloud.size(), k, radius) };
        })
        .then_sync([e](PointNormals&& normals) {
            e.emplace_or_replace<PointNormals>(std::move(normals));
        });
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
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState RecenterCloud::execute()
{
    this->centered = *this->cloud;
    groot::recenter_cloud_centroid(centered.cloud.data(), centered.cloud.size());
    return CommandState::Ok;
}

void RecenterCloud::on_finish(entt::registry& reg)
{
    reg.emplace_or_replace<PointCloud>(target, std::move(centered));
}

struct SplitCloudCommandData {
    std::string name;
    PointCloud* cloud;
    PointNormals* normals;
    PointColors* colors;
};

struct SplitCloudResultData {
    std::vector<PointCloud> clouds {};
    std::vector<PointNormals> normals {};
    std::vector<PointColors> colors {};
    std::vector<std::string> names {};
};

async::task<std::vector<entt::handle>> split_cloud_command(entt::handle h, float voxel_size)
{
    return create_task()
        .then_sync([h]() -> SplitCloudCommandData {
            SplitCloudCommandData data;

            Name* maybe_name = h.try_get<Name>();

            data.cloud = require_components<PointCloud>(h);
            data.normals = h.try_get<PointNormals>();
            data.colors = h.try_get<PointColors>();
            data.name = maybe_name ? maybe_name->name : "cloud";

            return data;
        })
        .then_async([voxel_size](SplitCloudCommandData&& data) {
            PointCloud* cloud = data.cloud;
            PointNormals* normals = data.normals;
            PointColors* colors = data.colors;
            std::string name = std::move(data.name);

            SplitCloudResultData result {};

            groot::VoxelGrid grid = groot::voxel_grid(cloud->cloud.data(), cloud->cloud.size(), voxel_size);
            for (size_t i = 0; i < grid.voxels.size(); i++) {

                if (!grid.voxels[i].empty()) {
                    std::stringstream namestream;
                    namestream << name << "_" << i;
                    result.names.emplace_back(namestream.str());

                    PointCloud& current_cloud = result.clouds.emplace_back();
                    for (size_t j = 0; j < grid.voxels[i].size(); j++) {
                        current_cloud.cloud.push_back(cloud->cloud[grid.voxels[i][j]]);
                    }

                    if (normals) {
                        PointNormals& current_normals = result.normals.emplace_back();
                        for (size_t j = 0; j < grid.voxels[i].size(); j++) {
                            current_normals.normals.push_back(normals->normals[grid.voxels[i][j]]);
                        }
                    }

                    if (colors) {
                        PointColors& current_colors = result.colors.emplace_back();
                        for (size_t j = 0; j < grid.voxels[i].size(); j++) {
                            current_colors.colors.push_back(colors->colors[grid.voxels[i][j]]);
                        }
                    }
                }
            }
            return result;
        })
        .then_sync([&reg = *h.registry()](SplitCloudResultData&& result) {
            std::vector<entt::handle> entities;
            for (size_t i = 0; i < result.clouds.size(); i++) {
                entt::entity entity = reg.create();

                reg.emplace<Name>(entity, result.names[i]);

                reg.emplace<PointCloud>(entity, std::move(result.clouds[i]));
                if (!result.normals.empty()) {
                    reg.emplace<PointNormals>(entity, std::move(result.normals[i]));
                }

                if (!result.colors.empty()) {
                    reg.emplace<PointColors>(entity, std::move(result.colors[i]));
                }

                entities.emplace_back(reg, entity);
            }
            return entities;
        });
}

void SplitCloudGui::draw_dialog()
{
    ImGui::InputFloat("Voxel size", &this->voxel_size, 0.5f, 0.1f);
}

void SplitCloudGui::schedule_commands(entt::registry& reg)
{
    reg.ctx<TaskManager>().push_task(
        "Splitting cloud",
        split_cloud_command(target, voxel_size));
}

namespace cloud_view_system {

entt::observer cloud_modified;
entt::observer normals_modified;

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
    NormalViewComponent(NormalViewComponent&& other) noexcept = default;
    NormalViewComponent& operator=(NormalViewComponent&& other) noexcept = default;

    gfx::VertexArray vao;
    gfx::Buffer<glm::vec3> directions;

    gfx::Uniform<Color> color;
    gfx::Uniform<VectorSize> vector_size;
};

struct CurvatureViewComponent {
    CurvatureViewComponent(PointViewComponent& points);
    CurvatureViewComponent(CurvatureViewComponent&& other) noexcept = default;
    CurvatureViewComponent& operator=(CurvatureViewComponent&& other) noexcept = default;

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
    if (!reg.all_of<PointViewComponent, PointNormals>(entity)) {
        return;
    }

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

void init(entt::registry& reg)
{
    auto& shaders = reg.ctx<ShaderCollection>();
    reg.set<SystemData>(SystemData {
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::ShaderID::Points))
            .build(),
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::ShaderID::PointsColor))
            .build(),
        gfx::RenderPipeline::Builder()
            .with_shader(shaders.get_shader(ShaderCollection::ShaderID::Vectors))
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

    cloud_modified.connect(reg,
        entt::collector
            .group<PointCloud>()
            .update<PointCloud>()
            .update<PointColors>());

    normals_modified.connect(reg,
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

void deinit(entt::registry& reg)
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

void run(entt::registry& reg)
{
    auto& view_data = reg.ctx<RenderData>();
    auto& system_data = reg.ctx<SystemData>();

    auto point_views = reg.view<PointViewComponent, Visible>().each();
    auto normal_views = reg.view<NormalViewComponent, Visible>().each();

    cloud_modified.each([&reg](auto entity) {
        if (!reg.all_of<PointViewComponent>(entity)) {
            reg.emplace<PointViewComponent>(entity);
        }
        update_cloud_view(reg, entity);
    });

    normals_modified.each([&reg](auto entity) {
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

}

namespace MM {

template <>
void ComponentEditorWidget<cloud_view_system::PointViewComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<cloud_view_system::PointViewComponent>(e);

    ImGui::DragFloat("Point size", &*t.size, 0.05f, 0.0f, INFINITY);

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
void ComponentEditorWidget<cloud_view_system::NormalViewComponent>(entt::registry& reg, entt::registry::entity_type e)
{
    auto& t = reg.get<cloud_view_system::NormalViewComponent>(e);

    ImGui::ColorEdit3("Line color", glm::value_ptr(*t.color));
    ImGui::DragFloat("Line size", &*t.vector_size, 0.01f, 0.0f, INFINITY);
}

template <>
void ComponentAddAction<cloud_view_system::PointViewComponent>(entt::registry& reg, entt::entity entity)
{
    if (reg.all_of<PointCloud>(entity)) {
        reg.emplace<cloud_view_system::PointViewComponent>(entity);
        cloud_view_system::update_cloud_view(reg, entity);
    }
}

template <>
void ComponentAddAction<cloud_view_system::NormalViewComponent>(entt::registry& reg, entt::entity entity)
{
    if (reg.all_of<PointNormals, cloud_view_system::PointViewComponent>(entity)) {
        cloud_view_system::create_normal_view(reg, entity);
    }
}

}