#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include "groot/cloud.hpp"
#include <groot_app/entt.hpp>
#include <groot/cgal.hpp>

GROOT_APP_API async::task<void> compute_normals_command(entt::handle e, size_t k, float radius);

class GROOT_APP_LOCAL ComputeNormals : public DialogGui {
public:
    ComputeNormals(entt::handle h);

    void draw_dialog() override;
    std::string_view name() const override { return "Compute Point Cloud Normals"; }
    void schedule_commands(entt::registry& reg) override;

private:
    entt::entity target;

    int selected_k = 1;

public:
    // Parameters

    int k = 0;
    float radius = 1.0;
};

class GROOT_APP_API RecenterCloud : public CommandGui {
public:
    RecenterCloud(entt::registry& reg);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish(entt::registry& reg) override;

private:
    entt::registry& reg;
    PointCloud* cloud;
    entt::entity target;

    PointCloud centered;

    int selected;
};

class GROOT_APP_API SplitCloud : public CommandGui {
public:
    SplitCloud(entt::handle&& handle);
    SplitCloud(entt::registry& _reg)
        : SplitCloud(entt::handle {
            _reg,
            _reg.ctx<SelectedEntity>().selected })
    {
    }

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish(entt::registry& reg) override;

private:
    entt::registry& reg;
    PointCloud* cloud;
    std::optional<PointNormals*> normals;
    std::optional<PointColors*> colors;

    entt::entity target;

    groot::VoxelGrid grid;

    std::vector<PointCloud> result_clouds;
    std::vector<PointNormals> result_normals;
    std::vector<PointColors> result_colors;

public:
    // Parameters
    float voxel_size = 1.0;

    // Result
    std::vector<entt::handle> result;
};

namespace cloud_view_system {

GROOT_APP_LOCAL void init(entt::registry& reg);
GROOT_APP_LOCAL void deinit(entt::registry& reg);
GROOT_APP_LOCAL void run(entt::registry& reg);

}