#pragma once

#include "command_gui.hpp"
#include "components.hpp"
#include "groot/cloud.hpp"
#include <entt/entt.hpp>
#include <groot/cgal.hpp>

async::task<void> compute_normals_command(entt::handle e, size_t k, float radius);

class ComputeNormals : public DialogGui {
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

class RecenterCloud : public CommandGui {
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

class SplitCloud : public CommandGui {
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

void init(entt::registry& reg);
void deinit(entt::registry& reg);
void run(entt::registry& reg);

}