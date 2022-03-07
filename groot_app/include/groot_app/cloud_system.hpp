#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include "groot/cloud.hpp"
#include <groot_app/entt.hpp>
#include <groot/cgal.hpp>

GROOT_APP_API async::task<void> compute_normals_command(entt::handle e, size_t k, float radius);
GROOT_APP_API async::task<std::vector<entt::handle>> split_cloud_command(entt::handle h, float voxel_size);

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


class GROOT_APP_LOCAL SplitCloudGui : public DialogGui {
public:
    SplitCloudGui(entt::handle h)
        : target(h)
    {}

    void draw_dialog() override;
    std::string_view name() const override { return "Split cloud in voxels"; }
    void schedule_commands(entt::registry& reg) override;

private:
    entt::handle target;

    // Parameters
    float voxel_size = 1.0;
};

namespace cloud_view_system {

GROOT_APP_LOCAL void init(entt::registry& reg);
GROOT_APP_LOCAL void deinit(entt::registry& reg);
GROOT_APP_LOCAL void run(entt::registry& reg);

}