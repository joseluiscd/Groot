#pragma once

#include "command_gui.hpp"
#include "components.hpp"
#include "groot/cloud.hpp"
#include <entt/entt.hpp>
#include <groot/cgal.hpp>

class ComputeNormals : public CommandGui {
public:
    ComputeNormals(entt::registry& reg);
    ComputeNormals(entt::handle&& handle);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish() override;

private:
    entt::registry& registry;
    PointCloud* cloud;
    entt::entity target;
    std::vector<groot::Vector_3> normals;

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

private:
    entt::registry& reg;
    PointCloud* cloud;
    entt::entity target;

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
    void on_finish() override;

private:
    entt::registry& reg;
    PointCloud* cloud;
    std::optional<PointNormals*> normals;

    entt::entity target;

    groot::VoxelGrid grid;

    std::vector<PointCloud> result_clouds;
    std::vector<PointNormals> result_normals;

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