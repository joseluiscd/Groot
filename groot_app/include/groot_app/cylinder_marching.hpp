#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <groot_graph/cylinder_marching.hpp>

class CylinderMarching : public CommandGui {
public:
    CylinderMarching(entt::registry& reg);
    CylinderMarching(entt::handle&& handle);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish(entt::registry& reg) override;

private:
    entt::registry& reg;
    entt::entity target;
    PointCloud* cloud;
    PointNormals* normals;
    std::vector<groot::CylinderWithPoints> result;

public:
    //Params
    int min_points = 20;
    float epsilon = 0.03;
    float sampling = 0.06;
    float normal_deviation = 25.0;
    float overlook_probability = 0.01;
    float voxel_size = 1.0f;
};

class CylinderFilter : public CommandGui {
public:
    CylinderFilter(entt::registry& reg);
    CylinderFilter(entt::handle&& handle);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish(entt::registry& reg) override;

private:
    entt::registry& reg;
    entt::entity target;

    Cylinders* cylinders;

    std::vector<groot::CylinderWithPoints> new_cylinders;

public:
    // Params
    bool filter_radius = true;
    float radius_range[2] = { 0.0, 1.0 };

    bool filter_length = false;
    float length_range[2] = { 0.0, 1.0 };
};

class CylinderPointFilter : public CommandGui {
public:
    CylinderPointFilter(entt::handle&& handle);
    CylinderPointFilter(entt::registry& _reg)
        : CylinderPointFilter(entt::handle {
            _reg,
            _reg.ctx<SelectedEntity>().selected })
    {
    }

    GuiState draw_gui() override
    {
        return GuiState::RunAsync;
    }

    CommandState execute() override;
    void on_finish(entt::registry& reg) override;

private:
    entt::registry& reg;
    entt::entity target;

    Cylinders* cylinders;
    PointCloud cloud;
};

namespace cylinder_view_system {

void init(entt::registry& reg);
void deinit(entt::registry& reg);
void run(entt::registry& reg);

}