#pragma once

#include <groot_app/command_gui.hpp>
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>
#include <groot_graph/cylinder_marching.hpp>

class GROOT_APP_API CylinderMarching : public CommandGui {
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
    float epsilon = 0.03f;
    float sampling = 0.06f;
    float normal_deviation = 25.0f;
    float overlook_probability = 0.01f;
    float voxel_size = 1.0f;
};

class GROOT_APP_API CylinderFilter : public CommandGui {
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

class GROOT_APP_API CylinderPointFilter : public CommandGui {
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

GROOT_APP_LOCAL void init(entt::registry& reg);
GROOT_APP_LOCAL void deinit(entt::registry& reg);
GROOT_APP_LOCAL void run(entt::registry& reg);

}