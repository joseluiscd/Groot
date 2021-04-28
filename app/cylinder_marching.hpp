#pragma once

#include <entt/entt.hpp>
#include <groot/cylinder_marching.hpp>
#include "command_gui.hpp"
#include "components.hpp"
#include <groot/cylinder_marching.hpp>

class CylinderMarching : public CommandGui {
public:
    CylinderMarching(entt::registry& reg);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish() override;

private:
    entt::registry& reg;
    entt::entity target;
    PointCloud* cloud;
    PointNormals* normals;
    std::vector<groot::CylinderWithPoints> result;

    //Params
    groot::Ransac::Parameters params;
    int min_points = 20;
    float epsilon = 0.03;
    float sampling = 0.06;
    float normal_deviation = 25.0;
    float overlook_probability = 0.01;

};

class CylinderFilter : public CommandGui {
public:
    CylinderFilter(entt::registry& reg);

    GuiState draw_gui() override;
    CommandState execute() override;
private:
    entt::registry& reg;
    entt::entity target;

    // Params
    bool filter_radius = true;
    float radius_range[2] = {0.0, 1.0};

    bool filter_length = false;
    float length_range[2] = {0.0, 1.0};
};

namespace cylinder_view_system {

void init(entt::registry& reg);
void run(entt::registry& reg);

}