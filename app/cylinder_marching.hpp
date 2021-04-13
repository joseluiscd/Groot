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
    std::vector<groot::Cylinder> result;

    //Params
    groot::Ransac::Parameters params;

};