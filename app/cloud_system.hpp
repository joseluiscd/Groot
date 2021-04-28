#pragma once

#include <groot/cgal.hpp>
#include <entt/entt.hpp>
#include "command_gui.hpp"
#include "components.hpp"


class ComputeNormals : public CommandGui {
public:
    ComputeNormals(entt::registry& reg);

    GuiState draw_gui() override;
    CommandState execute() override;
    void on_finish() override;

private:
    entt::registry& registry;
    PointCloud* cloud;
    entt::entity target;
    std::vector<groot::Vector_3> normals;

    int k = 0;
    float radius = 1.0;

    int selected_k = 1;
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

namespace cloud_view_system {

void init(entt::registry& reg);
void run(entt::registry& reg);

}