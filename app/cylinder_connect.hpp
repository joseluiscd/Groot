#pragma once

#include <groot/cylinder_connection.hpp>
#include "command_gui.hpp"
#include "entt.hpp"
#include "components.hpp"


class CylinderConnection : public CommandGui {
public:
    CylinderConnection(entt::handle&& handle);
    CylinderConnection(entt::registry& _reg)
        : CylinderConnection(entt::handle {
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
    
public:
    std::optional<groot::PlantGraph> result = {};
};