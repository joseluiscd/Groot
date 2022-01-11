#pragma once

#include <groot_graph/cylinder_connection.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/entt.hpp>
#include <groot_app/components.hpp>


class GROOT_APP_API CylinderConnection : public CommandGui {
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