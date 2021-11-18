#pragma once

#include <entt/entt.hpp>
#include "command_gui.hpp"
#include "command.hpp"
#include "components.hpp"

struct GraphResampleArgs {
    float sample_length = 0.3;
};

class GraphResample : public Command {
public:
    GraphResample(entt::handle h, const GraphResampleArgs& args = GraphResampleArgs{});

    CommandState execute() override;
    void on_finish(entt::registry& reg) override; 

    virtual const std::string_view name() override {
        return "Graph Resample";
    }
 
private:
    GraphResampleArgs args;
    groot::PlantGraph* graph;

    groot::PlantGraph sampled;
};

class GraphResampleGui: public Gui {
public:
    GraphResampleGui(const GraphResampleArgs& _args = GraphResampleArgs{})
        : args(_args)
    {}

    std::vector<Command*> get_commands() override;
    GuiResult draw_gui() override;

private:
    GraphResampleArgs args;
    std::vector<entt::entity> targets;
};