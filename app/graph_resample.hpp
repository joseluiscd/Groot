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
    GraphResample(const entt::handle h, const GraphResampleArgs& args = GraphResampleArgs{});

    CommandState execute() override;
    void on_finish(entt::registry& reg) override; 

    virtual const std::string_view name() const override {
        return "Graph Resample";
    }

    entt::entity result;
 
private:
    GraphResampleArgs args;
    groot::PlantGraph* graph;

    std::string old_name;
    groot::PlantGraph sampled;
};

class GraphResampleGui: public DialogGui {
public:
    GraphResampleGui(std::vector<entt::entity>&& _targets, const GraphResampleArgs& _args = GraphResampleArgs{})
        : args(_args)
        , targets(std::move(_targets))
    {}

    std::vector<Command*> get_commands(entt::registry& reg) override;
    void draw_dialog() override;
    std::string_view name() const override { return "Graph resample"; }

private:
    GraphResampleArgs args;
    std::vector<entt::entity> targets;
};