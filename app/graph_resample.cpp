#include "graph_resample.hpp"
#include "components.hpp"
#include "entt/entity/fwd.hpp"
#include "groot/plant_graph.hpp"
#include "task.hpp"
#include <gfx/imgui/imgui.h>
#include <groot/plant_graph_compare.hpp>

GraphResample::GraphResample(entt::handle h, const GraphResampleArgs& _args)
    : args(_args)
    , result(entt::null)
{
    graph = require_components<groot::PlantGraph>(h);

    Name* name_c = h.try_get<Name>();
    old_name = name_c ? name_c->name : std::string("");
}

CommandState GraphResample::execute()
{
    sampled = std::move(groot::resample_plant_graph(*graph, args.sample_length));
    return CommandState::Ok;
}

void GraphResample::on_finish(entt::registry& reg)
{
    result = reg.create();
    reg.emplace<groot::PlantGraph>(result, std::move(sampled));
    reg.emplace<Name>(result, old_name + "_resampled");
    reg.emplace<Visible>(result);
}

Task<entt::entity> cmd(GraphResampleArgs args, entt::registry& reg, entt::entity e)
{
    return create_task()
        .then_sync([&reg, e]() -> groot::PlantGraph* {
            groot::PlantGraph* graph = require_components<groot::PlantGraph>(entt::handle(reg, e));
            return graph;
        })
        .then_async([args](groot::PlantGraph* graph) -> groot::PlantGraph {
            return groot::resample_plant_graph(*graph, args.sample_length);
        })
        .then_sync([&reg, e](groot::PlantGraph&& sampled) -> entt::entity {
            Name* name_c = reg.try_get<Name>(e);
            std::string old_name = name_c ? name_c->name : std::string("");

            entt::entity result = reg.create();
            reg.emplace<groot::PlantGraph>(result, std::move(sampled));
            reg.emplace<Name>(result, old_name + "_resampled");
            reg.emplace<Visible>(result);

            return result;
        });
}

void GraphResampleGui::draw_dialog()
{
    ImGui::InputFloat("Sample length", &args.sample_length);
}

std::vector<Command*> GraphResampleGui::get_commands(entt::registry& r)
{
    return command_to_entities<GraphResample>(r, targets.begin(), targets.end(), [this](entt::handle h) -> GraphResample* {
        return new GraphResample(h, args);
    });
}