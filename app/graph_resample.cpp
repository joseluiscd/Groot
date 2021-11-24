#include "graph_resample.hpp"
#include "components.hpp"
#include "entt/entity/fwd.hpp"
#include "groot/plant_graph.hpp"
#include "task.hpp"
#include <gfx/imgui/imgui.h>
#include <groot/plant_graph_compare.hpp>

async::task<entt::entity> graph_resample_command(entt::handle h, float sample_length)
{
    entt::registry& reg = *h.registry();
    entt::entity e = h.entity();

    return async::spawn(sync_scheduler(), [&reg, e]() -> groot::PlantGraph* {
        groot::PlantGraph* graph = require_components<groot::PlantGraph>(entt::handle(reg, e));
        return graph;
    }).then(async_scheduler(), [sample_length](groot::PlantGraph* graph) -> groot::PlantGraph {
          return groot::resample_plant_graph(*graph, sample_length);
      }).then(sync_scheduler(), [&reg, e](groot::PlantGraph&& sampled) -> entt::entity {
        Name* name_c = reg.try_get<Name>(e);
        std::string old_name = name_c ? name_c->name : std::string("");

        entt::entity result = reg.create();
        reg.emplace<groot::PlantGraph>(result, std::move(sampled));
        reg.emplace<Name>(result, old_name + "_resampled");
        reg.emplace<Visible>(result);

        return result;
    });
}

async::task<entt::entity> graph_match_command(entt::handle h1, entt::handle h2)
{
    entt::registry& reg = *h1.registry();
    entt::entity e1 = h1.entity();
    entt::entity e2 = h2.entity();

    return async::spawn(sync_scheduler(), [&reg, e1, e2]() {
        std::pair<groot::PlantGraph*, groot::PlantGraph*> r;

        r.first = require_components<groot::PlantGraph>(entt::handle(reg, e1));
        r.second = require_components<groot::PlantGraph>(entt::handle(reg, e2));

        return r;

    }).then(async_scheduler(), [](std::pair<groot::PlantGraph*, groot::PlantGraph*>&& graphs) -> groot::PlantGraph {
        return groot::plant_graph_nn(*graphs.first, *graphs.second);
    }).then(sync_scheduler(), [&reg](groot::PlantGraph&& graph) -> entt::entity {
        entt::entity result = reg.create();
        reg.emplace<groot::PlantGraph>(result, std::move(graph));
        reg.emplace<Visible>(result);

        return result;
    });
}

void GraphResampleGui::draw_dialog()
{
    ImGui::InputFloat("Sample length", &args.sample_length);
}

void GraphResampleGui::schedule_commands(entt::registry& reg)
{
    auto&& cmd = graph_resample_command(reg, targets.begin(), targets.end(), args.sample_length);
    reg.ctx<TaskBroker>().push_task(targets.size() > 1 ? "Resampling graphs" : "Resampling graph", cmd.then(discard));
}