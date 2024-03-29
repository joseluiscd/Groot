#include "entt/entity/fwd.hpp"
#include <gfx/imgui/imgui.h>
#include <groot_app/components.hpp>
#include <groot_app/graph_resample.hpp>
#include <groot_app/task.hpp>
#include <groot_graph/plant_graph.hpp>
#include <groot_graph/plant_graph_compare.hpp>

async::task<entt::entity> graph_resample_command(entt::handle h, float sample_length)
{
    entt::registry& reg = *h.registry();
    entt::entity e = h.entity();

    return create_task()
        .require_component<groot::PlantGraph>(h)
        .then_async([sample_length](groot::PlantGraph* graph) -> groot::PlantGraph {
            return groot::resample_plant_graph(*graph, sample_length);
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

async::task<entt::entity> graph_match_command(entt::handle h1, entt::handle h2)
{
    entt::registry& reg = *h1.registry();
    entt::entity e1 = h1.entity();
    entt::entity e2 = h2.entity();

    return create_task()
        .then_sync([&reg, e1, e2]() {
            std::pair<groot::PlantGraph*, groot::PlantGraph*> r;

            r.first = handle_require_components<groot::PlantGraph>(entt::handle(reg, e1));
            r.second = handle_require_components<groot::PlantGraph>(entt::handle(reg, e2));

            return r;
        })
        .then_async([](std::pair<groot::PlantGraph*, groot::PlantGraph*>&& graphs) -> groot::PlantGraph {
            return groot::plant_graph_nn(*graphs.first, *graphs.second);
        })
        .then_sync([&reg](groot::PlantGraph&& graph) -> entt::entity {
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
    reg.ctx<TaskManager>().push_task(targets.size() > 1 ? "Resampling graphs" : "Resampling graph", cmd.then(discard));
}