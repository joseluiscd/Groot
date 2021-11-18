#include "graph_resample.hpp"
#include "components.hpp"
#include <groot/plant_graph_compare.hpp>


GraphResample::GraphResample(entt::handle h, const GraphResampleArgs& _args)
    : args(_args)
{
    graph = require_components<groot::PlantGraph>(h);
}

CommandState GraphResample::execute()
{
    sampled = std::move(groot::resample_plant_graph(*graph, args.sample_length));
    return CommandState::Ok;
}

void GraphResample::on_finish(entt::registry& reg)
{
}