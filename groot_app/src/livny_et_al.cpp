#include <groot_app/livny_et_al.hpp>
#include <groot_graph/livny_et_al.hpp>

async::task<void> compute_branch_weights_task(entt::handle h)
{
    return create_task()
        .require_component<groot::PlantGraph>(h)
        .then_async([](const groot::PlantGraph* g) {
            return BranchWeights { groot::compute_weights(*g) };
        })
        .emplace_component<BranchWeights>(h);
}

async::task<void> compute_orientation_field_task(entt::handle h)
{
    return create_task()
        .if_not_components<BranchWeights>(h, [](entt::handle h) {
            return compute_branch_weights_task(h);
        })
        .require_components<groot::PlantGraph, BranchWeights>(h)
        .then_async([](std::tuple<groot::PlantGraph*, BranchWeights*>&& data) {
            auto [graph, weights] = data;

            return OrientationField { groot::compute_orientation_field(*graph, weights->weights) };
        })
        .emplace_component<OrientationField>(h);
}