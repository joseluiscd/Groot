#include <groot_app/components.hpp>
#include <groot_app/graph_repair.hpp>
#include <groot_graph/connectivity_repair.hpp>

async::task<void> graph_compute_connected_components(entt::handle h)
{
    return create_task()
        .require_component<groot::PlantGraph>(h)
        .then_async([](const groot::PlantGraph* g) {
            return ConnectedComponents { groot::compute_connected_components(*g) };
        })
        .emplace_component<ConnectedComponents>(h);
}

struct GraphAndCC {
    groot::PlantGraph graph;
    ConnectedComponents cc;
};

async::task<void> graph_repair_command(entt::handle h)
{
    return create_task()
        .then_sync([h]() {
            require_components<groot::PlantGraph>(h);
            if (h.all_of<ConnectedComponents>()) {
                return async::spawn(async::inline_scheduler(), discard);
            } else {
                return graph_compute_connected_components(h);
            }
        })
        .then_sync([h]() {
            GraphAndCC ret {
                groot::empty(),
                ConnectedComponents { groot::DisjointSets { 0 } }
            };

            h.patch<ConnectedComponents>([&ret](ConnectedComponents& c) {
                std::swap(c, ret.cc);
            });

            h.patch<groot::PlantGraph>([&ret](groot::PlantGraph& g) {
                std::swap(g, ret.graph);
            });

            return ret;
        })
        .then_async([](GraphAndCC&& _gcc) {
            GraphAndCC gcc = std::move(_gcc);
            groot::repair_connectivity(gcc.graph, gcc.cc.components);
            return gcc;
        })
        .then_sync([h](GraphAndCC&& gcc) {
            h.emplace_or_replace<groot::PlantGraph>(std::move(gcc.graph));
            h.emplace_or_replace<ConnectedComponents>(std::move(gcc.cc));
        });
}