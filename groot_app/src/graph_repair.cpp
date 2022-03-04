#include <groot_app/graph_repair.hpp>
#include <groot_graph/connectivity_repair.hpp>

async::task<entt::entity> graph_compute_connected_components(entt::handle h)
{
    return async::spawn(sync_scheduler(), [h]() {
        return require_components<groot::PlantGraph>(h);
    }).then(async_scheduler(), [](const groot::PlantGraph* g) {
        return groot::compute_connected_components(*g);
    }).then(sync_scheduler(), [h](groot::DisjointSets&& c) {
        h.emplace_or_replace<ConnectedComponents>(std::move(c));
        return h.entity();
    });
}

struct GraphAndCC {
    groot::PlantGraph graph;
    ConnectedComponents cc;
};

async::task<entt::entity> graph_repair_command(entt::handle h)
{
    return async::spawn(sync_scheduler(), [h]() {
        require_components<groot::PlantGraph>(h);
        if (h.all_of<ConnectedComponents>()) {
            return graph_compute_connected_components(h).then(async::inline_scheduler(), discard);
        } else {
            return async::spawn(async::inline_scheduler(), discard);
        }
    }).then(sync_scheduler(), [h](){
        GraphAndCC ret {
            groot::empty(),
            ConnectedComponents { groot::DisjointSets{ 0 } }
        };

        h.patch<ConnectedComponents>([&ret](ConnectedComponents& c) {
            std::swap(c, ret.cc);
        });

        h.patch<groot::PlantGraph>([&ret] (groot::PlantGraph& g) {
            std::swap(g, ret.graph);
        });


        return ret;
    }).then(async_scheduler(), [](GraphAndCC&& _gcc) {
        GraphAndCC gcc = std::move(_gcc);
        groot::repair_connectivity(gcc.graph, gcc.cc.components);
        return gcc;
    }).then(sync_scheduler(), [h](GraphAndCC&& gcc) {
        h.emplace_or_replace<groot::PlantGraph>(std::move(gcc.graph));
        h.emplace_or_replace<ConnectedComponents>(std::move(gcc.cc));
        return h.entity();
    });
}