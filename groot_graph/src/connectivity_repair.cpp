#include "groot/assert.hpp"
#include <groot_graph/connectivity_repair.hpp>
#include <groot/disjoint_sets.hpp>
#include <groot_graph/cgal_helper.hpp>

namespace groot {

DisjointSets compute_connected_components(const PlantGraph& g)
{
    size_t n_vertices = boost::num_vertices(g);
    DisjointSets components(n_vertices);

    auto vertex_indices = boost::get(boost::vertex_index, g);

    PlantGraph::edge_iterator e_it, e_end;
    for (std::tie(e_it, e_end) = boost::edges(g); e_it != e_end; ++e_it) {
        Vertex source = boost::source(*e_it, g);
        Vertex target = boost::target(*e_it, g);

        size_t a = boost::get(vertex_indices, source);
        size_t b = boost::get(vertex_indices, target);

        components.union_set(a, b);
    }

    return components;
}

void repair_connectivity(PlantGraph& g, DisjointSets& components)
{
    GROOT_ASSERT(boost::num_vertices(g) == components.num_vertices(), "Graph and component set must have the same vertices");
    std::list<Vertex> loose_ends;
    auto vertex_indices = boost::get(boost::vertex_index, g);

    size_t main_component = components.find(g.m_property->root_index);

    IncrementalKNN::Tree kdtree;
    PlantGraph::vertex_iterator v_it, v_end;
    for (std::tie(v_it, v_end) = boost::vertices(g); v_it != v_end; ++v_it) {
        kdtree.insert(std::make_tuple(g[*v_it].position, *v_it));
        if (boost::degree(*v_it, g) == 1) {
            loose_ends.push_back(*v_it);
        }
    }

    while (! loose_ends.empty()) {
        Vertex v = loose_ends.front();
        loose_ends.pop_front();

        size_t index = boost::get(vertex_indices, v);
        size_t component = components.find(index);

        if (component == main_component) {
            continue;
        } else {
            IncrementalKNN search(kdtree, g[v].position);

            for (IncrementalKNN::iterator it = search.begin(); it != search.end(); ++it) {
                Vertex v2 = std::get<Vertex>(it->first);
                if (components.find(v2) != component) {
                    boost::add_edge(v, v2, g);
                    components.union_set(v, v2);
                    main_component = components.find(g.m_property->root_index);
                    break;
                }
            }
        }
    }

    reindex(g);
}

}