#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Fuzzy_sphere.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/iterator/function_output_iterator.hpp>
#include <groot_graph/cgal_helper.hpp>
#include <groot_graph/plant_graph.hpp>
#include <spdlog/spdlog.h>

namespace groot {

PlantGraph empty()
{
    return PlantGraph();
}

/*void reindex_vertices(PlantGraph& graph)
{
    auto vertex_indices = boost::get(boost::vertex_index, graph);
    size_t v_count = 0;

    auto [v_it, v_end] = boost::vertices(graph);
    for (; v_it != v_end; ++v_it) {
        vertex_indices[*v_it] = v_count++;
    }
}*/

void reindex_edges(PlantGraph& graph)
{
    auto edge_indices = boost::get(boost::edge_index, graph);
    size_t e_count = 0;

    auto [e_it, e_end] = boost::edges(graph);
    for (; e_it != e_end; ++e_it) {
        edge_indices[*e_it] = e_count++;
    }
}

void reindex(PlantGraph& graph)
{
    // reindex_vertices(graph);
    reindex_edges(graph);
}

void recompute_edge_length(PlantGraph& graph, Edge e)
{
    Vertex a = boost::source(e, graph);
    Vertex b = boost::target(e, graph);

    graph[e].length = std::sqrt(CGAL::squared_distance(
        graph[a].position, graph[b].position));
}

void recompute_edge_lengths(PlantGraph& graph)
{
    for (auto [it, end] = boost::edges(graph); it != end; ++it) {
        recompute_edge_length(graph, *it);
    }
}

PlantGraph from_search(
    cgal::Point_3* cloud,
    size_t count,
    const SearchParams& params)
{
    PlantGraph graph;
    cgal::KdTree kdtree;

    for (size_t i = 0; i < count; i++) {
        Vertex vertex = boost::add_vertex(graph);
        graph[vertex].position = cloud[i];
        kdtree.insert(std::make_tuple(cloud[i], vertex));
    }

    kdtree.build<CGAL::Parallel_tag>();

    auto vertices = boost::vertices(graph);
    if (params.search == SearchType::kRadiusSearch) {
        std::list<cgal::Point3Vertex> search_result;

        for (auto node = vertices.first; node != vertices.second; ++node) {
            cgal::Point_3 point = graph[*node].position;
            CGAL::Fuzzy_sphere<cgal::SearchTraits> query(point, params.radius);

            kdtree.search(std::back_inserter(search_result), query);
            for (auto it = search_result.begin(); it != search_result.end(); ++it) {
                auto [edge, _] = boost::add_edge(*node, std::get<Vertex>(*it), graph);
                graph[edge].length = std::sqrt(CGAL::squared_distance(point, std::get<cgal::Point_3>(*it)));
            }
            search_result.clear();
        }
    } else if (params.search == SearchType::kKnnSearch) {
        for (auto node = vertices.first; node != vertices.second; ++node) {
            cgal::KNeighbour knn(kdtree, graph[*node].position, params.k);
            for (auto it = knn.begin(); it != knn.end(); ++it) {
                auto [edge, _] = boost::add_edge(*node, std::get<Vertex>(it->first), graph);
                graph[edge].length = it->second;
            }
        }
    }
    spdlog::info("Edge count {}", boost::num_edges(graph));
    reindex(graph);
    return graph;
}

PlantGraph from_delaunay(cgal::Point_3* cloud, size_t count)
{
    PlantGraph graph;
    cgal::Delaunay delaunay;

    for (size_t i = 0; i < count; i++) {
        Vertex vertex = boost::add_vertex(graph);
        graph[vertex].position = cloud[i];

        Delaunay::Point_3 p(cloud[i].x(), cloud[i].y(), cloud[i].z());
        cgal::Delaunay::Vertex_handle handle = delaunay.insert(p);
        handle->info() = vertex;
    }

    for (auto i = delaunay.finite_edges_begin(); i != delaunay.finite_edges_end(); i++) {
        cgal::Delaunay::Vertex_handle v1 = i->first->vertex(i->second);
        cgal::Delaunay::Vertex_handle v2 = i->first->vertex(i->third);

        Vertex n1 = v1->info();
        Vertex n2 = v2->info();

        auto [edge, _] = boost::add_edge(n1, n2, graph);
        graph[edge].length = std::sqrt(CGAL::squared_distance(v1->point(), v2->point()));
    }

    reindex(graph);
    return graph;
}

PlantGraph from_alpha_shape(
    cgal::Point_3* cloud,
    size_t count,
    float alpha,
    size_t components)
{
    PlantGraph graph;

    // Just something stupid
    std::vector<std::pair<AlphaShape::Point_3, Vertex>> vertices;
    for (size_t i = 0; i < count; i++) {
        Vertex v = boost::add_vertex(graph);
        graph[v].position = cloud[i];

        AlphaShape::Point_3 p(cloud[i].x(), cloud[i].y(), cloud[i].z());

        vertices.push_back(std::make_pair(p, v));
    }

    cgal::AlphaShape alpha_shape(vertices.begin(), vertices.end());

    if (alpha == 0) {
        alpha = *alpha_shape.find_optimal_alpha(components);
    }
    alpha_shape.set_alpha(alpha);

    std::vector<cgal::AlphaShape::Edge> edges;

    alpha_shape.get_alpha_shape_edges(std::back_inserter(edges), AlphaShape::INTERIOR);

    for (auto i = edges.begin(); i != edges.end(); ++i) {
        cgal::AlphaShape::Vertex_handle v1 = i->first->vertex(i->second);
        cgal::AlphaShape::Vertex_handle v2 = i->first->vertex(i->third);

        Vertex n1 = v1->info();
        Vertex n2 = v2->info();

        auto [edge, _] = boost::add_edge(n1, n2, graph);
        graph[edge].length = std::sqrt(CGAL::squared_distance(v1->point(), v2->point()));
    }

    reindex(graph);
    return graph;
}

PlantGraph from_cardenas_et_al(Point_3* cloud, size_t count, float radius, const point_finder::PointFinder& f)
{
    PlantGraph radius_graph = from_search(cloud, count, SearchParams { 0, radius, SearchType::kRadiusSearch });

    std::vector<size_t> connected_components(boost::num_vertices(radius_graph));
    auto component_map = boost::make_iterator_property_map(connected_components.begin(), boost::get(boost::vertex_index, radius_graph));

    size_t components = boost::connected_components(radius_graph, component_map);

    if (components == 1) {
        // Only 1 connected component, no need for further operation
        return radius_graph;
    }
    spdlog::info("Connected components: {}", components);

    struct EdgeElement {
        Vertex a;
        Vertex b;
        float distance = +INFINITY;
    };

    // std::vector<EdgeElement> elements(components * components);

    // Further operation
    // PlantGraph alpha_graph = from_alpha_shape(cloud, count, 0.0, 1);
    // groot::find_root(radius_graph, f);
    // alpha_graph = geodesic(alpha_graph);
    // PlantGraph alpha_graph = from_search(cloud, count, SearchParams { .k = 0, .radius = radius * 2, .search = SearchType::kRadiusSearch});
    // PlantGraph alpha_graph = from_cardenas_et_al(cloud, count, radius * 2);

    // Este es el bueno:
    PlantGraph alpha_graph = from_delaunay(cloud, count);
    groot::find_root(radius_graph, f);
    alpha_graph = minimum_spanning_tree(alpha_graph);

    auto [edge_begin, edge_end] = boost::edges(alpha_graph);
    for (auto i = edge_begin; i != edge_end; ++i) {
        Vertex a = boost::source(*i, alpha_graph);
        Vertex b = boost::target(*i, alpha_graph);

        size_t c_a = component_map[a];
        size_t c_b = component_map[b];

        if (c_a < c_b) {
            std::swap(c_a, c_b);
        }

        if (c_a != c_b) {
            auto [edge, _] = boost::add_edge(a, b, radius_graph);
            radius_graph[edge].length = alpha_graph[*i].length;
        }
    }

    // for (size_t a = 0; a < components; a++) {
    //     for (size_t b = 0; b < a; b++) {
    //         EdgeElement& e = elements[a * components + b];
    //         if (e.distance != INFINITY) {
    //             auto [edge, _] = boost::add_edge(e.a, e.b, radius_graph);
    //             radius_graph[edge].length = e.distance;
    //         }
    //     }
    // }
    reindex(radius_graph);
    return radius_graph;
}

template <typename MapType>
struct EdgeFilter {
    EdgeFilter() {};
    EdgeFilter(MapType _map)
        : map(_map) {};

    bool operator()(const Edge& e) const { return map[e]; }

    MapType map;
};

//TODO: Cleanup
PlantGraph geodesic(const PlantGraph& graph, PropertyMap<float>* _distance_map, float* max_distance)
{
    std::vector<Vertex> predecessors(boost::num_vertices(graph));
    auto predecessor_map = make_vertex_property_map(predecessors, graph);

    auto weight_map = boost::get(&EdgeProperties::length, graph);

    std::vector<float> local_distance_map;

    std::vector<float>* distance_map;

    if (_distance_map) {
        *_distance_map = std::vector<float>(boost::num_vertices(graph), 0.0f);
        distance_map = _distance_map;
    } else {
        local_distance_map = std::vector<float>(boost::num_vertices(graph), 0.0f);
        distance_map = &local_distance_map;
    }

    auto dmap = boost::make_iterator_property_map(distance_map->begin(), boost::get(boost::vertex_index, graph));

    if (max_distance) {
        *max_distance = -INFINITY;
    }

    Vertex root = boost::vertex(graph.m_property->root_index, graph);
    boost::dijkstra_shortest_paths(graph, root,
        boost::weight_map(weight_map)
            .distance_map(dmap)
            .predecessor_map(predecessor_map));

    std::vector<bool> is_shortest_path(boost::num_edges(graph));
    auto shortest_path_map
        = boost::make_iterator_property_map(is_shortest_path.begin(), boost::get(boost::edge_index, graph));

    auto [vertex_begin, vertex_end] = boost::vertices(graph);
    for (auto n = vertex_begin; n != vertex_end; ++n) {
        if (predecessor_map[*n] != *n) {
            // Nodes are reachable from the root
            auto [edge, exists] = boost::edge(*n, predecessor_map[*n], graph);
            boost::edge(predecessor_map[*n], *n, graph);

            shortest_path_map[edge] = true;

            if (max_distance) {
                if ((*distance_map)[*n] > *max_distance) {
                    *max_distance = (*distance_map)[*n];
                }
            }
        }
    }

    boost::filtered_graph filter(graph, EdgeFilter(shortest_path_map));

    PlantGraph ret;
    boost::copy_graph(filter, ret);
    *ret.m_property = *graph.m_property;
    reindex_edges(ret);
    return ret;
}

//TODO: Cleanup
PlantGraph minimum_spanning_tree(const PlantGraph& graph, PropertyMap<float>* _distance_map, float* max_distance)
{
    std::vector<Vertex> predecessors(boost::num_vertices(graph));
    auto predecessor_map
        = boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, graph));

    auto weight_map = boost::get(&EdgeProperties::length, graph);

    std::vector<float> local_distance_map;
    std::vector<float>* distance_map;

    if (_distance_map) {
        *_distance_map = std::vector<float>(boost::num_vertices(graph), 0.0f);
        distance_map = _distance_map;
    } else {
        local_distance_map = std::vector<float>(boost::num_vertices(graph), 0.0f);
        distance_map = &local_distance_map;
    }

    if (max_distance) {
        *max_distance = -INFINITY;
    }

    auto dmap = boost::make_iterator_property_map(distance_map->begin(), boost::get(boost::vertex_index, graph));

    Vertex root = boost::vertex(graph.m_property->root_index, graph);
    boost::prim_minimum_spanning_tree(graph, predecessor_map,
        boost::root_vertex(root)
            .weight_map(weight_map)
            .distance_map(dmap));

    std::vector<bool> is_shortest_path(boost::num_edges(graph));
    auto shortest_path_map
        = boost::make_iterator_property_map(is_shortest_path.begin(), boost::get(boost::edge_index, graph));

    auto [vertex_begin, vertex_end] = boost::vertices(graph);
    for (auto n = vertex_begin; n != vertex_end; ++n) {
        if (predecessor_map[*n] != *n) {
            // Nodes are reachable from the root
            auto [edge, _] = boost::edge(*n, predecessor_map[*n], graph);
            shortest_path_map[edge] = true;

            if (max_distance) {
                if ((*distance_map)[*n] > *max_distance) {
                    *max_distance = (*distance_map)[*n];
                }
            }
        }
    }

    boost::filtered_graph filter(graph, EdgeFilter(shortest_path_map));

    PlantGraph ret;
    boost::copy_graph(filter, ret);
    *ret.m_property = *graph.m_property;
    reindex_edges(ret);
    return ret;
}

namespace point_finder {

    MinX MinXPointFinder;
    MinY MinYPointFinder;
    MinZ MinZPointFinder;
    MaxX MaxXPointFinder;
    MaxY MaxYPointFinder;
    MaxZ MaxZPointFinder;

    Vertex min_coord(const PlantGraph& graph, size_t axis)
    {
        Vertex point;
        float min_coord = +INFINITY;

        auto [it, end] = boost::vertices(graph);
        for (; it != end; ++it) {
            float value = graph[*it].position[axis];
            if (value < min_coord) {
                point = *it;
                min_coord = value;
            }
        }

        return point;
    }
    Vertex max_coord(const PlantGraph& graph, size_t axis)
    {
        Vertex point;
        float max_coord = -INFINITY;

        auto [it, end] = boost::vertices(graph);
        for (; it != end; ++it) {
            float value = graph[*it].position[axis];
            if (value > max_coord) {
                point = *it;
                max_coord = value;
            }
        }

        return point;
    }
}

} // namespace groot
