#include <CGAL/Fuzzy_sphere.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <groot/cgal_helper.hpp>
#include <groot/plant_graph.hpp>
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
    //reindex_vertices(graph);
    reindex_edges(graph);
}

PlantGraph from_search(
    glm::vec3* cloud,
    size_t count,
    const SearchParams& params)
{
    PlantGraph graph;
    cgal::KdTree kdtree;

    for (size_t i = 0; i < count; i++) {
        Vertex vertex = boost::add_vertex(graph);
        cgal::Point_3 point = cgal::Point_3(cloud[i].x, cloud[i].y, cloud[i].z);
        graph[vertex].position = point;
        kdtree.insert(std::make_tuple(point, vertex));
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

PlantGraph from_delaunay(glm::vec3* cloud, size_t count)
{
    PlantGraph graph;
    cgal::Delaunay delaunay;

    for (size_t i = 0; i < count; i++) {
        Vertex vertex = boost::add_vertex(graph);
        cgal::Point_3 point = cgal::Point_3(cloud[i].x, cloud[i].y, cloud[i].z);
        graph[vertex].position = point;

        cgal::Delaunay::Vertex_handle handle = delaunay.insert(point);
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

template <typename MapType>
struct EdgeFilter {
    EdgeFilter() {};
    EdgeFilter(MapType _map)
        : map(_map) {};

    bool operator()(const Edge& e) const { return map[e]; }

    MapType map;
};

PlantGraph geodesic(PlantGraph& graph)
{
    std::vector<Vertex> predecessors(boost::num_vertices(graph));
    auto predecessor_map
        = boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, graph));

    auto weight_map = boost::get(&EdgeProperties::length, graph);
    auto distance_map = boost::get(&VertexProperties::root_distance, graph);

    Vertex root = boost::vertex(graph.m_property->root_index, graph);
    boost::dijkstra_shortest_paths(graph, root,
        boost::weight_map(weight_map)
            .distance_map(distance_map)
            .predecessor_map(predecessor_map));

    std::vector<bool> is_shortest_path(boost::num_edges(graph));
    auto shortest_path_map
        = boost::make_iterator_property_map(is_shortest_path.begin(), boost::get(boost::edge_index, graph));

    auto edge_index = boost::get(boost::edge_index, graph);

    auto [vertex_begin, vertex_end] = boost::vertices(graph);
    for (auto n = vertex_begin; n != vertex_end; ++n) {
        if (predecessor_map[*n] != *n) {
            // Nodes are reachable from the root
            auto [edge, exists] = boost::edge(*n, predecessor_map[*n], graph);
            auto [edge_2, exists_2] = boost::edge(predecessor_map[*n], *n, graph);

            shortest_path_map[edge] = true;

            if (graph[*n].root_distance > graph.m_property->max_root_distance) {
                graph.m_property->max_root_distance = graph[*n].root_distance;
            }
        }
    }
    auto edge_indices = boost::get(boost::edge_index, graph);

    boost::filtered_graph filter(graph, EdgeFilter(shortest_path_map));

    PlantGraph ret;
    boost::copy_graph(filter, ret);
    *ret.m_property = *graph.m_property;
    reindex_edges(ret);
    return ret;
}

PlantGraph minimum_spanning_tree(PlantGraph& graph)
{
    std::vector<Vertex> predecessors(boost::num_vertices(graph));
    auto predecessor_map
        = boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, graph));

    auto distance_map = boost::get(&VertexProperties::root_distance, graph);
    auto weight_map = boost::get(&EdgeProperties::length, graph);

    Vertex root = boost::vertex(graph.m_property->root_index, graph);
    boost::prim_minimum_spanning_tree(graph, predecessor_map,
        boost::root_vertex(root)
            .weight_map(weight_map)
            .distance_map(distance_map));

    std::vector<bool> is_shortest_path(boost::num_edges(graph));
    auto shortest_path_map
        = boost::make_iterator_property_map(is_shortest_path.begin(), boost::get(boost::edge_index, graph));

    auto [vertex_begin, vertex_end] = boost::vertices(graph);
    for (auto n = vertex_begin; n != vertex_end; ++n) {
        if (predecessor_map[*n] != *n) {
            // Nodes are reachable from the root
            auto [edge, _] = boost::edge(*n, predecessor_map[*n], graph);
            shortest_path_map[edge] = true;

            if (graph[*n].root_distance > graph.m_property->max_root_distance) {
                graph.m_property->max_root_distance = graph[*n].root_distance;
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

void write_to_file(const PlantGraph& g, std::ostream& output)
{
    boost::archive::binary_oarchive out_archive(output);
    out_archive << g;
}

PlantGraph read_from_file(std::istream& input)
{
    PlantGraph g;
    size_t root_id;
    boost::archive::binary_iarchive input_archive(input);
    input_archive >> g >> root_id;

    return g;
}

namespace point_finder {
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
