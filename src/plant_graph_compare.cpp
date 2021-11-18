#include <groot/plant_graph_compare.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace groot {

template <typename DistanceMap, typename VertexMap>
class ResampleVisitor : public boost::default_dfs_visitor {
public:
    ResampleVisitor(DistanceMap& dmap, VertexMap& vmap, float _length, PlantGraph& _new_graph)
        : remaining_distance_map(dmap)
        , last_corresponding_vertex_map(vmap)
        , sample_length(_length)
        , new_graph(_new_graph)
    {
    }

    void examine_edge(Edge e, const PlantGraph& g)
    {
        Vertex source = boost::source(e, g);
        Vertex target = boost::target(e, g);

        cgal::Point_3 source_point = g[source].position;
        cgal::Point_3 target_point = g[target].position;
        cgal::Vector_3 edge_direction(target_point - source_point);
        edge_direction /= std::sqrt(edge_direction.squared_length());

        float length = g[e].length;
        float remaining_distance = remaining_distance_map[source];
        float current_step_length = length - remaining_distance;

        Vertex previous_vertex = last_corresponding_vertex_map[source];

        while (current_step_length < length)
        {
            Vertex new_vertex = boost::add_vertex(new_graph);
            Edge new_edge = boost::add_edge(previous_vertex, new_vertex, new_graph).first;

            new_graph[new_vertex].position = source_point + current_step_length * edge_direction;
            new_graph[new_edge].length = std::sqrt(CGAL::squared_distance(new_graph[new_vertex].position, new_graph[previous_vertex].position));

            previous_vertex = new_vertex;

            current_step_length += sample_length;
        }

        last_corresponding_vertex_map[target] = previous_vertex;
        remaining_distance_map[target] = current_step_length - length;
    }


private:
    DistanceMap& remaining_distance_map;
    VertexMap& last_corresponding_vertex_map;
    float sample_length;
    PlantGraph& new_graph;
};

template <typename DistanceMap, typename VertexMap>
ResampleVisitor<DistanceMap, VertexMap> make_resample_visitor(DistanceMap& dmap, VertexMap& vmap, float _length, PlantGraph& _new_graph)
{
    return ResampleVisitor<DistanceMap, VertexMap>(dmap, vmap, _length, _new_graph);
}

PlantGraph resample_plant_graph(const PlantGraph& graph, float sample_length)
{
    PlantGraph sampled_plant;

    std::vector<float> remaining_distances(boost::num_vertices(graph), 0.0f);
    std::vector<Vertex> last_corresponding_vertex(boost::num_vertices(graph));
    std::vector<int> color(boost::num_vertices(graph), 0);

    auto remaining_distance_map = boost::make_iterator_property_map(remaining_distances.begin(), boost::get(boost::vertex_index, graph));
    auto last_corresponding_vertex_map = boost::make_iterator_property_map(last_corresponding_vertex.begin(), boost::get(boost::vertex_index, graph));
    auto color_map = boost::make_iterator_property_map(color.begin(), boost::get(boost::vertex_index, graph));

    last_corresponding_vertex_map[graph.m_property->root_index] = boost::add_vertex(sampled_plant);

    auto visitor = make_resample_visitor(remaining_distance_map, last_corresponding_vertex_map, sample_length, sampled_plant);

    boost::depth_first_visit(graph, graph.m_property->root_index, visitor, color_map);

    return sampled_plant;
}

}