#include <groot_graph/line_graph.hpp>

namespace groot {

template <typename DistanceMap, typename VertexMap>
class LineGraphVisitor : public boost::default_dfs_visitor {
public:
    LineGraphVisitor(PlantGraph& _target, PropertyMap<Vertex>& _prev_vertex, PropertyMap<cgal::Vector_3>& _tangents)
        : new_graph(_target)
        , previous_vertex(_prev_vertex)
        , tangents(_tangents)
    {
    }

    void tree_edge(Edge e, const PlantGraph& g)
    {
        Vertex source = boost::source(e, g);
        Vertex target = boost::target(e, g);

        cgal::Point_3 source_point = g[source].position;
        cgal::Point_3 target_point = g[target].position;

        // Unit vector in the direction of the edge
        cgal::Vector_3 edge_direction(target_point - source_point);
        edge_direction /= std::sqrt(edge_direction.squared_length());

        
        // New vertex in midpoint
        VertexProperties vp;
        vp.position = CGAL::midpoint(source_point, target_point);
        Vertex new_vertex = boost::add_vertex(vp, new_graph);

        tangents[new_vertex] = edge_direction;

        previous_vertex[target] = new_vertex;
        Vertex prev_vertex = previous_vertex[source];

        EdgeProperties ep;
        ep.length = std::sqrt(CGAL::squared_distance(new_graph[prev_vertex].position, vp.position));
        
        boost::add_edge(prev_vertex, new_vertex, new_graph);
    }

private:
    PlantGraph& new_graph;
    PropertyMap<Vertex>& previous_vertex;
    PropertyMap<cgal::Vector_3>& tangents;
};

LineGraph compute_line_graph(const groot::PlantGraph& line_graph);

}