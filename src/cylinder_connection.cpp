#include <groot/plant_graph.hpp>
#include <groot/cylinder_connection.hpp>
#include <groot/cgal_helper.hpp>

namespace groot {

PlantGraph connect_cylinders(CylinderWithPoints* cylinders, size_t count)
{
    PlantGraph graph;
    cgal::Delaunay delaunay;

    std::vector<Vertex> vertex_cylinder(2*count);
    auto vertex_cylinder_map
        = boost::make_iterator_property_map(vertex_cylinder.begin(), boost::get(boost::vertex_index, graph));

    for (size_t i = 0; i < count; i++) {
        Cylinder& cylinder = cylinders[i].cylinder;

        Vertex vertex_a = boost::add_vertex(graph);
        Vertex vertex_b = boost::add_vertex(graph);

        vertex_cylinder_map[vertex_a] = vertex_b;
        vertex_cylinder_map[vertex_b] = vertex_a;

        graph[vertex_a].position = cylinder.center + cylinder.direction * cylinder.middle_height;
        graph[vertex_b].position = cylinder.center - cylinder.direction * cylinder.middle_height;

        cgal::Delaunay::Vertex_handle handle_a = delaunay.insert(graph[vertex_a].position);
        cgal::Delaunay::Vertex_handle handle_b = delaunay.insert(graph[vertex_b].position);

        handle_a->info() = vertex_a;
        handle_b->info() = vertex_b;

        auto [edge, _] = boost::add_edge(vertex_a, vertex_b, graph);

        // This edge is from a detected cylinder, it must have maximum priority
        graph[edge].length = 0.0;
    }

    for (auto i = delaunay.finite_edges_begin(); i != delaunay.finite_edges_end(); i++) {
        cgal::Delaunay::Vertex_handle v1 = i->first->vertex(i->second);
        cgal::Delaunay::Vertex_handle v2 = i->first->vertex(i->third);

        Vertex n1 = v1->info();
        Vertex n2 = v2->info();

        auto [edge, inserted] = boost::add_edge(n1, n2, graph);
        if (inserted) {
            graph[edge].length = std::sqrt(CGAL::squared_distance(v1->point(), v2->point()));
        }
    }

    reindex(graph);

    // TODO: Recompute weights
    return minimum_spanning_tree(graph);
}

}
