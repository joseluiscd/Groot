#include "python_graph.hpp"
#include <boost/iterator/transform_iterator.hpp>
#include <boost/python.hpp>
#include <boost/python/iterator.hpp>
#include <groot_graph/plant_graph.hpp>

using custodian_this = boost::python::with_custodian_and_ward_postcall<0, 1>;

struct Vertex {
    groot::PlantGraph& graph;
    groot::Vertex vertex;

    groot::Point_3 position() const
    {
        return graph[vertex].position;
    }

    void set_position(const groot::Point_3& p)
    {
        graph[vertex].position = p;
    }
};

struct Edge {
    groot::PlantGraph& graph;
    groot::Edge e;

    float length() const
    {
        return graph[e].length;
    }
};

struct Vertices {
    groot::PlantGraph& graph;

    static auto make_transform_iterator(Vertices* v, groot::PlantGraph::vertex_iterator v_iter)
    {
        return boost::make_transform_iterator(
            v_iter,
            [v](groot::Vertex vertex) {
                return Vertex { v->graph, vertex };
            });
    }

    auto begin()
    {
        return make_transform_iterator(this, boost::vertices(graph).first);
    }

    auto end()
    {
        return make_transform_iterator(this, boost::vertices(graph).second);
    }
};

struct Edges {
    groot::PlantGraph& graph;

    static auto make_transform_iterator(Edges* e, groot::PlantGraph::edge_iterator e_iter)
    {
        return boost::make_transform_iterator(
            e_iter,
            [e](groot::Edge edge) {
                return Edge { e->graph, edge };
            });
    }
    auto begin()
    {
        return make_transform_iterator(this, boost::edges(graph).first);
    }

    auto end()
    {
        return make_transform_iterator(this, boost::edges(graph).second);
    }
};

Vertices vertices(groot::PlantGraph& graph)
{
    return Vertices { graph };
}

Edges edges(groot::PlantGraph& graph)
{
    return Edges { graph };
}

void create_plant_graph_component()
{
    using namespace boost::python;

    class_<groot::PlantGraph>("PlantGraph", no_init)
        .add_property("num_vertices", &boost::num_vertices<groot::PlantGraph>, "Number of vertices in the graph")
        .add_property(
            "num_edges", +[](const groot::PlantGraph& s) { return boost::num_edges(s); }, "Number of edges in the graph")
        .add_property("vertices", make_function(&vertices, custodian_this()), "Vertices iterator")
        .add_property("edges", make_function(&edges, custodian_this()), "Edges iterator");

    class_<Vertices>("PlantGraphVertices", no_init)
        .add_property(
            "__len__", +[](const Vertices& s) { return boost::num_vertices(s.graph); })
        .def("__iter__", range<custodian_this>(&Vertices::begin, &Vertices::end));

    class_<Edges>("PlantGraphEdges", no_init)
        .add_property(
            "__len__", +[](const Edges& s) { return boost::num_edges(s.graph); })
        .def("__iter__", range<custodian_this>(&Edges::begin, &Edges::end));

    class_<Vertex>("Vertex", no_init)
        .add_property("point", &Vertex::position);

    class_<Edge>("Edge", no_init)
        .add_property("length", &Edge::length);
}