#include "python_graph.hpp"
#include <boost/iterator/transform_iterator.hpp>
#include <groot_graph/plant_graph.hpp>

namespace py = pybind11;

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

void create_plant_graph_component(py::module& m)
{

    py::class_<groot::PlantGraph>(m, "PlantGraph")
        .def_property_readonly("num_vertices", [](const groot::PlantGraph& s) {
            return boost::num_vertices(s);
        }, "Number of vertices in the graph")
        .def_property_readonly(
            "num_edges", [](const groot::PlantGraph& s) { return boost::num_edges(s); }, "Number of edges in the graph")
        .def_property_readonly("vertices", &vertices, "Vertices iterator")
        .def_property_readonly("edges", &edges, "Edges iterator");

    py::class_<Vertices>(m, "PlantGraphVertices")
        .def(
            "__len__", [](const Vertices& s) { return boost::num_vertices(s.graph); })
        .def("__iter__", [](Vertices& s) {
            return py::make_iterator(s.begin(), s.end());
        }, py::keep_alive<0, 1>());

    py::class_<Edges>(m, "PlantGraphEdges")
        .def(
            "__len__", +[](const Edges& s) { return boost::num_edges(s.graph); })
        .def("__iter__", [](Edges& s) {
            return py::make_iterator(s.begin(), s.end());
        }, py::keep_alive<0, 1>());

    py::class_<Vertex>(m, "Vertex")
        .def_property_readonly("point", &Vertex::position);

    py::class_<Edge>(m, "Edge")
        .def_property_readonly("length", &Edge::length);
}