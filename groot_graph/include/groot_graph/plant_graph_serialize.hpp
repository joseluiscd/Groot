#pragma once

#include <groot/groot.hpp>
#include <cereal/cereal.hpp>
#include <groot_graph/plant_graph.hpp>
#include <groot_graph/cylinder_marching.hpp>

namespace groot {

GROOT_API void write_to_file(const PlantGraph& g, std::ostream& output);
GROOT_API PlantGraph read_from_file(std::istream& input);

template <class Archive>
void serialize(Archive& ar, VertexProperties& props)
{
    ar(props.position, props.root_distance);
}

template <class Archive>
void serialize(Archive& ar, EdgeProperties& props)
{
    ar(props.length);
}

template <class Archive>
void serialize(Archive& ar, PlantProperties& props)
{
    ar(props.max_root_distance, props.root_index);
}

template <typename Archive>
void serialize(Archive& ar, Cylinder& cylinder)
{
    ar(cylinder.center, cylinder.direction, cylinder.radius, cylinder.middle_height);
}

template <typename Archive>
void serialize(Archive& ar, CylinderWithPoints& cylinder)
{
    ar(cylinder.cylinder, cylinder.points);
}

/// Copied and modified from <boost/graph/adj_list_serialize.hpp>
template <class Archive>
void save(Archive& ar, const PlantGraph& graph)
{
    int V = boost::num_vertices(graph);
    int E = boost::num_edges(graph);
    ar(CEREAL_NVP(V));
    ar(CEREAL_NVP(E));

    // assign indices to vertices
    std::map<Vertex, int> indices;
    int num = 0;
    for (auto [it, end] = boost::vertices(graph); it != end; ++it) {
        indices[*it] = num++;
        ar(cereal::make_nvp("vertex_property", graph[*it]));
    }

    // write edges
    for (auto [it, end] = boost::edges(graph); it != end; ++it) {
        ar(cereal::make_nvp("u", indices[boost::source(*it, graph)]));
        ar(cereal::make_nvp("v", indices[boost::target(*it, graph)]));
        ar(cereal::make_nvp("edge_property", graph[*it]));
    }

    ar(cereal::make_nvp("graph_property", (const groot::PlantProperties&)boost::get_property(graph, boost::graph_all)));
}

/// Copied and modified from <boost/graph/adj_list_serialize.hpp>
template <class Archive>
void load(Archive& ar, PlantGraph& graph)
{
    unsigned int V;
    ar(CEREAL_NVP(V));
    unsigned int E;
    ar(CEREAL_NVP(E));

    std::vector<Vertex> verts(V);
    int i = 0;
    while (V-- > 0) {
        Vertex v = boost::add_vertex(graph);
        verts[i++] = v;
        ar(cereal::make_nvp("vertex_property", graph[v]));
    }
    while (E-- > 0) {
        int u;
        int v;
        ar(CEREAL_NVP(u));
        ar(CEREAL_NVP(v));
        Edge e;
        bool inserted;
        boost::tie(e, inserted) = boost::add_edge(verts[u], verts[v], graph);
        ar(cereal::make_nvp("edge_property", graph[e]));
    }
    ar(cereal::make_nvp("graph_property", boost::get_property(graph, boost::graph_all_t())));
}

}

namespace cereal {

template <class Archive>
struct specialize<Archive, groot::PlantProperties, cereal::specialization::non_member_serialize> {};


template <class Archive>
void load(Archive& ar, groot::cgal::Point_3& point)
{
    float x, y, z;
    ar(x, y, z);
    point = groot::cgal::Point_3(x, y, z);
}

template <class Archive>
void save(Archive& ar, const groot::cgal::Point_3& point)
{
    ar(point.x(), point.y(), point.z());
}

template <class Archive>
void load(Archive& ar, groot::cgal::Vector_3& point)
{
    float x, y, z;
    ar(x, y, z);
    point = groot::cgal::Vector_3(x, y, z);
}

template <class Archive>
void save(Archive& ar, const groot::cgal::Vector_3& point)
{
    ar(point.x(), point.y(), point.z());
}

}