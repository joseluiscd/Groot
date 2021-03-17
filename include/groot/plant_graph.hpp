#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <glm/glm.hpp>
#include <groot/cgal.hpp>
#include <iostream>
#include <optional>
#include <unordered_map>

namespace groot {

enum class SearchType {
    kKnnSearch = 0,
    kRadiusSearch = 1,
    kCount,
};

struct SearchParams {
    int k;
    float radius;

    SearchType search;
};

struct VertexProperties;
struct EdgeProperties;
struct PlantProperties;

using PlantGraph = boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
    VertexProperties,
    boost::property<boost::edge_index_t, size_t, EdgeProperties>,
    PlantProperties>;

using Vertex = PlantGraph::vertex_descriptor;
using Edge = PlantGraph::edge_descriptor;

void reindex(PlantGraph& graph);
void reindex_vertices(PlantGraph& graph);
void reindex_edges(PlantGraph& graph);

struct VertexProperties {
    cgal::Point_3 position = cgal::Point_3(0, 0, 0);
    float radius = 0.0;
    float root_distance = 0.0;
};

struct EdgeProperties {
    float length = 0.0;
};

struct PlantProperties {
    float max_root_distance = 0.0;
    size_t root_index;
};

/// Operations to obtain a certain point in the tree
namespace point_finder {
    struct PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const = 0;
    };

    Vertex min_coord(const PlantGraph& graph, size_t axis);
    Vertex max_coord(const PlantGraph& graph, size_t axis);

    struct MinX : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 0);
        }
    };

    struct MinY : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 1);
        }
    };

    struct MinZ : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 2);
        }
    };

    struct MaxX : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 0);
        }
    };

    struct MaxY : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 1);
        }
    };

    struct MaxZ : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 2);
        }
    };
};

void write_to_file(const PlantGraph& g, std::ostream& output);
PlantGraph read_from_file(std::istream& input);

PlantGraph empty();
PlantGraph from_delaunay(
    cgal::Point_3* cloud,
    size_t size);
PlantGraph from_alpha_shape(
    cgal::Point_3* cloud,
    size_t count,
    float alpha = 0.0f);
PlantGraph from_search(
    cgal::Point_3* cloud,
    size_t size,
    const SearchParams& search = SearchParams { 3, 0.0, SearchType::kKnnSearch });

/// Computes distances to ther root on the original graph and returns simplified version.
PlantGraph geodesic(PlantGraph& g);
/// Computes distances to ther root on the original graph and returns simplified version.
PlantGraph minimum_spanning_tree(PlantGraph& g);

inline void find_root(PlantGraph& graph, const point_finder::PointFinder& pf = point_finder::MinY())
{
    graph.m_property->root_index = boost::get(boost::vertex_index, graph)[pf(graph)];
}

}

namespace boost {
namespace serialization {
    template <class Archive>
    void serialize(Archive& ar, groot::cgal::Point_3& point, unsigned version)
    {
        float x = point.x();
        float y = point.y();
        float z = point.z();

        ar& x& y& z;

        point = groot::cgal::Point_3(x, y, z);
    }

    template <class Archive>
    void serialize(Archive& ar, groot::VertexProperties& props, unsigned /*version*/)
    {
        ar& props.position& props.root_distance& props.radius;
    }

    template <class Archive>
    void serialize(Archive& ar, groot::EdgeProperties& props, unsigned /*version*/)
    {
        ar& props.length;
    }

    template <class Archive>
    void serialize(Archive& ar, groot::PlantProperties& props, unsigned /*version*/)
    {
        ar& props.max_root_distance& props.root_index;
    }
}
}
