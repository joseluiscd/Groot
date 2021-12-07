#pragma once

#include <groot/groot.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/property_map/property_map.hpp>
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

struct GROOT_LOCAL SearchParams {
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

struct GROOT_LOCAL VertexTag {
};
struct GROOT_LOCAL EdgeTag {
};

GROOT_API void reindex(PlantGraph& graph);
GROOT_API void reindex_vertices(PlantGraph& graph);
GROOT_API void reindex_edges(PlantGraph& graph);

GROOT_API void recompute_edge_length(PlantGraph& graph, Edge e);
GROOT_API void recompute_edge_lengths(PlantGraph& graph);

struct GROOT_LOCAL VertexProperties {
    cgal::Point_3 position = cgal::Point_3(0, 0, 0);
    float root_distance = 0.0;
};

struct GROOT_LOCAL EdgeProperties {
    float length = 0.0;
};

struct GROOT_LOCAL PlantProperties {
    float max_root_distance = 0.0;
    size_t root_index;
};

template <typename T>
using PropertyMap = std::vector<T>;

template <typename T>
inline auto make_vertex_property_map(const PropertyMap<T>& m, const PlantGraph& g)
{
    return boost::make_iterator_property_map(m.begin(), boost::get(boost::vertex_index, g));
}

template <typename T>
inline auto make_edge_property_map(const PropertyMap<T>& m, const PlantGraph& g)
{
    return boost::make_iterator_property_map(m.begin(), boost::get(boost::edge_index, g));
}

template <typename T>
inline auto make_vertex_property_map(PropertyMap<T>& m, const PlantGraph& g)
{
    return boost::make_iterator_property_map(m.begin(), boost::get(boost::vertex_index, g));
}

template <typename T>
inline auto make_edge_property_map(PropertyMap<T>& m, const PlantGraph& g)
{
    return boost::make_iterator_property_map(m.begin(), boost::get(boost::edge_index, g));
}

/// Operations to obtain a certain point in the tree
namespace point_finder {
    struct GROOT_LOCAL PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const = 0;
    };

    Vertex min_coord(const PlantGraph& graph, size_t axis);
    Vertex max_coord(const PlantGraph& graph, size_t axis);

    struct GROOT_LOCAL MinX : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 0);
        }
    };
    extern GROOT_API MinX MinXPointFinder;

    struct GROOT_LOCAL MinY : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 1);
        }
    };
    extern GROOT_API MinY MinYPointFinder;

    struct GROOT_LOCAL MinZ : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 2);
        }
    };
    extern GROOT_API MinZ MinZPointFinder;

    struct GROOT_LOCAL MaxX : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 0);
        }
    };
    extern GROOT_API MaxX MaxXPointFinder;

    struct GROOT_LOCAL MaxY : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 1);
        }
    }; 
    extern GROOT_API MaxY MaxYPointFinder;

    struct GROOT_LOCAL MaxZ : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 2);
        }
    };
    extern GROOT_API MaxZ MaxZPointFinder;
};


GROOT_API PlantGraph empty();
GROOT_API PlantGraph from_delaunay(
    cgal::Point_3* cloud,
    size_t size);
/// Finds the alpha shape and converts to a graph
/// If alpha == 0, the specified number of components are searched
/// If alpha != 0, the last parameter is ignored
GROOT_API PlantGraph from_alpha_shape(
    cgal::Point_3* cloud,
    size_t count,
    float alpha = 0.0f,
    size_t components = 1);
GROOT_API PlantGraph from_search(
    cgal::Point_3* cloud,
    size_t size,
    const SearchParams& search = SearchParams { 3, 0.0, SearchType::kKnnSearch });

GROOT_API PlantGraph from_cardenas_et_al(Point_3* cloud, size_t count, float radius, const point_finder::PointFinder& f = point_finder::MinY());

/// Computes distances to ther root on the original graph and returns simplified version.
GROOT_API PlantGraph geodesic(PlantGraph& g);
/// Computes distances to ther root on the original graph and returns simplified version.
GROOT_API PlantGraph minimum_spanning_tree(PlantGraph& g);

GROOT_LOCAL inline void find_root(PlantGraph& graph, const point_finder::PointFinder& pf = point_finder::MinY())
{
    graph.m_property->root_index = boost::get(boost::vertex_index, graph)[pf(graph)];
}

}
