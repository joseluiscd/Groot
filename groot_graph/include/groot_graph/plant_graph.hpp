#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/property_map/property_map.hpp>
#include <glm/glm.hpp>
#include <groot/cgal.hpp>
#include <groot_graph/groot_graph.hpp>
#include <iostream>
#include <optional>
#include <unordered_map>

namespace groot {

struct VertexProperties;
struct EdgeProperties;
struct PlantProperties;

using PlantGraph = boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
    VertexProperties,
    boost::property<boost::edge_index_t, size_t, EdgeProperties>,
    PlantProperties>;

using Vertex = PlantGraph::vertex_descriptor;
using Edge = PlantGraph::edge_descriptor;

struct GROOT_GRAPH_LOCAL VertexTag {
};
struct GROOT_GRAPH_LOCAL EdgeTag {
};

GROOT_GRAPH_API void reindex(PlantGraph& graph);
GROOT_GRAPH_API void reindex_vertices(PlantGraph& graph);
GROOT_GRAPH_API void reindex_edges(PlantGraph& graph);

GROOT_GRAPH_API void recompute_edge_length(PlantGraph& graph, Edge e);
GROOT_GRAPH_API void recompute_edge_lengths(PlantGraph& graph);

struct GROOT_GRAPH_LOCAL VertexProperties {
    cgal::Point_3 position = cgal::Point_3(0, 0, 0);
};

struct GROOT_GRAPH_LOCAL EdgeProperties {
    float length = 0.0;
};

struct GROOT_GRAPH_LOCAL PlantProperties {
    //float max_root_distance = 0.0;
    size_t root_index = 0;
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
    struct GROOT_GRAPH_LOCAL PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const = 0;
    };

    GROOT_GRAPH_API Vertex min_coord(const PlantGraph& graph, size_t axis);
    GROOT_GRAPH_API Vertex max_coord(const PlantGraph& graph, size_t axis);

    struct GROOT_GRAPH_LOCAL MinX : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 0);
        }
    };
    extern GROOT_GRAPH_API MinX MinXPointFinder;

    struct GROOT_GRAPH_LOCAL MinY : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 1);
        }
    };
    extern GROOT_GRAPH_API MinY MinYPointFinder;

    struct GROOT_GRAPH_LOCAL MinZ : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 2);
        }
    };
    extern GROOT_GRAPH_API MinZ MinZPointFinder;

    struct GROOT_GRAPH_LOCAL MaxX : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 0);
        }
    };
    extern GROOT_GRAPH_API MaxX MaxXPointFinder;

    struct GROOT_GRAPH_LOCAL MaxY : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 1);
        }
    };
    extern GROOT_GRAPH_API MaxY MaxYPointFinder;

    struct GROOT_GRAPH_LOCAL MaxZ : public PointFinder {
        virtual Vertex operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 2);
        }
    };
    extern GROOT_GRAPH_API MaxZ MaxZPointFinder;
};

GROOT_GRAPH_API PlantGraph empty();
GROOT_GRAPH_API PlantGraph from_delaunay(
    cgal::Point_3* cloud,
    size_t size);

/**
 * @brief Alpha shape of the input cloud, converted to a graph.
 * @param cloud Input point cloud
 * @param count Number of points in the cloud
 * @param alpha Alpha value.
 *   If 0, the specified number of components are searched.
 *   Else, the last parameter is ignored.
 * @param components If `alpha == 0`, computes `alpha` to obtain
 *   this number of connected components.
 */
GROOT_GRAPH_API PlantGraph from_alpha_shape(
    cgal::Point_3* cloud,
    size_t count,
    float alpha = 0.0f,
    size_t components = 1);

GROOT_GRAPH_API PlantGraph from_search_knn(
    cgal::Point_3* cloud,
    size_t size,
    size_t k);

GROOT_GRAPH_API PlantGraph from_search_radius(
    cgal::Point_3* cloud,
    size_t size,
    float radius);

GROOT_GRAPH_API PlantGraph from_cardenas_et_al(Point_3* cloud, size_t count, float radius, const point_finder::PointFinder& f = point_finder::MinY());

/**
 * @brief Computes distances to ther root on the original graph and returns simplified version.
 * @param[in] g Graph to compute the geodesic graph.
 * @param[out] distance_map Property map with distances to the root. If non-NULL, it is created.
 * @param[out] max_distance Max distance to the root. If non-NULL, it is created.
 * @return Geodesic graph of `g`.
 */
GROOT_GRAPH_API PlantGraph geodesic(const PlantGraph& g, PropertyMap<float>* distance_map = nullptr, float* max_distance = nullptr);

/**
 * @brief Computes minimum spanning tree of input graph
 * @param[in] g Graph to compute the MST.
 * @param[out] distance_map Property map with distances to the root. If non-NULL, it is created.
 * @param[out] max_distance Max distance to the root. If non-NULL, it is created.
 * @return Minimum spanning tree of the graph `g`.
 */
GROOT_GRAPH_API PlantGraph minimum_spanning_tree(const PlantGraph& g, PropertyMap<float>* distance_map = nullptr, float* max_distance = nullptr);

GROOT_GRAPH_LOCAL inline void find_root(PlantGraph& graph, const point_finder::PointFinder& pf = point_finder::MinY())
{
    graph.m_property->root_index = boost::get(boost::vertex_index, graph)[pf(graph)];
}

}
