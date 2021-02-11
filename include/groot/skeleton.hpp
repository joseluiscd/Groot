#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <lemon/list_graph.h>
#include <optional>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <unordered_map>

namespace groot {

using GraphT = lemon::ListGraph;
using NodeT = GraphT::Node;
using EdgeT = GraphT::Edge;
using ArcT = GraphT::Arc;
using NodeIt = GraphT::NodeIt;
using ArcIt = GraphT::ArcIt;
using EdgeIt = GraphT::EdgeIt;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

template <typename T>
using NodeMapT = GraphT::NodeMap<T>;

template <typename T>
using EdgeMapT = GraphT::EdgeMap<T>;

template <typename T>
using ArcMapT = GraphT::ArcMap<T>;

struct PlantGraph;

/// Operations to obtain a certain point in the tree
namespace point_finder {
    struct PointFinder {
        virtual NodeT operator()(const PlantGraph& graph) const = 0;
    };

    NodeT min_coord(const PlantGraph& graph, size_t axis);
    NodeT max_coord(const PlantGraph& graph, size_t axis);

    struct MinX : public PointFinder {
        virtual NodeT operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 0);
        }
    };

    struct MinY : public PointFinder {
        virtual NodeT operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 1);
        }
    };

    struct MinZ : public PointFinder {
        virtual NodeT operator()(const PlantGraph& graph) const
        {
            return min_coord(graph, 2);
        }
    };

    struct MaxX : public PointFinder {
        virtual NodeT operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 0);
        }
    };

    struct MaxY : public PointFinder {
        virtual NodeT operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 1);
        }
    };

    struct MaxZ : public PointFinder {
        virtual NodeT operator()(const PlantGraph& graph) const
        {
            return max_coord(graph, 2);
        }
    };
};

enum class SearchType {
    kKnnSearch = 0,
    kRadiusSearch = 1,
    kCount,
};

extern std::string SearchType_Names[(size_t)SearchType::kCount + 1];

struct SearchParams {
    int k;
    float radius;

    SearchType search;
};

struct PlantGraph {
    std::unique_ptr<GraphT> graph;

    std::unique_ptr<NodeMapT<glm::vec3>> point;
    std::unique_ptr<EdgeMapT<float>> length;
    std::unique_ptr<NodeMapT<float>> radius;

    NodeT root;

    PlantGraph()
        : graph()
        , point()
        , length()
        , radius()
        , root(lemon::INVALID)
    {
    }

    void write_to_file(std::ostream& output);
    static PlantGraph read_from_file(std::istream& input);

    static PlantGraph empty();
    static PlantGraph from_delaunay(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    static PlantGraph from_search(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        const SearchParams& search = SearchParams { 3, 0.0, SearchType::kKnnSearch },
        pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr _kdtree = nullptr);

    PlantGraph& find_root(const point_finder::PointFinder& pf = point_finder::MinY())
    {
        root = pf(*this);
        return *this;
    }

    PlantGraph& geodesic();
    PlantGraph& minimum_spanning_tree();
};

struct SkeletonNode {
    glm::vec3 center;
    float radius;
};

class Skeleton {

public:
    Skeleton();

    /// Root node
    NodeT root() const;

    /// Get the `SkeletonNode` data inside the `Skeleton`
    SkeletonNode node(const NodeT& node) const;
    float radius(const NodeT& node) const;
    glm::vec3 center(const NodeT& node) const;

private:
    GraphT _graph;
    NodeMapT<float> _radius;
    NodeMapT<glm::vec3> _centers;
};

}