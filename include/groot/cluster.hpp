#pragma once

#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <groot/reconstruction.hpp>
#include <groot/skeleton.hpp>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/adaptors.h>
#include <vector>

namespace groot {

enum SearchType {
    Knn, Radius
};

struct ClusterParams {
    int k;
    float radius;

    int search;

    int root;
};

template<typename T>
class ClusterReconstruction: Reconstruction<T> {
    using CloudConstPtr = pcl::PointCloud<T>::ConstPtr;
    using KdTree = pcl::KdTreeFLANN<T>;

    public:
    ClusterReconstruction(const CloudConstPtr &cloud, const ClusterParams& params)
        : _cloud(cloud);

    /// If the kd-tree has been computed previously, pass it to the algorithm.
    /// Otherwise, it is computed automatically.
    void setKdTree(const pcl::KdTreeFLANN<T> &kdtree);

    Skeleton run();

    private:
    CloudConstPtr _cloud;
    ClusterParams _params;
    std::optional<KdTree> _kdtree;
};


template<typename T>
Skeleton ClusterReconstruction<T>::run() {
    CloudConstPtr cloud = this->_cloud;

    size_t point_count = cloud->size();

    GraphT neighbourhood;
    NodeMap<T> points(neighbourhood);
    ArcMap<float> lengths;
    NodeT root;

    { // Compute mappings, neighbourhood and lengths
        std::vector<NodeT> point_nodes;

        std::transform(
            cloud->begin(),
            cloud->end(),
            std::back_inserter(point_nodes),
            [&neighbourhood](const T &item) {
                return neighbourhood.addNode();
            });

        for (int i = 0; i < point_nodes.size(); i++) {
            points[point_nodes[i]] = cloud[i];
        }

        root = point_nodes[this->_params.root];

        KdTree &kdtree = this->_kdtree ? this->_kdtree
                                       : [const & ]() {
                                             KdTree kdtree;
                                             kdtree.setInputCloud(cloud);

                                             return kdtree;
                                         }();

        for (size_t point_i = 0; point_i < point_count; point_i++) {
            std::vector<int> result;
            std::vector<float> dist_pow_2;

            switch (_params.search) {
            case SearchType::Radius:
                kdtree.radiusSearch(cloud[point_i], _params.radius, result, dist_pow_2, _params.k);
                break;

            case SearchType::Knn:
                kdtree.nearestKSearch(cloud[point_i], _params.k, result, dist_pow_2);
                break;
            }

            for (size_t j = 0; j < result.size(); j++) {
                if (result[j] != point_i)
                { // Do not link a node to itself
                    ArcT a = neighbourhood.addArc(
                        point_nodes[point_i],
                        point_nodes[result[j]]);

                    float distance = pcl::geometry::distance(
                        cloud[point_i],
                        cloud[result[j]]);
                    
                    lengths[a] = distance;
                }
            }
        }
    }

    NodeMap<float> distances;
    NodeMap<bool> pred;

    // Compute minimum weight tree from the root
    lemon::Dijkstra<GraphT> path_calculator(neighbourhood, lengths);
    path_calculator
        .distMap(distances)
        .predMap(pred)
        .run(root);
    
    ArcMap<bool> shortest_paths(false);
    for (NodeIt n(neighbourhood); n != lemon::INVALID; ++n) {
        shortest_paths[pred[*n]] = true;
    }
    
    for (ArcIt)
}

}