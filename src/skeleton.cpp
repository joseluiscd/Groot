#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <groot/cgal_helper.hpp>
#include <groot/reconstruction.hpp>
#include <groot/util.hpp>
#include <lemon/adaptors.h>
#include <lemon/dijkstra.h>
#include <lemon/kruskal.h>
#include <lemon/lgf_reader.h>
#include <lemon/lgf_writer.h>
#include <optional>
#include <pcl/cloud_iterator.h>
#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <spdlog/spdlog.h>

namespace groot {

std::string SearchType_Names[] = {
    "KnnSearch",
    "RadiusSearch",
    "Count"
};

namespace point_finder {
    NodeT min_coord(const PlantGraph& graph, size_t axis)
    {
        NodeT point = lemon::INVALID;
        float min_coord = +INFINITY;

        for (NodeIt n(*graph.graph); n != lemon::INVALID; ++n) {
            if ((*graph.point)[n][axis] < min_coord) {
                point = n;
                min_coord = (*graph.point)[n][axis];
            }
        }

        return point;
    }

    NodeT max_coord(const PlantGraph& graph, size_t axis)
    {
        NodeT point = lemon::INVALID;
        float max_coord = -INFINITY;

        for (NodeIt n(*graph.graph); n != lemon::INVALID; ++n) {
            if ((*graph.point)[n][axis] > max_coord) {
                point = n;
                max_coord = (*graph.point)[n][axis];
            }
        }

        return point;
    }
} // namespace point_finder

PlantGraph PlantGraph::from_delaunay(Cloud::ConstPtr cloud)
{
    cgal::Delaunay delaunay;

    GraphT* neighbourhood(new GraphT);
    NodeMapT<glm::vec3>* points = new NodeMapT<glm::vec3>(*neighbourhood);
    EdgeMapT<float>* lengths = new EdgeMapT<float>(*neighbourhood);

    for (pcl::ConstCloudIterator<pcl::PointXYZ> it(*cloud); it; it++) {
        NodeT node = neighbourhood->addNode();
        (*points)[node] = glm::vec3(it->x, it->y, it->z);

        cgal::Point_3 point(it->x, it->y, it->z);
        cgal::Delaunay::Vertex_handle handle = delaunay.insert(point);

        handle->info() = node;
    }

    for (auto i = delaunay.finite_edges_begin(); i != delaunay.finite_edges_end(); i++) {
        cgal::Delaunay::Vertex_handle v1 = i->first->vertex(i->second);
        cgal::Delaunay::Vertex_handle v2 = i->first->vertex(i->third);

        double distance = CGAL::squared_distance(v1->point(), v2->point());

        NodeT n1 = v1->info();
        NodeT n2 = v2->info();

        lemon::ListGraph::Edge e = neighbourhood->addEdge(n1, n2);
        (*lengths)[e] = distance;
    }

    PlantGraph ret;

    ret.graph.reset(neighbourhood);
    ret.point.reset(points);
    ret.length.reset(lengths);
    ret.radius.reset(new NodeMapT<float>(*neighbourhood, 0));

    return ret;
}

PlantGraph PlantGraph::from_search(
    Cloud::ConstPtr cloud,
    const SearchParams& params,
    pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr _kdtree)
{
    size_t point_count = cloud->size();

    GraphT* neighbourhood(new GraphT);
    NodeMapT<glm::vec3>* points = new NodeMapT<glm::vec3>(*neighbourhood);
    EdgeMapT<float>* lengths = new EdgeMapT<float>(*neighbourhood);

    std::vector<NodeT> point_nodes;

    std::transform(
        cloud->begin(),
        cloud->end(),
        std::back_inserter(point_nodes),
        [&neighbourhood, &points](const pcl::PointXYZ& item) {
            NodeT node = neighbourhood->addNode();
            (*points)[node] = glm::vec3(item.x, item.y, item.z);

            return node;
        });

    pcl::KdTreeFLANN<pcl::PointXYZ>::ConstPtr kdtree;

    if (_kdtree == nullptr) {
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tmp_kdtree
            = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();

        tmp_kdtree->setInputCloud(cloud);

        kdtree = tmp_kdtree;
    } else {
        kdtree = _kdtree;
    }

    for (size_t point_i = 0; point_i < point_count; point_i++) {
        std::vector<int> result;
        std::vector<float> dist_pow_2;

        switch (params.search) {
        case SearchType::kRadiusSearch:
            kdtree->radiusSearch((*cloud)[point_i], params.radius, result, dist_pow_2, params.k);
            break;

        case SearchType::kKnnSearch:
            kdtree->nearestKSearch((*cloud)[point_i], params.k, result, dist_pow_2);
            break;
        default:
            break;
        }

        for (size_t j = 0; j < result.size(); j++) {
            // Do not link a node to itself
            if (result[j] != point_i) {
                EdgeT a = neighbourhood->addEdge(
                    point_nodes[point_i],
                    point_nodes[result[j]]);

                float distance = pcl::geometry::distance(
                    (*cloud)[point_i],
                    (*cloud)[result[j]]);

                (*lengths)[a] = distance;
            }
        }
    }

    PlantGraph ret;

    ret.graph.reset(neighbourhood);
    ret.point.reset(points);
    ret.length.reset(lengths);
    ret.radius.reset(new NodeMapT<float>(*neighbourhood, 0));

    return ret;
}

PlantGraph PlantGraph::empty()
{
    PlantGraph ret;

    GraphT* neighbourhood(new GraphT);

    ret.graph.reset(neighbourhood);
    ret.point.reset(new NodeMapT<glm::vec3>(*neighbourhood));
    ret.length.reset(new EdgeMapT<float>(*neighbourhood));
    ret.radius.reset(new NodeMapT<float>(*neighbourhood));

    return ret;
}

PlantGraph PlantGraph::read_from_file(std::istream& input)
{
    GraphT* graph = new GraphT;
    auto points = new NodeMapT<glm::vec3>(*graph);
    auto radii = new NodeMapT<float>(*graph);
    auto lengths = new EdgeMapT<float>(*graph);

    NodeT root_node;

    lemon::graphReader(*graph, input)
        .nodeMap("point", *points, GlmFromString<glm::vec3>())
        .nodeMap("radius", *radii)
        .edgeMap("length", *lengths)
        .node("root", root_node)
        .run();

    PlantGraph ret;

    ret.graph.reset(graph);
    ret.point.reset(points);
    ret.length.reset(lengths);
    ret.radius.reset(new NodeMapT<float>(*graph, 0));
    ret.root = root_node;

    return ret;
}

void PlantGraph::write_to_file(std::ostream& output)
{
    lemon::graphWriter(*this->graph, output)
        .nodeMap("point", *this->point, GlmToString<glm::vec3>())
        .nodeMap("radius", *this->radius)
        .edgeMap("length", *this->length)
        .node("root", this->root)
        .run();
}

PlantGraph& PlantGraph::geodesic()
{
    NodeMapT<float> distances(*graph);
    NodeMapT<ArcT> pred(*graph);

    // Compute minimum weight tree from the root
    lemon::Dijkstra<GraphT, EdgeMapT<float>> path_calculator(*graph, *length);

    assert(root != lemon::INVALID);

    path_calculator
        .distMap(distances)
        .predMap(pred)
        .run(root);

    EdgeMapT<bool> shortest_paths(*graph, false);
    for (NodeIt n(*graph); n != lemon::INVALID; ++n) {
        if (pred[n] != lemon::INVALID) {
            shortest_paths[pred[n]] = true;
        }
    }

    auto filtered = lemon::filterEdges(*graph, shortest_paths);

    PlantGraph dest = PlantGraph::empty();

    lemon::graphCopy(filtered, *dest.graph)
        .nodeMap(*point, *dest.point)
        .nodeMap(*radius, *dest.radius)
        .edgeMap(*length, *dest.length)
        .node(root, dest.root)
        .run();

    *this = std::move(dest);
    return *this;
}

PlantGraph& PlantGraph::minimum_spanning_tree()
{
    EdgeMapT<bool> min_cost(*graph, false);
    lemon::kruskal(*graph, *length, min_cost);

    auto filtered = lemon::filterEdges(*graph, min_cost);

    PlantGraph dest = PlantGraph::empty();

    lemon::graphCopy(filtered, *dest.graph)
        .nodeMap(*point, *dest.point)
        .nodeMap(*radius, *dest.radius)
        .edgeMap(*length, *dest.length)
        .node(root, dest.root)
        .run();

    *this = std::move(dest);
    return *this;
}

} // namespace groot
