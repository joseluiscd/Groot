#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <ceres/ceres.h>
#include <glm/glm.hpp>
#include <groot_graph/livny_et_al.hpp>
#include <spdlog/spdlog.h>

namespace groot {

struct BFSSortVisitor : public boost::default_bfs_visitor {
    BFSSortVisitor(std::vector<Vertex>& _vertices)
        : vertices(_vertices)
    {
    }

    void discover_vertex(Vertex v, const PlantGraph& g)
    {
        vertices.push_back(v);
    }

    std::vector<Vertex> vertices;
};

std::vector<Vertex> sort_vertices(const PlantGraph& g)
{
    std::vector<Vertex> vertices;
    vertices.reserve(boost::num_vertices(g));

    boost::breadth_first_search(g, g.m_property->root_index, boost::visitor(BFSSortVisitor(vertices)));

    return vertices;
}

struct TreeLengthVisitor : public boost::default_dfs_visitor {
    TreeLengthVisitor(PropertyMap<float>& _lengths, PropertyMap<Vertex>& _predecessors)
        : lengths(_lengths)
        , predecessors(_predecessors)
    {
    }

    void examine_edge(Edge e, const PlantGraph& g)
    {
        lengths[boost::source(e, g)] += g[e].length;
        predecessors[boost::target(e, g)] = boost::source(e, g);
    }

    void finish_edge(Edge e, const PlantGraph& g)
    {
        if (predecessors[boost::source(e, g)] == boost::target(e, g)) {
            return;
        }

        lengths[boost::source(e, g)] += lengths[boost::target(e, g)];
    }

    PropertyMap<float>& lengths;
    PropertyMap<Vertex>& predecessors;
};

/// TODO: Debug
PropertyMap<float> compute_weights(const PlantGraph& g)
{
    PropertyMap<float> lenghts(boost::num_vertices(g));
    PropertyMap<Vertex> predecessors(boost::num_vertices(g));
    auto visitor = TreeLengthVisitor(lenghts, predecessors);
    std::vector<int> color(boost::num_vertices(g));
    auto color_map = boost::make_iterator_property_map(color.begin(), boost::get(boost::vertex_index, g));

    boost::depth_first_search(g, visitor, color_map, g.m_property->root_index);

    return lenghts;
}

struct ParentOrientationCostFunction {
    ParentOrientationCostFunction(double _weight, double _parent_weight, double* parent)
        : sq_weight((_weight + _parent_weight) / 2.0)
        , op(parent)
    {
        sq_weight *= sq_weight;
    }

    template <typename T>
    bool operator()(const T* o, T* residual) const
    {
        T diff_x = op[0] - o[0];
        T diff_y = op[1] - o[1];
        T diff_z = op[2] - o[2];

        T k = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
        *residual = sq_weight * k;

        return true;
    }

    double sq_weight;
    double* op;
};

struct EdgeCostFunction {
    EdgeCostFunction(double _weight, glm::dvec3 _edge_direction)
        : sq_weight(_weight * _weight)
        , edge_direction(_edge_direction)
    {
    }

    template <typename T>
    bool operator()(const T* o, T* residual) const
    {
        T diff_x = o[0] - edge_direction[0];
        T diff_y = o[1] - edge_direction[1];
        T diff_z = o[2] - edge_direction[2];

        T k = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

        *residual = sq_weight * k;

        return true;
    }

    double sq_weight;
    glm::dvec3 edge_direction;
};

PropertyMap<glm::dvec3> compute_orientation_field(const PlantGraph& g, const PropertyMap<float>& weights)
{
    ceres::Problem problem;
    PropertyMap<glm::dvec3> orientations(boost::num_vertices(g));

    for (auto [it, end] = boost::vertices(g); it != end; ++it) {
        problem.AddParameterBlock(&orientations[*it].x, 3, new ceres::SphereManifold<3>);
        orientations[*it] = glm::dvec3(0.0, 1.0, 0.0);
    }

    for (auto [it, end] = boost::edges(g); it != end; ++it) {
        Vertex op = boost::source(*it, g);
        Vertex o = boost::target(*it, g);

        if (weights[o] > weights[op]) {
            std::swap(o, op);
        }

        Vector_3 _ed = g[o].position - g[op].position;
        if (_ed.squared_length() < 0.00000000001) {
            throw std::runtime_error("Some edges have 0 length");
        }

        glm::dvec3 edge_direction(_ed.x(), _ed.y(), _ed.z());
        edge_direction = glm::normalize(edge_direction);

        orientations[o] = edge_direction; 

        ceres::CostFunction* f_parent_orientation = new ceres::AutoDiffCostFunction<ParentOrientationCostFunction, 1, 3>(
            new ParentOrientationCostFunction(weights[o], weights[op], &orientations[op].x));

        ceres::CostFunction* f_edge = new ceres::AutoDiffCostFunction<EdgeCostFunction, 1, 3>(
            new EdgeCostFunction(weights[o], edge_direction));

        problem.AddResidualBlock(f_parent_orientation, nullptr, &orientations[o].x);
        problem.AddResidualBlock(f_edge, nullptr, &orientations[o].x);
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    spdlog::info(summary.BriefReport());

    return orientations;
}

groot::PlantGraph reconstruct_livny_et_al(const Point_3* cloud, size_t size, point_finder::PointFinder& root_finder)
{
    PlantGraph initial = from_delaunay(cloud, size);
    PlantGraph mst = minimum_spanning_tree(initial);

    find_root(mst, root_finder);

    PropertyMap<float> vertex_weights = compute_weights(mst);
    PropertyMap<glm::dvec3> orientations = compute_orientation_field(mst, vertex_weights);

    return mst;
}

}