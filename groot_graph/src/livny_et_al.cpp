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
    ParentOrientationCostFunction(double _weight, double _parent_weight)
        : weight((_weight + _parent_weight) / 2.0)
    {
    }

    template <typename T>
    bool operator()(const T* o, const T* op, T* residual) const
    {
        residual[0] = weight * (op[0] - o[0]);
        residual[1] = weight * (op[1] - o[1]);
        residual[2] = weight * (op[2] - o[2]);

        return true;
    }

    double weight;
};

struct EdgeCostFunction {
    EdgeCostFunction(double _weight, glm::dvec3 _edge_direction)
        : weight(_weight)
        , edge_direction(_edge_direction)
    {
    }

    template <typename T>
    bool operator()(const T* o, T* residual) const
    {
        residual[0] = weight * (o[0] - edge_direction[0]);
        residual[1] = weight * (o[1] - edge_direction[1]);
        residual[2] = weight * (o[2] - edge_direction[2]);

        return true;
    }

    double weight;
    glm::dvec3 edge_direction;
};

PropertyMap<glm::dvec3> compute_orientation_field(const PlantGraph& g, const PropertyMap<float>& weights, unsigned max_iterations)
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

        ceres::CostFunction* f_parent_orientation = new ceres::AutoDiffCostFunction<ParentOrientationCostFunction, 3, 3, 3>(
            new ParentOrientationCostFunction(weights[o], weights[op]));

        ceres::CostFunction* f_edge = new ceres::AutoDiffCostFunction<EdgeCostFunction, 3, 3>(
            new EdgeCostFunction(weights[o], edge_direction));

        problem.AddResidualBlock(f_parent_orientation, nullptr, &orientations[o].x, &orientations[op].x);
        problem.AddResidualBlock(f_edge, nullptr, &orientations[o].x);
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.max_num_iterations = max_iterations;

#ifndef NDEBUG
    options.minimizer_progress_to_stdout = true;
#endif

    ceres::Solve(options, &problem, &summary);

    spdlog::info(summary.BriefReport());

    return orientations;
}

struct OrientationUpdateCostFunction {
    inline OrientationUpdateCostFunction(
        const glm::dvec3& orig_u,
        const glm::dvec3& orig_v,
        const glm::dvec3& orientation_u,
        const glm::dvec3& orientation_v,
        double u_weight,
        double v_weight)

        : weight(0.5 * (u_weight + v_weight))
        , second((glm::length(orig_u - orig_v) * (orientation_u + orientation_v)) / (glm::length(orientation_u + orientation_v)))
    {
    }

    template <typename T>
    bool operator()(const T* u, const T* v, T* residual) const
    {
        residual[0] = weight * (u[0] - v[0] - second[0]);
        residual[1] = weight * (u[1] - v[1] - second[1]);
        residual[2] = weight * (u[2] - v[2] - second[2]);
        return true;
    }

    double weight;
    glm::dvec3 second;
};

struct OrientationConstraintCostFunction {
    inline OrientationConstraintCostFunction(
        const glm::dvec3& orig_u,
        const glm::dvec3& orig_v,
        const glm::dvec3& orientation_u,
        const glm::dvec3& orientation_v,
        double u_weight,
        double v_weight)

        : weight(v_weight)
        , original(0.5 * (orig_u + orig_v))
    {
    }

    template <typename T>
    bool operator()(const T* u, const T* v, T* residual) const
    {
        residual[0] = weight * (0.5 * (u[0] + v[0]) - original[0]);
        residual[1] = weight * (0.5 * (u[1] + v[1]) - original[1]);
        residual[2] = weight * (0.5 * (u[2] + v[2]) - original[2]);
        return true;
    }

    double weight;
    glm::dvec3 original;
};

void update_positions_for_orientation(
    PlantGraph& g,
    const PropertyMap<float>& weights,
    const PropertyMap<glm::dvec3>& orientations,
    unsigned max_iterations)
{
    ceres::Problem problem;

    std::vector<glm::dvec3> positions(boost::num_vertices(g));
    for (auto [it, end] = boost::vertices(g); it != end; ++it) {
        positions[*it] = glm::dvec3(g[*it].position.x(), g[*it].position.y(), g[*it].position.z());
    }

    for (auto [it, end] = boost::edges(g); it != end; ++it) {
        Vertex source = boost::source(*it, g);
        Vertex target = boost::target(*it, g);

        if (weights[target] > weights[source]) {
            std::swap(target, source);
        }

        ceres::CostFunction* f1 = new ceres::AutoDiffCostFunction<OrientationUpdateCostFunction, 3, 3, 3>(
            new OrientationUpdateCostFunction(
                positions[source], positions[target],
                orientations[source], orientations[target],
                weights[source], weights[target]));

        ceres::CostFunction* f2 = new ceres::AutoDiffCostFunction<OrientationConstraintCostFunction, 3, 3, 3>(
            new OrientationConstraintCostFunction(
                positions[source], positions[target],
                orientations[source], orientations[target],
                weights[source], weights[target]));

        problem.AddResidualBlock(f1, nullptr, &positions[source].x, &positions[target].x);
        problem.AddResidualBlock(f2, nullptr, &positions[source].x, &positions[target].x);
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.max_num_iterations = max_iterations;

#ifndef NDEBUG
    options.minimizer_progress_to_stdout = true;
#endif

    ceres::Solve(options, &problem, &summary);

    spdlog::info(summary.BriefReport());

    for (auto [it, end] = boost::vertices(g); it != end; ++it) {
        g[*it].position = Point_3(positions[*it].x, positions[*it].y, positions[*it].z);
    }
}

PlantGraph iteration_livny_et_al(
    const PlantGraph& g,
    point_finder::PointFinder& root_finder,
    unsigned max_orientation_iterations,
    unsigned max_relocation_iterations)
{
    PlantGraph mst = rebuild_minimum_spanning_tree(g);
    find_root(mst, root_finder);

    PropertyMap<float> vertex_weights = compute_weights(mst);
    PropertyMap<glm::dvec3> orientations = compute_orientation_field(mst, vertex_weights, max_orientation_iterations);

    update_positions_for_orientation(mst, vertex_weights, orientations, max_relocation_iterations);
    return mst;
}

groot::PlantGraph reconstruct_livny_et_al(
    const Point_3* cloud,
    size_t size,
    point_finder::PointFinder& root_finder,
    unsigned max_iterations,
    unsigned max_orientation_iterations,
    unsigned max_relocation_iterations)

{
    PlantGraph initial = from_delaunay(cloud, size);
    PlantGraph mst = minimum_spanning_tree(initial);

    find_root(mst, root_finder);

    for (unsigned i = 0; i < max_iterations; ++i) {
        mst = iteration_livny_et_al(mst, root_finder, max_orientation_iterations, max_relocation_iterations);
    }

    return mst;
}

}