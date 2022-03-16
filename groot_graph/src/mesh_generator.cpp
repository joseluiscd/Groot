#include <boost/graph/depth_first_search.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <groot/glm_cgal.hpp>
#include <groot/mesh.hpp>
#include <groot/util.hpp>
#include <groot_graph/plant_graph.hpp>

namespace groot {

template <typename DistanceMap, typename VertexMap>
class AddVolumeVisitor : public boost::default_dfs_visitor {
public:
    AddVolumeVisitor(
        size_t _resolution,
        Mesh& _mesh,
        const PropertyMap<cgal::Vector_3>& _tangents,
        const PropertyMap<float>& _radii,
        PropertyMap<size_t>& _vertex_mesh_first_index)

        : resolution(_resolution)
        , mesh(_mesh)
        , tangents(_tangents)
        , radii(_radii)
        , vertex_mesh_first_index(_vertex_mesh_first_index)
    {
    }

    void tree_edge(Edge e, const PlantGraph& g)
    {
        Vertex current = boost::target(e, g);
        Vertex previous = boost::source(e, g);

        for (size_t resIdx = 0; resIdx < resolution; ++resIdx) {
            size_t baseIndex = current * resolution;

            mesh.indices.push_back(resIdx + baseIndex);
            mesh.indices.push_back((resIdx + 1) % resolution + baseIndex);
            mesh.indices.push_back(resIdx + resolution + baseIndex);
        }
    }

private:
    size_t resolution;
    Mesh& mesh;
    const PropertyMap<cgal::Vector_3>& tangents;
    const PropertyMap<float>& radii;
    PropertyMap<size_t>& vertex_mesh_first_index;
};

const size_t resolution = 32;
Mesh generate_mesh(const PlantGraph& graph, const PropertyMap<float>& radii, const PropertyMap<Vector_3>& tangents)
{
    size_t num_vertices = boost::num_vertices(graph);

    Mesh mesh {
        std::vector<Point_3>(resolution * num_vertices),
        std::vector<unsigned int>(),
        std::vector<Vector_3>(resolution * num_vertices),
    };

    for (auto [v_it, v_end] = boost::vertices(graph); v_it != v_end; ++v_it) {
        Vertex current = *v_it;

        Point_3 point = graph[current].position;
        Vector_3 tangent = tangents[current];
        float radius = radii[current];

        glm::vec3 g_p = to_glm(point);
        glm::vec3 u = glm::normalize(to_glm(Kernel::Plane_3(point, tangent).base1())) * radius;

        for (size_t rotIdx = 0; rotIdx < resolution; ++rotIdx) {
            glm::vec3 normal = glm::rotate(u, glm::radians(rotIdx * resolution / 360.0f), to_glm(tangent));
            glm::vec3 result = g_p + normal;
            normal = glm::normalize(normal);

            mesh.vertices.emplace_back(result.x, result.y, result.z);
            mesh.normals->emplace_back(normal.x, normal.y, normal.z);
        }
    }

    return mesh;
}

}