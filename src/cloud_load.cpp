#include <fstream>
#include <glm/glm.hpp>
#include <groot/cloud_load.hpp>
#include <groot/glm_cgal.hpp>
#include <groot/tinyply.h>

namespace groot {

CloudData load_PLY(const char* filename)
{
    static_assert(sizeof(glm::vec3) == (3 * sizeof(float)), "cgal::Vector_3 must be 3 floats without padding");
    static_assert(sizeof(glm::vec3) == (3 * sizeof(float)), "cgal::Point_3 must be 3 floats without padding");

    std::ifstream file(filename, std::ios::binary);

    tinyply::PlyFile ply;
    ply.parse_header(file);

    std::vector<cgal::Point_3> vertices;
    std::vector<cgal::Vector_3> normals;
    std::vector<glm::vec3> colors;

    std::shared_ptr<tinyply::PlyData> vertices_data = ply.request_properties_from_element("vertex", { "x", "y", "z" });
    std::shared_ptr<tinyply::PlyData> normals_data = ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
    std::shared_ptr<tinyply::PlyData> color_data = ply.request_properties_from_element("vertex", { "r", "g", "b" });
    std::shared_ptr<tinyply::PlyData> alt_color_data = ply.request_properties_from_element("vertex", { "red", "green", "blue" });
    ply.read(file);

    if (vertices_data != nullptr && vertices_data->t == tinyply::Type::FLOAT32) {
        glm::vec3* buffer = (glm::vec3*)vertices_data->buffer.get();
        to_point_iterator(buffer, buffer + vertices_data->count, std::back_inserter(vertices));
    }

    if (normals_data != nullptr && normals_data->t == tinyply::Type::FLOAT32) {
        glm::vec3* buffer = (glm::vec3*)normals_data->buffer.get();
        to_vector_iterator(buffer, buffer + normals_data->count, std::back_inserter(normals));
    }

    if (color_data == nullptr) {
        color_data = alt_color_data;
    }

    if (color_data != nullptr && color_data->t == tinyply::Type::UINT8) {
        glm::tvec3<uint8_t>* buffer = (glm::tvec3<uint8_t>*)color_data->buffer.get();
        std::transform(buffer, buffer + color_data->count, std::back_inserter(colors), [](const glm::tvec3<uint8_t>& current) {
            return glm::vec3(
                float(current.x) / 255.0f,
                float(current.y) / 255.0f,
                float(current.z) / 255.0f);
        });
    }

    CloudData result;
    if (!vertices.empty()) {
        result.points = std::move(vertices);

        if (!normals.empty()) {
            result.normals = std::move(normals);
        }

        if (!colors.empty()) {
            result.colors = std::move(colors);
        }
    }

    return result;
}

std::pair<std::vector<cgal::Point_3>, std::vector<cgal::Vector_3>> load_PLY_old(const char* filename)
{
    std::ifstream file(filename);

    tinyply::PlyFile ply;
    ply.parse_header(file);

    std::shared_ptr<tinyply::PlyData> vertices = ply.request_properties_from_element("vertex", { "x", "y", "z" });
    std::shared_ptr<tinyply::PlyData> normals;
    try {
        normals = ply.request_properties_from_element("vertex", { "nx", "ny", "nz" });
    } catch (const std::exception&) {
    }

    ply.read(file);

    std::vector<cgal::Point_3> ret_points(vertices->count);
    std::vector<cgal::Vector_3> ret_normals(normals ? normals->count : 0);

    std::copy_n((cgal::Point_3*)vertices->buffer.get(), vertices->count, ret_points.data());
    if (normals) {
        std::copy_n((cgal::Vector_3*)normals->buffer.get(), normals->count, ret_normals.data());
    }

    return std::make_pair(std::move(ret_points), std::move(ret_normals));
}

void save_PLY(const char* filename, Point_3* cloud, size_t size)
{
    std::ofstream file(filename, std::ios::binary);

    tinyply::PlyFile ply;
    ply.add_properties_to_element("vertex", { "x", "y", "z" }, tinyply::Type::FLOAT32, size, reinterpret_cast<uint8_t*>(cloud), tinyply::Type::INVALID, 0);
    ply.write(file, true);
}

void save_PLY(const char* filename, Point_3* cloud, Vector_3* normals, size_t size)
{
    std::ofstream file(filename, std::ios::binary);

    tinyply::PlyFile ply;
    ply.add_properties_to_element("vertex", { "x", "y", "z" }, tinyply::Type::FLOAT32, size, reinterpret_cast<uint8_t*>(cloud), tinyply::Type::INVALID, 0);
    ply.add_properties_to_element("vertex", { "nx", "ny", "nz" }, tinyply::Type::FLOAT32, size, reinterpret_cast<uint8_t*>(normals), tinyply::Type::INVALID, 0);
    ply.write(file, true);
}

}
