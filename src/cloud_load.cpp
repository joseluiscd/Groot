#include <groot/tinyply.h>
#include <groot/cloud_load.hpp>
#include <fstream>

namespace groot {

std::pair<std::vector<cgal::Point_3>, std::vector<cgal::Vector_3>> load_PLY(const char* filename)
{
    std::ifstream file(filename);

    tinyply::PlyFile ply;
    ply.parse_header(file);

    
    std::shared_ptr<tinyply::PlyData> vertices = ply.request_properties_from_element("vertex", { "x", "y", "z" });
    std::shared_ptr<tinyply::PlyData> normals;
    try {
        normals = ply.request_properties_from_element("vertex", {"nx", "ny", "nz"});
    } catch (const std::exception& e) {}

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
    ply.add_properties_to_element("vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, size, reinterpret_cast<uint8_t*>(cloud), tinyply::Type::INVALID, 0);
    ply.write(file, true);
}

void save_PLY(const char* filename, Point_3* cloud, Vector_3* normals, size_t size)
{
    std::ofstream file(filename, std::ios::binary);

    tinyply::PlyFile ply;
    ply.add_properties_to_element("vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, size, reinterpret_cast<uint8_t*>(cloud), tinyply::Type::INVALID, 0);
    ply.add_properties_to_element("vertex", {"nx", "ny", "nz"}, tinyply::Type::FLOAT32, size, reinterpret_cast<uint8_t*>(normals), tinyply::Type::INVALID, 0);
    ply.write(file, true);
}

}
