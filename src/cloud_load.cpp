#include <groot/tinyply.h>
#include <groot/cloud_load.hpp>
#include <fstream>

namespace groot {

std::vector<glm::vec3> load_PLY(const char* filename)
{
    std::ifstream file(filename);

    tinyply::PlyFile ply;
    ply.parse_header(file);

    
    std::shared_ptr<tinyply::PlyData> vertices = ply.request_properties_from_element("vertex", { "x", "y", "z" });
    ply.read(file);

    std::vector<glm::vec3> ret(vertices->count);
    std::copy_n((glm::vec3*)vertices->buffer.get(), vertices->count, ret.data());
    return ret;
}

}