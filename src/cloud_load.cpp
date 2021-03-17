#include <groot/tinyply.h>
#include <groot/cloud_load.hpp>
#include <fstream>

namespace groot {

std::vector<cgal::Point_3> load_PLY(const char* filename)
{
    std::ifstream file(filename);

    tinyply::PlyFile ply;
    ply.parse_header(file);

    
    std::shared_ptr<tinyply::PlyData> vertices = ply.request_properties_from_element("vertex", { "x", "y", "z" });
    ply.read(file);

    std::vector<cgal::Point_3> ret(vertices->count);
    std::copy_n((cgal::Point_3*)vertices->buffer.get(), vertices->count, ret.data());
    return ret;
}

}
