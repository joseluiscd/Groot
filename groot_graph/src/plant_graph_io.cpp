#include <fstream>
#include <groot_graph/plant_graph_io.hpp>
#include <spdlog/spdlog.h>

namespace groot {

groot::PlantGraph load_plant_graph(const char* filename)
{
    std::ifstream file(filename);
    std::string line;
    std::stringstream ss;
    groot::PlantGraph graph;

    float x, y, z;
    Vertex a, b;
    size_t n_vertices = 0;

    while (std::getline(file, line)) {
        if (line.substr(0, 2) == "v ") {
            ss.clear();
            ss << line.substr(2);
            ss >> x >> y >> z;

            Vertex v = boost::add_vertex(graph);
            graph[v].position = cgal::Point_3 { x, y, z };

            n_vertices++;
        } else if (line.substr(0, 2) == "l ") {
            ss.clear();
            ss << line.substr(2);
            ss >> a >> b;
            a--;
            b--;

            if (a < n_vertices && b < n_vertices) {
                auto [e, _] = boost::add_edge(a, b, graph);
                graph[e].length = std::sqrt(CGAL::squared_distance(
                    graph[a].position,
                    graph[b].position));
            }
        }
    }

    return graph;
}

void save_plant_graph(const char* filename, const groot::PlantGraph& plant)
{
    std::ofstream file(filename);
    file << "o tree\n";

    auto [v_it, v_end] = boost::vertices(plant);
    auto [e_it, e_end] = boost::edges(plant);

    for (; v_it != v_end; ++v_it) {
        float x = plant[*v_it].position.x();
        float y = plant[*v_it].position.y();
        float z = plant[*v_it].position.z();

        file << "v " << x << " " << y << " " << z << "\n";
    }

    auto vertex_ids = boost::get(boost::vertex_index, plant);
    for (; e_it != e_end; ++e_it) {
        Vertex a = boost::source(*e_it, plant);
        Vertex b = boost::target(*e_it, plant);

        size_t va = vertex_ids[a];
        size_t vb = vertex_ids[b];

        file << "l " << va + 1 << " " << vb + 1 << "\n";
    }
}

}