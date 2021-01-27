#include <fstream>
#include <groot/skeleton.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <spdlog/spdlog.h>
#include <groot/toml.hpp>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

int main()
{
    auto config = toml::parse_file("config.toml");

    std::string input_file = config["input"].value_or("input.ply");
    std::string output_file = config["output"].value_or("output.lgf");

    auto params = config["params"];
    std::string search_type = params["search_type"].value_or("RadiusSearch");
    long k = params["k"].value_or(0);
    double radius = params["radius"].value_or(0.0);

    Cloud::Ptr cloud(new Cloud());
    if (pcl::io::loadPLYFile(input_file, *cloud) < 0) {
        spdlog::error("Could not open input file \"{}\"", input_file);
        exit(1);
    }

    groot::SearchType search_val = groot::SearchType::kCount;
    if (search_type == "KnnSearch"){
        search_val = groot::SearchType::kKnnSearch;
    } else if (search_type == "RadiusSearch") {
        search_val = groot::SearchType::kRadiusSearch;
    }
    
    groot::SearchParams search_params {
        .k = (int)k,
        .radius = (float)radius,
        .search = search_val
    };

    groot::PlantGraph graph = groot::PlantGraph::from_search(cloud, search_params);

    std::ofstream out_stream(output_file);
    if (out_stream.is_open()) {
        graph.write_to_file(out_stream);
    } else {
        spdlog::error("Could not open output file \"{}\"", output_file);
        exit(1);
    }
}