#include <fstream>
#include <groot/skeleton.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <spdlog/spdlog.h>
#include <cfglib/cfg.hpp>
#include <cxxopts/cxxopts.hpp>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Config {
    std::string input_file = "input.ply";
    std::string output_file = "output.lgf";

    // Search
    std::string search_type = "Radius";
    long k = 0;
    double radius = 0.0;

    // Others
    bool geodesic = false;
};

void do_cfg(cfg::Cfg&& cfglib, Config& conf)
{
    cfglib.begin_global_section()
        .add_value("input", conf.input_file)
        .add_value("output", conf.output_file)
        .end_section()

        .begin_section("params")
        .add_value("search_type", conf.search_type)
        .add_value("k", conf.k)
        .add_value("radius", conf.radius)
        .end_section()

        .begin_section("other")
        .add_value("geodesic", conf.geodesic)
        .end_section()
        .run();
}

int main(int argc, char** argv)
{
    Config conf;
    cxxopts::Options options("create_graph", "Create a graph from a point cloud for tree processing");

    options.add_options()
        ("c,config", "Config file", cxxopts::value<std::string>()->default_value("config.toml"))
        ("g,generate", "Generate default config", cxxopts::value<bool>()->default_value("false"))
        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);
    std::string cfgfile = result["config"].as<std::string>();
    
    if (result["h"].as<bool>()) {
        fmt::print(options.help());
        return 0;
    }

    if (result["generate"].as<bool>())
    {
        do_cfg(cfg::write(cfgfile.data()), conf);

        return 0;
    }

    do_cfg(cfg::parse(cfgfile.data()), conf);


    Cloud::Ptr cloud(new Cloud());
    if (pcl::io::loadPLYFile(conf.input_file, *cloud) < 0) {
        spdlog::error("Could not open input file \"{}\"", conf.input_file);
        exit(1);
    }

    groot::SearchType search_val = groot::SearchType::kCount;
    if (conf.search_type == "Knn"){
        search_val = groot::SearchType::kKnnSearch;
    } else if (conf.search_type == "Radius") {
        search_val = groot::SearchType::kRadiusSearch;
    }
    
    groot::SearchParams params {
        .k = (int)conf.k,
        .radius = (float)conf.radius,
        .search = search_val
    };

    groot::PlantGraph graph = groot::PlantGraph::from_search(cloud, params);

    if (conf.geodesic) {
        graph.find_root().geodesic();
    }

    std::ofstream out_stream(conf.output_file);
    if (out_stream.is_open()) {
        graph.write_to_file(out_stream);
    } else {
        spdlog::error("Could not open output file \"{}\"", conf.output_file);
        exit(1);
    }
}