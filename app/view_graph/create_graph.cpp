#include "create_graph.hpp"
#include "imfilebrowser.h"
#include <cfglib/cfg.hpp>
#include <cxxopts/cxxopts.hpp>
#include <fstream>
#include <groot/skeleton.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <spdlog/spdlog.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

CreateGraph::CreateGraph(std::function<void(groot::PlantGraph&&)> callback)
    : Operation(callback)
    , open(open_flags)
    , save(save_flags)
{
    open.SetTitle("Open PLY Cloud");
    open.SetTypeFilters({ ".ply" });

    save.SetTitle("Save Graph");
    save.SetTypeFilters({ ".lgf" });
}

void CreateGraph::window()
{
    ImGui::SetNextWindowSize(ImVec2(200, 200), ImGuiCond_FirstUseEver);

    ImGui::Begin("Create graph", &_show);

    if (ImGui::Button("Set Input File")) {
        open.Open();
    }
    ImGui::SameLine();
    ImGui::Text(input_file.c_str());

    if (ImGui::Button("Set Output File")) {
        save.Open();
    }
    ImGui::SameLine();
    ImGui::Text(output_file.c_str());

    ImGui::Separator();

    ImGui::Combo("Method", &selected_method, method_labels, kMethod_COUNT);

    ImGui::Separator();
    ImGui::Text("Parameters");

    switch (selected_method) {
    case kRadius:
        ImGui::InputDouble("Radius", &radius, 0.1, 0.5);
        ImGui::InputInt("Max neighbours", &k, 1, 5);
        break;

    case kKnn:
        ImGui::InputInt("k", &k, 1, 5);
        break;

    case kDelaunay:
        ImGui::Text("--None--");
        break;
    }

    ImGui::Separator();

    ImGui::Combo("Root find", &selected_root_find_method, root_find_labels, kRootFindMethod_COUNT);
    
    ImGui::Combo("Remove Loops", &selected_make_tree_method, make_tree_method_labels, kMakeTreeMethod_COUNT);


    ImGui::Separator();

    if (ImGui::Button("Run")) {
        if(ImGui::BeginPopupModal("running")) {
            ImGui::Text("Running...");
            ImGui::EndPopup();
        }

        run();
    }

    ImGui::End();

    open.Display();
    save.Display();

    if (open.HasSelected()) {
        input_file = open.GetSelected().string();
        open.ClearSelected();
    }

    if (save.HasSelected()) {
        output_file = save.GetSelected().string();
        open.ClearSelected();
    }
}

std::variant<groot::PlantGraph, std::string> CreateGraph::operation() const
{
    Cloud::Ptr cloud(new Cloud());

    spdlog::info("Loading PLY file...");

    if (pcl::io::loadPLYFile(input_file, *cloud) < 0) {
        spdlog::error("Could not open input file \"{}\"", input_file);
        return "Unable to open input file";
    }

    spdlog::info("Loaded PLY file!");

    bool is_delaunay = false;
    groot::SearchType search_val;

    switch (selected_method) {
    case kKnn:
        search_val = groot::SearchType::kKnnSearch;
        break;
    case kRadius:
        search_val = groot::SearchType::kRadiusSearch;
        break;
    case kDelaunay:
        is_delaunay = true;
        break;
    default:
        spdlog::error("Unknown method");
        return "Unknown method";
    }

    groot::PlantGraph graph;
    if (is_delaunay) {
        spdlog::info("Building 3D Delaunay...");
        graph = groot::PlantGraph::from_delaunay(cloud);
        spdlog::info("Built 3D delaunay!");
    } else {
        groot::SearchParams params {
            .k = (int)k,
            .radius = (float)radius,
            .search = search_val
        };
        
        spdlog::info("Running neighbour search...");
        graph = groot::PlantGraph::from_search(cloud, params);
        spdlog::info("Neighbour search done!");
    }

    spdlog::info("Finding root point...");
    switch (selected_root_find_method)
    {
    case kMinX:
        graph.find_root(groot::point_finder::MinX());
        break;
    case kMinY:
        graph.find_root(groot::point_finder::MinY());
        break;
    case kMinZ:
        graph.find_root(groot::point_finder::MinZ());
        break;
    case kMaxX:
        graph.find_root(groot::point_finder::MaxX());
        break;
    case kMaxY:
        graph.find_root(groot::point_finder::MaxY());
        break;
    case kMaxZ:
        graph.find_root(groot::point_finder::MaxZ());
        break;
    default:
        spdlog::error("Unrecognized root find method");
        return "Unknown find method";
    }
    spdlog::info("Found root point!");

    switch (selected_make_tree_method)
    {
    case kNone:
        break;
    case kMST:
        spdlog::info("Computing Minimum Spanning Tree...");
        graph.minimum_spanning_tree();
        spdlog::info("Computed Minimum Spanning Tree!");
        break;
    case kGeodesic:
        spdlog::info("Computing geodesic graph...");
        graph.geodesic();
        spdlog::info("Computed geodesic graph!");
        break;
    default:
        spdlog::error("Unrecognized loop remover");
        break;
    }

    if (output_file != "") {
        spdlog::info("Writing output file...");
        std::ofstream file(output_file);
        graph.write_to_file(file);
        spdlog::info("Writed output file...");
    }

    spdlog::info("Plant graph is created!");

    return graph;
}

/*
void do_cfg(cfg::Cfg&& cfglib, CreateGraph& conf)
{
    cfglib.begin_global_section()
        .add_value("input", conf.input_file)
        .add_value("output", conf.output_file)
        .end_section()

        .begin_section("params")
        .add_value("search_type", conf.selected_method)
        .add_value("k", conf.k)
        .add_value("radius", conf.radius)
        .end_section()

        .run();
}*/


/*
int main_asdf(int argc, char** argv)
{

    do_cfg(cfg::parse(cfgfile.data()), conf);



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
}*/