#include "create_graph.hpp"
#include <fstream>
#include <gfx/imgui/imfilebrowser.h>
#include <groot/cloud_load.hpp>
#include <spdlog/spdlog.h>

const int open_flags = ImGuiFileBrowserFlags_CloseOnEsc;
const int save_flags = open_flags | ImGuiFileBrowserFlags_CreateNewDir | ImGuiFileBrowserFlags_EnterNewFilename;

CreateGraph::CreateGraph(IDataOutput<groot::PlantGraph>& _output)
    : output(_output)
    , open(open_flags)
{
    open.SetTitle("Open PLY Cloud");
    open.SetTypeFilters({ ".ply" });
}

GuiState CreateGraph::draw_gui()
{
    bool show = true;
    if (ImGui::BeginPopupModal("Import PLY", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        if (ImGui::Button("Set Input File")) {
            open.Open();
        }
        ImGui::SameLine();
        ImGui::Text("%s", input_file.c_str());

        ImGui::Separator();
        ImGui::Text("Topology build method");
        ImGui::RadioButton("Radius search", (int*)&selected_method, Method::kRadius);
        ImGui::RadioButton("kNN", (int*)&selected_method, Method::kKnn);
        ImGui::RadioButton("3D Delaunay", (int*)&selected_method, Method::kDelaunay);
        ImGui::Separator();

        ImGui::Text("Parameters");

        switch (selected_method) {
        case kRadius:
            ImGui::InputDouble("Radius", &radius, 0.1, 0.5);
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
            if (ImGui::BeginPopupModal("running")) {
                ImGui::Text("Running...");
                ImGui::EndPopup();
            }

            //All configured, run the command
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        open.Display();

        ImGui::EndPopup();
    }

    ImGui::OpenPopup("Create graph");

    if (open.HasSelected()) {
        input_file = open.GetSelected().string();
        open.ClearSelected();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState CreateGraph::execute()
{
    spdlog::info("Loading PLY file...");

    if (input_file.empty()) {
        error_string = "Cannot open file";
        return CommandState::Error;
    }

    std::vector<glm::vec3> cloud = groot::load_PLY(input_file.c_str());

    spdlog::info("Loaded PLY file with {} points!", cloud.size());

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
        error_string = "Unknown method";
        return CommandState::Error;
    }

    groot::PlantGraph graph;
    if (is_delaunay) {
        spdlog::info("Building 3D Delaunay...");
        graph = groot::from_delaunay(cloud.data(), cloud.size());
        spdlog::info("Built 3D delaunay!");
    } else {
        groot::SearchParams params {
            .k = (int)k,
            .radius = (float)radius,
            .search = search_val
        };

        spdlog::info("Running neighbour search...");
        graph = groot::from_search(cloud.data(), cloud.size(), params);
        spdlog::info("Neighbour search done!");
    }

    spdlog::info("Finding root point...");
    switch (selected_root_find_method) {
    case kMinX:
        groot::find_root(graph, groot::point_finder::MinX());
        break;
    case kMinY:
        groot::find_root(graph, groot::point_finder::MinY());
        break;
    case kMinZ:
        groot::find_root(graph, groot::point_finder::MinZ());
        break;
    case kMaxX:
        groot::find_root(graph, groot::point_finder::MaxX());
        break;
    case kMaxY:
        groot::find_root(graph, groot::point_finder::MaxY());
        break;
    case kMaxZ:
        groot::find_root(graph, groot::point_finder::MaxZ());
        break;
    default:
        error_string = "Unrecognized root find method";
        return CommandState::Error;
    }

    spdlog::info("Found root point!");

    switch (selected_make_tree_method) {
    case kGeodesic:
        graph = groot::geodesic(graph);
        break;
    case kMST:
        graph = groot::minimum_spanning_tree(graph);
        break;
    default:
        break;
    }

    spdlog::info("Plant graph is created!");

    output = std::move(graph);
    return CommandState::Ok;
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