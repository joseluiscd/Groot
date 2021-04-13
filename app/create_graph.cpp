#include "create_graph.hpp"
#include <spdlog/spdlog.h>
#include "components.hpp"

CreateGraph::CreateGraph(entt::registry& _registry)
    : registry(_registry)
{
    target = registry.ctx<SelectedEntity>().selected;

    if (registry.valid(target) && registry.all_of<PointCloud>(target)) {
        cloud = &registry.get<PointCloud>(target);
    } else {
        throw std::runtime_error("Selected entity must have PointCloud and PointNormals");
    }
}

GuiState CreateGraph::draw_gui()
{
    bool show = true;
    ImGui::OpenPopup("Import PLY");
    if (ImGui::BeginPopupModal("Import PLY", &show, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {

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
            //All configured, run the command
            ImGui::EndPopup();
            return GuiState::RunAsync;
        }

        ImGui::EndPopup();
    }

    return show ? GuiState::Editing : GuiState::Close;
}

CommandState CreateGraph::execute()
{
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
        graph = groot::from_delaunay(cloud->cloud.data(), cloud->cloud.size());
        spdlog::info("Built 3D delaunay!");
    } else {
        groot::SearchParams params {
            .k = (int)k,
            .radius = (float)radius,
            .search = search_val
        };

        spdlog::info("Running neighbour search...");
        graph = groot::from_search(cloud->cloud.data(), cloud->cloud.size(), params);
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

    this->result = std::move(graph);

    return CommandState::Ok;
}

void CreateGraph::on_finish()
{
    if (this->result) {
        registry.emplace_or_replace<groot::PlantGraph>(target, std::move(*this->result));
    }
}

